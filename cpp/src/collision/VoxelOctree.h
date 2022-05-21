#ifndef VOXEL_OCTREE_H
#define VOXEL_OCTREE_H

#include <collision/Capsule.h>
#include <collision/Mesh.h>
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/detail/TreeNode.h>
#include <cpptoml/cpptoml.h>
#include <util/json_io.h>  // has forward-declaration of nlohmann::json

#include <functional>
#include <iostream>
#include <limits>    // for std::numeric_limits
#include <map>       // for std::map
#include <memory>    // for std::unique_ptr
#include <stack>     // for std::stack
#include <stdexcept> // for std::length_error
#include <tuple>     // for std::tuple
#include <utility>   // for std::pair
#include <variant>   // for std::variant

#include <cstddef> // for size_t
#include <cstdint> // for uint64_t

// forward declarations
namespace itk {
  template <typename PixelType, unsigned int> class Image;
  template <typename ImageType> class SmartPointer;
} // end of namespace itk


namespace collision {

/** Occupancy octree for voxelization of a collision space.
 *
 * This is a sparse representation of a voxelization.  The space is separated
 * into individual voxels which can be either empty or occupied.  This is
 * condensed into blocks of 4x4x4.  Each block is represented by a single
 * 64-bit unsigned integer, one bit per voxel to indicate occupied or not.
 *
 * Only the blocks that have occupied voxel cells are stored.  Those blocks
 * that are occupied are stored into a tree structure.  At each level, the
 * space is subdivided in each of the three dimensions, making eight
 * subdivisions.  That is where the name Octree comes from.  For collision
 * checking against a particular point, we recurse down the octant child until
 * we either get to a null pointer (meaning that whole subtree is empty space)
 * or a leaf node containing a single 64-bit unsigned integer.
 *
 * You can specify the limits of the space.  The bottom-left of each voxel
 * (i.e., toward the origin) is the coordinate of that voxel.  This is
 * different than many other voxel implementations where the center of the
 * voxel is the voxel coordinate.  However, having the center be the voxel
 * coordinate causes geometric changes when scaling up or down to a different
 * voxelization resultion.  No such problem occurs when using the bottom-left
 * corner as the voxel coordinate.  Instead the center of a voxel at (ix, iy, iz) is
 *
 *   Point voxel_center(
 *       xlim().first + (ix + 0.5) * dx(),
 *       ylim().first + (iy + 0.5) * dy(),
 *       zlim().first + (iz + 0.5) * dz()
 *       );
 *
 * Note: discretizations need to match for all dimensions and it needs to be a
 * power of 2 bigger than 4.  We may be able to remove this limitation later,
 * but for now, it seems to work well.
 */
class VoxelOctree {

public:
  using BlockType = uint64_t;
  using ConstBlockVisitor = std::function<void(size_t, size_t, size_t, uint64_t)>;
  using ModBlockVisitor = std::function<uint64_t(size_t, size_t, size_t, uint64_t)>;
  using ConstOccupiedVoxelVisitor = std::function<void(size_t, size_t, size_t)>;
  using ConstVoxelVisitor = std::function<void(size_t, size_t, size_t, bool)>;
  using ModVoxelVisitor = std::function<bool(size_t, size_t, size_t, bool)>;

  using ItkPixelType = unsigned char;
  using ItkImageType = itk::Image<ItkPixelType, 3>;
  using ItkImagePtr  = itk::SmartPointer<ItkImageType>;

public:
  /** upscales from Ndim to the next supported size
   *
   * If Ndim is larger than the largest supported size, then a
   * std::invalid_argument exception is thrown.
   */
  static size_t to_supported_size(size_t Ndim);
  static size_t largest_supported_size() { return 512; }

  explicit VoxelOctree(size_t Ndim = 4);
  VoxelOctree(const VoxelOctree &other) = default; // copy
  VoxelOctree(VoxelOctree &&other) = default;      // move

  VoxelOctree& operator= (const VoxelOctree &other) = default; // copy
  VoxelOctree& operator= (VoxelOctree &&other) = default;      // move

  bool operator== (const VoxelOctree &other) const;

  size_t Nx() const { return _N; }       // number of voxels in the x-direction
  size_t Ny() const { return _N; }       // number of voxels in the y-direction
  size_t Nz() const { return _N; }       // number of voxels in the z-direction
  size_t N()  const { return _N*_N*_N;}  // number of voxels

  size_t Nbx() const { return _N/4;        }  // number of blocks in the x-direction
  size_t Nby() const { return _N/4;        }  // number of blocks in the y-direction
  size_t Nbz() const { return _N/4;        }  // number of blocks in the z-direction
  size_t Nb () const { return _N*_N*_N/64; }  // number of blocks

  void copy_limits(const VoxelOctree &other);
  VoxelOctree empty_copy() const; // covers same space, but empty

  void set_xlim(double xmin, double xmax);
  void set_ylim(double ymin, double ymax);
  void set_zlim(double zmin, double zmax);

  void set_xlim(const std::pair<double, double> &lim);
  void set_ylim(const std::pair<double, double> &lim);
  void set_zlim(const std::pair<double, double> &lim);

  std::pair<double, double> xlim() const { return {_xmin, _xmax}; }
  std::pair<double, double> ylim() const { return {_ymin, _ymax}; }
  std::pair<double, double> zlim() const { return {_zmin, _zmax}; }

  Point lower_left()  const { return {_xmin, _ymin, _zmin}; }
  Point upper_right() const { return {_xmax, _ymax, _zmax}; }

  // size of cells
  double dx() const { return _dx; }
  double dy() const { return _dy; }
  double dz() const { return _dz; }

  // size of blocks
  double dbx() const { return _dx * 4; }
  double dby() const { return _dy * 4; }
  double dbz() const { return _dz * 4; }

  bool is_empty() const;  // means no voxels are on
  size_t nblocks() const; // number of occupied blocks
  size_t ncells() const;  // number of occupied cells

  bool is_in_domain(double x, double y, double z) const;

  Point voxel_center(size_t ix, size_t iy, size_t iz) const {
    return {
      _xmin + (ix + 0.5) * dx(),
      _ymin + (iy + 0.5) * dy(),
      _zmin + (iz + 0.5) * dz()
    };
  }

  Point block_center(size_t bx, size_t by, size_t bz) const {
    return {
      _xmin + (bx + 0.5) * dbx(),
      _ymin + (by + 0.5) * dby(),
      _zmin + (bz + 0.5) * dbz()
    };
  }

  // Note: bounds checks are NOT performed for performance reasons
  uint64_t block(size_t bx, size_t by, size_t bz) const;
  void set_block(size_t bx, size_t by, size_t bz, uint64_t value);

  // returns old block value before unioning or intersecting with value
  // Note: bounds checks are NOT performed for performance reasons
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t subtract_block(size_t bx, size_t by, size_t bz, uint64_t value);

  bool cell(size_t ix, size_t iy, size_t iz) const;

  // sets the cell's value
  // returns true if the cell's value changed
  bool set_cell(size_t ix, size_t iy, size_t iz, bool value = true);

  uint64_t find_block(double x, double y, double z) const;
  std::tuple<size_t, size_t, size_t> nearest_block_idx(double x, double y, double z) const; // bounds truncating
  std::tuple<size_t, size_t, size_t> find_block_idx(double x, double y, double z) const; // bounds checking
  std::tuple<size_t, size_t, size_t> nearest_cell(double x, double y, double z) const; // bounds truncating
  std::tuple<size_t, size_t, size_t> find_cell(double x, double y, double z) const; // bounds checking

  uint64_t find_block(const Point &p) const { return find_block(p[0], p[1], p[2]); }

  std::tuple<size_t, size_t, size_t> nearest_block_idx(const Point &p) const
  { return nearest_block_idx(p[0], p[1], p[2]); }

  std::tuple<size_t, size_t, size_t> find_block_idx(const Point &p) const
  { return find_block_idx(p[0], p[1], p[2]); }

  std::tuple<size_t, size_t, size_t> nearest_cell(const Point &p) const
  { return nearest_cell(p[0], p[1], p[2]); }

  std::tuple<size_t, size_t, size_t> find_cell(const Point &p) const
  { return find_cell(p[0], p[1], p[2]); }

  void add_point(double x, double y, double z);
  void add_point(const Point &p) { add_point(p[0], p[1], p[2]); }
  void add_line(const Point &a, const Point &b);
  void add_piecewise_line(const std::vector<Point> &line); // connected line segments
  void add_sphere(const Sphere &s);
  void add_capsule(const Capsule &c);
  void add_voxels(const VoxelOctree &other);
  //void remove_voxels(const VoxelOctree &other);

  void remove_interior_6neighbor();
  void remove_interior_27neighbor();
  void remove_interior(bool keep_diagonal = true) {
    if (keep_diagonal) { remove_interior_27neighbor(); }
    else               { remove_interior_6neighbor();  }
  }

  // dilate operations on voxel-level, regardless of dimension scale
  void dilate_6neighbor(int num = 1);
  void dilate_27neighbor(int num = 1);
  void dilate(int num = 1, bool use_diagonal = false) {
    if (use_diagonal) { dilate_27neighbor(num); }
    else              { dilate_6neighbor(num);  }
  }
  /** dilate based on a dimensional sphere
   * only marking voxel centers that intersect with the sphere
   */
  void dilate_sphere(double r); // := union(Sphere{r, p} for p in voxels)

  // erode operations on voxel-level, regardless of dimension scale
  void erode_6neighbor();
  void erode_27neighbor();
  void erode(bool use_diagonal = false) {
    if (use_diagonal) { erode_27neighbor(); }
    else              { erode_6neighbor();  }
  }
  /** erode based on a dimensional sphere
   * only keeping points where all voxel centers intersect with the sphere
   */
  void erode_sphere(double r); // := {p : Sphere{r, p} in voxels}

  bool collides(const Point &p) const;
  bool collides(const VoxelOctree &other) const;

  // an add method that is overloaded for different types
  void add(const Point &p) { add_point(p); }
  void add(const Sphere &s) { add_sphere(s); }
  void add(const Capsule &c) { add_capsule(c); }
  void add(const VoxelOctree &other) { add_voxels(other); }

  void remove_point(double x, double y, double z);
  void remove_point(const Point &p) { remove_point(p[0], p[1], p[2]); }
  void remove_voxels(const VoxelOctree &other);

  // a remove method that is overloaded for different types
  void remove(const Point &p) { remove_point(p); }
  void remove(const VoxelOctree &other) { remove_voxels(other); }

  void intersect_voxels(const VoxelOctree &other);

  // an intersect method that is overloaded for different types
  void intersect(const VoxelOctree &other) { intersect_voxels(other); }

  // Have a callback function at each occupied block
  void visit_leaves(const ConstBlockVisitor &visitor) const;

  // Have a callback function at each block, occupied or not.
  void visit_blocks(const ConstBlockVisitor &visitor) const;

  // Like visit_blocks() except the return value from visitor is the new block value
  void visit_modify_blocks(const ModBlockVisitor &visitor);

  // Have a callback frunction at each occupied voxel
  void visit_occupied_voxels(const ConstOccupiedVoxelVisitor &visitor) const;

  // Have a callback function at each voxel, occupied or not
  void visit_voxels(const ConstVoxelVisitor &visitor) const;

  // Like visit_voxels() except the visitor's return value is the new voxel value
  void visit_modify_voxels(const ModVoxelVisitor &visitor);

  /** Convert voxel object to a mesh
   *
   * This converts each voxel into triangles that represent the box shape of
   * the voxel.  It is not the typical idea of what it would mean to convert a
   * voxelized object to a surface mesh.
   */
  Mesh to_mesh() const;

  ItkImagePtr to_itk_image() const;

  static VoxelOctree from_itk_image(ItkImagePtr im);

  void to_nrrd(const std::string &fname, bool compress = true) const;
  static VoxelOctree from_nrrd(const std::string &fname);

  nlohmann::json to_json() const;
  static VoxelOctree from_json(const nlohmann::json &obj);

  std::shared_ptr<cpptoml::table> to_toml() const;
  static VoxelOctree from_toml(std::shared_ptr<cpptoml::table> table);

  // write and load based on file extension.  Throws if not a supported extension
  void to_file(const std::string &fname);
  static VoxelOctree from_file(const std::string &fname);

  // one in the given place, zeros everywhere else
  // for x, y, z within a block (each should be 0 <= x < 4)
  static uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z);

protected:
  void domain_check(double x, double y, double z) const;
  void limit_check(const VoxelOctree &other) const;

private:
  using TreeNodeVariant = std::variant<
      detail::TreeNode<  4>,
      detail::TreeNode<  8>,
      detail::TreeNode< 16>,
      detail::TreeNode< 32>,
      detail::TreeNode< 64>,
      detail::TreeNode<128>,
      detail::TreeNode<256>,
      detail::TreeNode<512>>;
  size_t _N;
  TreeNodeVariant _tree; // sparse voxel tree data
  double _xmin;
  double _xmax;
  double _ymin;
  double _ymax;
  double _zmin;
  double _zmax;
  double _dx;
  double _dy;
  double _dz;
};

} // end of namespace collision

#endif // VOXEL_OCTREE_H
