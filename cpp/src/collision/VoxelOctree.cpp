#include "collision/VoxelOctree.h"
#include "collision/collision.h"
#include "collision/collision_primitives.h"
#include "collision/stl_io.h"  // for some reuse of internal stl implementation
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <util/json_io.h>
#include <util/macros.h>
#include <util/openfile_check.h>
#include <util/string_ops.h>

#include <3rdparty/nlohmann/json.hpp>

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkNrrdImageIO.h>

#include <algorithm>
#include <bitset>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <iostream>

#define my_assert(val) if (!(val)) { throw std::runtime_error(#val); }

template <typename T> using NNType = ompl::NearestNeighborsGNATNoThreadSafety<T>;
using i8 =  int_fast8_t;
using u8 = uint_fast8_t;
using u64 = uint64_t;

namespace collision {

namespace {

inline void check_dims(size_t Na, size_t Nb) {
  if (Na != Nb) {
    std::ostringstream msg;
    msg << "voxel dimension mismatch ("
        << Na << " != " << Nb << ")";
    throw std::invalid_argument(msg.str());
  }
}

// only call function f if a and b are the same types
template <typename TreeNodeA, typename TreeNodeB, typename F>
inline void visit_both(TreeNodeA &a, TreeNodeB &b, const F &f) {
  std::visit([&f] (auto &_a, auto &_b) {
    using A = std::decay_t<decltype(_a)>;
    using B = std::decay_t<decltype(_b)>;
    if constexpr (std::is_same_v<A, B>) {
      f(_a, _b);
    }
  }, a, b);
}

// variant that returns a value with a default if the two types are not the same
template <typename TreeNodeA, typename TreeNodeB, typename F, typename D>
inline auto visit_both(TreeNodeA &a, TreeNodeB &b, const F &f, const D &default_val) {
  return std::visit([&f, &default_val] (auto &_a, auto &_b) {
    using A = std::decay_t<decltype(_a)>;
    using B = std::decay_t<decltype(_b)>;
    if constexpr (std::is_same_v<A, B>) {
      return f(_a, _b);
    } else {
      return default_val;
    }
  }, a, b);
}

} // end of unnamed namespace
  
size_t VoxelOctree::to_supported_size(size_t Ndim) {
  if      (Ndim <=   4) { return   4; }
  else if (Ndim <=   8) { return   8; }
  else if (Ndim <=  16) { return  16; }
  else if (Ndim <=  32) { return  32; }
  else if (Ndim <=  64) { return  64; }
  else if (Ndim <= 128) { return 128; }
  else if (Ndim <= 256) { return 256; }
  else if (Ndim <= 512) { return 512; }
  else {
    throw std::invalid_argument("too large for supported voxel octree: "
                                + std::to_string(Ndim));
  }
}

VoxelOctree::VoxelOctree(size_t Ndim) : _N(Ndim) {
  // create the TreeNode depending on Ndim
  switch (Ndim) {
    case   4: _tree.emplace<detail::TreeNode<  4>>(); break;
    case   8: _tree.emplace<detail::TreeNode<  8>>(); break;
    case  16: _tree.emplace<detail::TreeNode< 16>>(); break;
    case  32: _tree.emplace<detail::TreeNode< 32>>(); break;
    case  64: _tree.emplace<detail::TreeNode< 64>>(); break;
    case 128: _tree.emplace<detail::TreeNode<128>>(); break;
    case 256: _tree.emplace<detail::TreeNode<256>>(); break;
    case 512: _tree.emplace<detail::TreeNode<512>>(); break;
    default:
      throw std::invalid_argument("unsupported voxel dimension: "
                                  + std::to_string(Ndim));
  }
  set_xlim(0.0, 1.0);
  set_ylim(0.0, 1.0);
  set_zlim(0.0, 1.0);
}

bool VoxelOctree::operator== (const VoxelOctree &other) const {
  return _N    == other._N
      && _xmin == other._xmin
      && _ymin == other._ymin
      && _zmin == other._zmin
      && _xmax == other._xmax
      && _ymax == other._ymax
      && _zmax == other._zmax
      && _dx   == other._dx
      && _dy   == other._dy
      && _dz   == other._dz
      && visit_both(_tree, other._tree,
                    [] (const auto &a, const auto &b) { return a == b; },
                    false);
}

void VoxelOctree::copy_limits(const VoxelOctree &other) {
  this->_xmin = other._xmin;
  this->_xmax = other._xmax;
  this->_ymin = other._ymin;
  this->_ymax = other._ymax;
  this->_zmin = other._zmin;
  this->_zmax = other._zmax;
  this->_dx   = other._dx;
  this->_dy   = other._dy;
  this->_dz   = other._dz;
}

VoxelOctree VoxelOctree::empty_copy() const {
  VoxelOctree copy(this->_N);
  copy.copy_limits(*this);
  return copy;
}

void VoxelOctree::set_xlim(double xmin, double xmax) {
  if (xmin >= xmax) {
    throw std::length_error("xlimits must be positive in size");
  }
  _xmin = xmin;
  _xmax = xmax;
  _dx = (xmax - xmin) / Nx();
}

void VoxelOctree::set_ylim(double ymin, double ymax) {
  if (ymin >= ymax) {
    throw std::length_error("ylimits must be positive in size");
  }
  _ymin = ymin;
  _ymax = ymax;
  _dy = (ymax - ymin) / Ny();
}

void VoxelOctree::set_zlim(double zmin, double zmax) {
  if (zmin >= zmax) {
    throw std::length_error("zlimits must be positive in size");
  }
  _zmin = zmin;
  _zmax = zmax;
  _dz = (zmax - zmin) / Nz();
}

void VoxelOctree::set_xlim(const std::pair<double, double> &lim) {
  set_xlim(lim.first, lim.second);
}

void VoxelOctree::set_ylim(const std::pair<double, double> &lim) {
  set_ylim(lim.first, lim.second);
}

void VoxelOctree::set_zlim(const std::pair<double, double> &lim) {
  set_zlim(lim.first, lim.second);
}

bool VoxelOctree::is_empty() const {
  return std::visit([] (const auto &tree) { return tree.is_empty(); }, _tree);
}

size_t VoxelOctree::nblocks() const {
  return std::visit([] (const auto &tree) { return tree.nblocks(); }, _tree);
}

size_t VoxelOctree::ncells() const {
  size_t count = 0;
  visit_leaves([&count](size_t bx, size_t by, size_t bz, u64 val) {
    UNUSED_VAR(bx);
    UNUSED_VAR(by);
    UNUSED_VAR(bz);
    std::bitset<64> bits(val);
    count += bits.count();
  });
  return count;
}

u64 VoxelOctree::block(size_t bx, size_t by, size_t bz) const {
  return std::visit(
      [bx, by, bz] (const auto &tree) { return tree.block(bx, by, bz); },
      _tree);
}

void VoxelOctree::set_block(size_t bx, size_t by, size_t bz, u64 value) {
  std::visit(
      [bx, by, bz, value] (auto &tree) { tree.set_block(bx, by, bz, value); },
      _tree);
}

// returns old block type before unioning with value
u64 VoxelOctree::union_block(size_t bx, size_t by, size_t bz, u64 value) {
  if (value) {
    return std::visit(
        [bx, by, bz, value] (auto &tree) {
          return tree.union_block(bx, by, bz, value);
        }, _tree);
  } else {
    return this->block(bx, by, bz);
  }
}

u64 VoxelOctree::intersect_block(size_t bx, size_t by, size_t bz, u64 value) {
  return std::visit(
      [bx, by, bz, value] (auto &tree) {
        return tree.intersect_block(bx, by, bz, value);
      }, _tree);
}

u64 VoxelOctree::subtract_block(size_t bx, size_t by, size_t bz, u64 value) {
  return std::visit(
      [bx, by, bz, value] (auto &tree) {
        return tree.intersect_block(bx, by, bz, ~value);
      }, _tree);
}

bool VoxelOctree::cell(size_t ix, size_t iy, size_t iz) const {
  auto b = block(ix / 4, iy / 4, iz / 4);
  return b && (b & bitmask(ix % 4, iy % 4, iz % 4));
}

// sets the cell's value
// returns true if the cell's value changed
bool VoxelOctree::set_cell(size_t ix, size_t iy, size_t iz, bool value) {
  auto mask = bitmask(ix % 4, iy % 4, iz % 4);
  if (value) {
    auto oldval = this->union_block(ix / 4, iy / 4, iz / 4, mask);
    return oldval & mask;
  } else {
    auto oldval = this->intersect_block(ix / 4, iy / 4, iz / 4, ~mask);
    return oldval & ~mask;
  }
}

u64 VoxelOctree::find_block(double x, double y, double z) const {
  auto [bx, by, bz] = find_block_idx(x, y, z);
  return block(bx, by, bz);
}

std::tuple<size_t, size_t, size_t> VoxelOctree::nearest_block_idx(
    double x, double y, double z) const
{
  int ix = ((x - _xmin) / _dx);
  int iy = ((y - _ymin) / _dy);
  int iz = ((z - _zmin) / _dz);
  // truncate to nearest block
  size_t bx = std::min(Nbx()-1, size_t(std::max(0, ix / 4)));
  size_t by = std::min(Nby()-1, size_t(std::max(0, iy / 4)));
  size_t bz = std::min(Nbz()-1, size_t(std::max(0, iz / 4)));
  return {bx, by, bz};
}

std::tuple<size_t, size_t, size_t> VoxelOctree::find_block_idx(
    double x, double y, double z) const
{
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix / 4, iy / 4, iz / 4};
}

std::tuple<size_t, size_t, size_t> VoxelOctree::nearest_cell(
    double x, double y, double z) const
{
  int ix = ((x - _xmin) / _dx);
  int iy = ((y - _ymin) / _dy);
  int iz = ((z - _zmin) / _dz);
  // truncate to nearest cell
  return {
    std::min(Nx()-1, size_t(std::max(0, ix))),
    std::min(Ny()-1, size_t(std::max(0, iy))),
    std::min(Nz()-1, size_t(std::max(0, iz))),
  };
}

std::tuple<size_t, size_t, size_t> VoxelOctree::find_cell(
    double x, double y, double z) const
{
  domain_check(x, y, z);
  size_t ix = size_t((x - _xmin) / _dx);
  size_t iy = size_t((y - _ymin) / _dy);
  size_t iz = size_t((z - _zmin) / _dz);
  return {ix, iy, iz};
}

void VoxelOctree::add_point(double x, double y, double z) {
  if (!is_in_domain(x, y, z)) { return; }
  auto [ix, iy, iz] = nearest_cell(x, y, z);
  set_cell(ix, iy, iz);
}

void VoxelOctree::add_line(const Point &a, const Point &b) {
  // this implementation is adapted from
  //   "A Fast Voxel Traaversal Algorithm for Ray Tracing"
  //   by John Amanatides and Andrew Woo

  const Point ll = lower_left();
  const Point ur = upper_right();
  if (!segment_aabox_intersect(a, b, ll, ur)) {
    return; // line does not intersect with the voxel workspace
  }

  // beginning and ending voxel locations for endpoints (may be outside)
  // Convert a and b into the voxel coordinate frame (an affine transform)
  const Point nvoxels_per_meter {1/dx(), 1/dy(), 1/dz()};
  const Point A = (a - ll).cwiseProduct(nvoxels_per_meter);
  const Point B = (b - ll).cwiseProduct(nvoxels_per_meter);

  // voxel indices (may be outside)
  // this is like floor, but not checking FP stuff
  // int() rounds down for positive numbers and rounds up for negative numbers,
  // so we subtract one for negative numbers to compensate.
  const int Axi = int(A[0]) - (A[0] < 0);
  const int Ayi = int(A[1]) - (A[1] < 0);
  const int Azi = int(A[2]) - (A[2] < 0);
  const int Bxi = int(B[0]) - (B[0] < 0);
  const int Byi = int(B[1]) - (B[1] < 0);
  const int Bzi = int(B[2]) - (B[2] < 0);

  auto idx_is_in = [N = int(Nx())](int x) { return 0 <= x && x < N; };
  auto voxel_is_in = [&idx_is_in](int x, int y, int z) {
    return idx_is_in(x) && idx_is_in(y) && idx_is_in(z);
  };

  // add the endpoints
  // TODO: move A to be the first intersecting point, if not inside
  bool entered = voxel_is_in(Axi, Ayi, Azi);
  if (entered)                    { set_cell(Axi, Ayi, Azi); }
  if (voxel_is_in(Bxi, Byi, Bzi)) { set_cell(Bxi, Byi, Bzi); }

  // step directions, +1 for positive step, -1 for negative step
  const Point U    = (B - A).normalized(); // direction unit vector from A to B
  const int step_x = 1 - 2*(U[0] < 0);
  const int step_y = 1 - 2*(U[1] < 0);
  const int step_z = 1 - 2*(U[2] < 0);

  // starting voxel distance in each direction to the nearst voxel boundary
  const double ex = std::abs(A[0] - (Axi + step_x) * dx());
  const double ey = std::abs(A[1] - (Ayi + step_y) * dy());
  const double ez = std::abs(A[2] - (Azi + step_z) * dz());

  // we handle dimensions of U that are close to zero
  const Point Uabs = U.cwiseAbs();
  const double threshold = 1e-10;
  const bool x_valid = Uabs[0] > threshold;
  const bool y_valid = Uabs[1] > threshold;
  const bool z_valid = Uabs[2] > threshold;

  // full voxel step sizes in each direction
  // If the dimension is invalid, mark it as a really big step
  // so that we do at most one step in that direction
  const double tx_delta = x_valid ? 1/Uabs[0] : 1/threshold;
  const double ty_delta = y_valid ? 1/Uabs[1] : 1/threshold;
  const double tz_delta = z_valid ? 1/Uabs[2] : 1/threshold;

  // current position along the line to the next voxel in each direction
  double tx = std::abs(ex * tx_delta);
  double ty = std::abs(ey * ty_delta);
  double tz = std::abs(ez * tz_delta);

  // current voxel location
  int xi = Axi;
  int yi = Ayi;
  int zi = Azi;

  while (   step_x * (Bxi - xi) >= 0
         && step_y * (Byi - yi) >= 0
         && step_z * (Bzi - zi) >= 0)
  {
    // step in the direction that is closest to crossing a boundary
    // Note: reuse (tx < ty) to enable compiler optimizations
    const bool tx_is_min =  (tx < ty) && (tx < tz);
    const bool ty_is_min = !(tx < ty) && (ty < tz);
    if (tx_is_min) {
      xi += step_x;
      if (entered && !idx_is_in(xi)) { break; }
      tx += tx_delta;

    } else if (ty_is_min) {
      yi += step_y;
      if (entered && !idx_is_in(yi)) { break; }
      ty += ty_delta;

    } else { // tz is min
      zi += step_z;
      if (entered && !idx_is_in(zi)) { break; }
      tz += tz_delta;
    }

    if (!entered && voxel_is_in(xi, yi, zi)) { entered = true; }
    if (entered) { set_cell(xi, yi, zi); }
  }
}

void VoxelOctree::add_piecewise_line(const std::vector<Point> &line) {
  for (size_t i = 1; i < line.size(); i++) {
    this->add_line(line[i-1], line[i]);
  }
}

void VoxelOctree::add_sphere(const Sphere &s) {
  auto voxel_ctr_is_in_sphere = [&s, this]
    (size_t _ix, size_t _iy, size_t _iz) {
      double x = this->_xmin + this->_dx * (_ix + 0.5);
      double y = this->_ymin + this->_dy * (_iy + 0.5);
      double z = this->_zmin + this->_dz * (_iz + 0.5);
      return collision::collides(s, Point{x, y, z});
    };

  add_point(s.c);

  // bounding box
  Point ll = s.c - Point{s.r, s.r, s.r};
  Point tr = s.c + Point{s.r, s.r, s.r};
  auto [block_llx, block_lly, block_llz] = nearest_block_idx(ll[0], ll[1], ll[2]);
  auto [block_trx, block_try, block_trz] = nearest_block_idx(tr[0], tr[1], tr[2]);

  // check all blocks within that bounding box
  for (size_t bx = block_llx; bx <= block_trx; bx++) {
    for (size_t by = block_lly; by <= block_try; by++) {
      for (size_t bz = block_llz; bz <= block_trz; bz++) {
        u64 b = 0;
        for (u8 i = 0; i < 4; i++) {
          for (u8 j = 0; j < 4; j++) {
            for (u8 k = 0; k < 4; k++) {
              if (voxel_ctr_is_in_sphere((bx<<2) + i, (by<<2) + j, (bz<<2) + k)) {
                b |= bitmask(i, j, k);
              }
            }
          }
        }
        if (b) { union_block(bx, by, bz, b); }
      }
    }
  }
}

void VoxelOctree::add_capsule(const Capsule &c) {
  auto voxel_ctr_is_in_capsule = [&c, this]
    (size_t _ix, size_t _iy, size_t _iz) {
      double x = this->_xmin + this->_dx * (_ix + 0.5);
      double y = this->_ymin + this->_dy * (_iy + 0.5);
      double z = this->_zmin + this->_dz * (_iz + 0.5);
      return collision::collides(c, Point{x, y, z});
    };

  add_point(c.a);
  add_point(c.b);

  // bounding box
  Point ll = Point{
      std::min(c.a[0], c.b[0]) - c.r,
      std::min(c.a[1], c.b[1]) - c.r,
      std::min(c.a[2], c.b[2]) - c.r
    };
  Point tr = Point{
      std::max(c.a[0], c.b[0]) + c.r,
      std::max(c.a[1], c.b[1]) + c.r,
      std::max(c.a[2], c.b[2]) + c.r
    };
  auto [block_llx, block_lly, block_llz] = nearest_block_idx(ll[0], ll[1], ll[2]);
  auto [block_trx, block_try, block_trz] = nearest_block_idx(tr[0], tr[1], tr[2]);

  // check all blocks within that bounding box
  for (size_t bx = block_llx; bx <= block_trx; bx++) {
    for (size_t by = block_lly; by <= block_try; by++) {
      for (size_t bz = block_llz; bz <= block_trz; bz++) {
        u64 b = 0;
        for (u8 i = 0; i < 4; i++) {
          for (u8 j = 0; j < 4; j++) {
            for (u8 k = 0; k < 4; k++) {
              if (voxel_ctr_is_in_capsule((bx<<2) + i, (by<<2) + j, (bz<<2) + k)) {
                b |= bitmask(i, j, k);
              }
            }
          }
        }
        if (b) { union_block(bx, by, bz, b); }
      }
    }
  }
}

void VoxelOctree::add_voxels(const VoxelOctree &other) {
  check_dims(_N, other._N);
  visit_both(_tree, other._tree,
             [] (auto &a, const auto &b) { a.union_tree(b); });
}

/** For solid objects, removes the interior.
 *
 * Basically, this simply removes any occupied cells that are completely
 * surrounded by other occupied cells.  So, only leaves the occupied cells
 * that have an empty neighbor.  Neighbors are only the six directly adjacent
 * cells.
 *
 * For sparse voxel objects, this should reduce the number of occupied cells
 * and hopefully blocks, thus reducing memory.
 */
void VoxelOctree::remove_interior_6neighbor() {
  std::visit([this] (auto &tree) {
    auto copy(tree);

    // iterate over copy and modify _tree
    auto visitor = [this, &tree, &copy]
                   (size_t bx, size_t by, size_t bz, u64 old_b)
    {

      my_assert(old_b == copy.block(bx, by, bz));

      const u64 full = ~u64(0);
      auto new_b = old_b;
      const u64 left   = (bx <= 0)       ? full : copy.block(bx-1, by, bz);
      const u64 right  = (bx >= Nbx()-1) ? full : copy.block(bx+1, by, bz);
      const u64 front  = (by <= 0)       ? full : copy.block(bx, by-1, bz);
      const u64 behind = (by >= Nby()-1) ? full : copy.block(bx, by+1, bz);
      const u64 below  = (bz <= 0)       ? full : copy.block(bx, by, bz-1);
      const u64 above  = (bz >= Nbz()-1) ? full : copy.block(bx, by, bz+1);

      auto is_interior =
        [left, right, front, behind, below, above, old_b, this]
        (unsigned ix, unsigned iy, unsigned iz) {
          bool is_in = true;
          u64 mask = this->bitmask(ix, iy, iz);

          if (ix > 0) { mask |= this->bitmask(ix-1, iy, iz); }
          else { is_in = (is_in && bool(left & this->bitmask(3, iy, iz))); }

          if (ix < 3) { mask |= this->bitmask(ix+1, iy, iz); }
          else { is_in = (is_in && bool(right & this->bitmask(0, iy, iz))); }

          if (iy > 0) { mask |= this->bitmask(ix, iy-1, iz); }
          else { is_in = (is_in && bool(front & this->bitmask(ix, 3, iz))); }

          if (iy < 3) { mask |= this->bitmask(ix, iy+1, iz); }
          else { is_in = (is_in && bool(behind & this->bitmask(ix, 0, iz))); }

          if (iz > 0) { mask |= this->bitmask(ix, iy, iz-1); }
          else { is_in = (is_in && bool(below & this->bitmask(ix, iy, 3))); }

          if (iz < 3) { mask |= this->bitmask(ix, iy, iz+1); }
          else { is_in = (is_in && bool(above & this->bitmask(ix, iy, 0))); }

          is_in = (is_in && (mask == (old_b & mask)));
          return is_in;
        };

      for (unsigned ix = 0; ix < 4; ix++) {
        for (unsigned iy = 0; iy < 4; iy++) {
          for (unsigned iz = 0; iz < 4; iz++) {
            if (is_interior(ix, iy, iz)) {
              new_b &= ~bitmask(ix, iy, iz);
            }
          }
        }
      }

      // store this new block
      tree.set_block(bx, by, bz, new_b);
    };
    copy.visit_leaves(visitor);
  }, _tree);
}

/** For solid objects, removes the interior.
 *
 * Basically, this simply removes any occupied cells that are completely
 * surrounded by other occupied cells.  So, only leaves the occupied cells
 * that have an empty neighbor.  Neighbors are the six directly adjacent
 * cells plus the 8 diagonally adjacent cells.
 *
 * For sparse voxel objects, this should reduce the number of occupied cells
 * and hopefully blocks, thus reducing memory.
 */
void VoxelOctree::remove_interior_27neighbor() {
  std::visit([this] (auto &tree) {
    auto copy(tree);

    // iterate over copy and modify _tree
    auto visitor = [this, &tree, &copy]
                   (size_t bx, size_t by, size_t bz, u64 old_b)
    {

      my_assert(old_b == copy.block(bx, by, bz));

      const u64 full = ~u64(0);
      u64 block_neighborhood[3][3][3];
      for (int i = 0; i < 3; ++i) {
        long x = long(bx) - 1 + i;
        for (int j = 0; j < 3; ++j) {
          long y = long(by) - 1 + j;
          for (int k = 0; k < 3; ++k) {
            long z = long(bz) - 1 + k;
            // if out of bounds, mark as full
            if (x < 0 || long(Nbx())-1 < x ||
                y < 0 || long(Nby())-1 < y ||
                z < 0 || long(Nbz())-1 < z)
            {
              block_neighborhood[i][j][k] = full;
            // if the center, use the given value
            } else if (i == 1 && j == 1 && k == 1) {
              block_neighborhood[i][j][k] = old_b;
            // otherwise, get that block
            } else {
              block_neighborhood[i][j][k] = copy.block(x, y, z);
            }
          }
        }
      }

      auto new_b = old_b;

      // get the neighbor at the index relative to the current block (from -1 to 4)
      auto neighbor = [this, &block_neighborhood] (int ix, int iy, int iz) {
        int nx = 1, ny = 1, nz = 1; // block neighbor indices from block_neighborhood

        if      (ix == -1) { ix = 3; nx = 0; }
        else if (ix ==  4) { ix = 0; nx = 2; }
        if      (iy == -1) { iy = 3; ny = 0; }
        else if (iy ==  4) { iy = 0; ny = 2; }
        if      (iz == -1) { iz = 3; nz = 0; }
        else if (iz ==  4) { iz = 0; nz = 2; }

        return block_neighborhood[nx][ny][nz] & this->bitmask(ix, iy, iz);
      };

      auto is_interior = [&neighbor]
        (unsigned ix, unsigned iy, unsigned iz) {
          for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
              for (int k = -1; k <= 1; ++k) {
                if (!neighbor(ix + i, iy + j, iz + k)) {
                  return false;
                }
              }
            }
          }
          return true;
        };

      for (unsigned ix = 0; ix < 4; ix++) {
        for (unsigned iy = 0; iy < 4; iy++) {
          for (unsigned iz = 0; iz < 4; iz++) {
            if (is_interior(ix, iy, iz)) {
              new_b &= ~bitmask(ix, iy, iz);
            }
          }
        }
      }

      // store this new block
      tree.set_block(bx, by, bz, new_b);
    };
    copy.visit_leaves(visitor);
  }, _tree);
}

namespace {

template <typename VisitorType>
void dilate_one_impl(VoxelOctree &tree, const VisitorType &visit_cell_neighbors,
                     int num)
{
  if (num <= 0) { return; }

  // iterate over copy and modify _tree
  auto tree_visitor = [&tree, &visit_cell_neighbors, &num]
                      (size_t bx, size_t by, size_t bz, u64 old_b)
  {
    const auto n = std::min(num, 4);

    /// lookup table set.  In set means stored depth is greater than current
    struct DepthSet {
      u8  depths[12][12][12] {}; // record depths of visiting for search
      u64 voxels[3][3][3] {};    // local voxels

      bool in(u8 x, u8 y, u8 z, u8 d) { return d == 0 || d <= depths[x][y][z]; }
      bool add(u8 x, u8 y, u8 z, u8 d) { // return true if updated
        if (!in(x, y, z, d)) {
          depths[x][y][z] = d;
          const auto nx = x / 4, ny = y / 4, nz = z / 4;
          const auto lx = x % 4, ly = y % 4, lz = z % 4;
          voxels[nx][ny][nz] |= VoxelOctree::bitmask(lx, ly, lz);
          return true;
        }
        return false;
      }
    };
    DepthSet visited;
    auto &block_neighborhood = visited.voxels;

    // depth-first-search expansion, only up to four levels
    // the (x, y, z) is to be expanded to depth d
    // note, the forward declaration in order to use recursion
    std::function<void(u8,u8,u8,u8)> dfs_visitor;
    dfs_visitor = [&](u8 x, u8 y, u8 z, u8 d) {
      if (!visited.add(x, y, z, d)) { return; }
      auto sub_visit = [d, &dfs_visitor](u8 x, u8 y, u8 z) {
        dfs_visitor(x, y, z, d-1);
      };
      visit_cell_neighbors(sub_visit, x, y, z);
    };

    for (u8 x = 0; x < 4; ++x) {
      for (u8 y = 0; y < 4; ++y) {
        for (u8 z = 0; z < 4; ++z) {
          if (old_b & VoxelOctree::bitmask(x, y, z)) {
            dfs_visitor(x+4, y+4, z+4, n+1);
          }
        }
      }
    }

    for (i8 nx = 0; nx < 3; ++nx) {
      const auto bxi = bx + nx - 1;
      if (bxi < 0 || bxi >= tree.Nbx()) { continue; }
      for (i8 ny = 0; ny < 3; ++ny) {
        const auto byi = by + ny - 1;
        if (byi < 0 || byi >= tree.Nby()) { continue; }
        for (i8 nz = 0; nz < 3; ++nz) {
          const auto bzi = bz + nz - 1;
          if (bzi < 0 || bzi >= tree.Nbz()) { continue; }
          tree.union_block(bxi, byi, bzi, block_neighborhood[nx][ny][nz]);
        }
      }
    }
  };

  // do four dilations at a time
  for (; num > 0; num -= 4) {
    auto copy(tree);
    copy.visit_leaves(tree_visitor);
  }
}

} // end of unnamed namespace

void VoxelOctree::dilate_6neighbor(int num) {
  auto visit_cell_neighbors =
      [](const auto &set_neighbor, auto x, auto y, auto z) {
        set_neighbor(x-1, y, z);
        set_neighbor(x+1, y, z);
        set_neighbor(x, y-1, z);
        set_neighbor(x, y+1, z);
        set_neighbor(x, y, z-1);
        set_neighbor(x, y, z+1);
      };
  dilate_one_impl(*this, visit_cell_neighbors, num);
}

void VoxelOctree::dilate_27neighbor(int num) {
  auto visit_cell_neighbors =
      [&num](const auto &set_neighbor, auto x, auto y, auto z) {
        set_neighbor( x ,  y ,  z );
        set_neighbor(x-1,  y ,  z );
        set_neighbor(x+1,  y ,  z );
        set_neighbor( x , y-1,  z );
        set_neighbor(x-1, y-1,  z );
        set_neighbor(x+1, y-1,  z );
        set_neighbor( x , y+1,  z );
        set_neighbor(x-1, y+1,  z );
        set_neighbor(x+1, y+1,  z );
        set_neighbor( x ,  y , z-1);
        set_neighbor(x-1,  y , z-1);
        set_neighbor(x+1,  y , z-1);
        set_neighbor( x , y-1, z-1);
        set_neighbor(x-1, y-1, z-1);
        set_neighbor(x+1, y-1, z-1);
        set_neighbor( x , y+1, z-1);
        set_neighbor(x-1, y+1, z-1);
        set_neighbor(x+1, y+1, z-1);
        set_neighbor( x ,  y , z+1);
        set_neighbor(x-1,  y , z+1);
        set_neighbor(x+1,  y , z+1);
        set_neighbor( x , y-1, z+1);
        set_neighbor(x-1, y-1, z+1);
        set_neighbor(x+1, y-1, z+1);
        set_neighbor( x , y+1, z+1);
        set_neighbor(x+1, y+1, z+1);
        set_neighbor(x+1, y+1, z+1);
      };
  dilate_one_impl(*this, visit_cell_neighbors, num);
}

void VoxelOctree::dilate_sphere(double r) {
  /* Attempt #1
   * yeah... this approach was waaay too slow, but correct
   *
   * // very simple implementation that is probably very inneficient
   * auto copy(*this);
   * copy.visit_leaves(
   *   [this, r](size_t bx, size_t by, size_t bz, u64 old_b) {
   *     for (u8 x = 0; x < 4; ++x) {
   *       for (u8 y = 0; y < 4; ++y) {
   *         for (u8 z = 0; z < 4; ++z) {
   *           if (old_b & VoxelOctree::bitmask(x, y, z)) {
   *             const size_t ix = 4*bx + x;
   *             const size_t iy = 4*by + y;
   *             const size_t iz = 4*bz + z;
   *             this->add_sphere({this->voxel_center(ix, iy, iz), r});
   *           }
   *         }
   *       }
   *     }
   *   });
   * return;
   */

  /* Attempt #2
   * This didn't work for some reason...  It seemed to not do anything
   * // Attempt #2
   * 
   * // construct a nearest-neighbors data structure with block centers
   * // for each block:
   * //   make nearest neighbor of voxels with blocks within r (+ funge)
   * //   for each voxel in block:
   * //     if nearest voxel neighbor is within r of center
   * //       set to true

   * // construct the block nearest neighbor structure
   * struct Neighbor {
   *   Point c;
   *   size_t bx, by, bz;
   *   uint64_t val;

   *   bool operator==(const Neighbor &n) const {
   *     return bx == n.bx && by == n.by && bz == n.bz;
   *   }
   *   bool operator!=(const Neighbor &n) const { return !(*this == n); }
   * };
   * 
   * NNType<Neighbor> block_nn;
   * NNType<Neighbor> voxel_nn;
   * auto dist = [](auto &a, auto &b) { return (b.c - a.c).norm(); };
   * block_nn.setDistanceFunction(dist);
   * voxel_nn.setDistanceFunction(dist);

   * // populate block_nn
   * visit_leaves(
   *     [this, &block_nn](size_t bx, size_t by, size_t bz, uint64_t val) {
   *       block_nn.add(
   *           Neighbor{this->block_center(bx, by, bz), bx, by, bz, val});
   *     });

   * // equivalent distance between blocks
   * const double delta = Point{dx(), dy(), dz()}.norm();
   * const double funge = 3*delta;

   * // visit each block, setting the block based on the nearest neighbors
   * visit_modify_blocks(
   *     [this, &block_nn, funge, &voxel_nn, r, &dist]
   *     (size_t bx, size_t by, size_t bz, uint64_t val)
   *     {
   *       const uint64_t full = ~uint64_t(0);

   *       // if it's already full, nothing to do
   *       if (val == full) { return val; }

   *       Neighbor bnode{this->block_center(bx, by, bz), bx, by, bz, val};
   *       std::vector<Neighbor> neighbors;
   *       block_nn.nearestR(bnode, r + funge, neighbors); // assume sorted

   *       // nothing to do if no neighbors
   *       if (neighbors.empty()) {
   *         return val;
   *       }

   *       // check to see if we can just mark the whole block as full
   *       if (r >= funge + dist(bnode, neighbors[0])) {
   *         return full;
   *       }

   *       // populate the voxel nearest neighbor structure
   *       voxel_nn.clear();
   *       for (auto &n : neighbors) {
   *         for (u8 x = 0; x < 4; x++) {
   *           const size_t ix = 4*n.bx + x;
   *           for (u8 y = 0; y < 4; y++) {
   *             const size_t iy = 4*n.by + y;
   *             for (u8 z = 0; z < 4; z++) {
   *               const size_t iz = 4*n.bz + z;
   *               if (n.val & bitmask(x, y, z)) {
   *                 Neighbor vnode {this->voxel_center(ix, iy, iz), ix, iy, iz, 1};
   *                 // check if we can just mark the full block
   *                 if (r >= dist(vnode, bnode) + funge/2) { return full; }
   *                 voxel_nn.add(std::move(vnode));
   *               }
   *             }
   *           }
   *         }
   *       }

   *       // for voxels in this block, see if there are nearby voxels
   *       for (u8 x = 0; x < 4; x++) {
   *         const size_t ix = 4*bx + x;
   *         for (u8 y = 0; y < 4; y++) {
   *           const size_t iy = 4*by + y;
   *           for (u8 z = 0; z < 4; z++) {
   *             const size_t iz = 4*bz + z;
   *             const auto mask = bitmask(x, y, z);
   *             if (!(val & mask)) {
   *               Neighbor vnode {this->voxel_center(ix, iy, iz), ix, iy, iz, 1};
   *               auto nearest = voxel_nn.nearest(vnode);
   *               if (dist(vnode, nearest) <= r) {
   *                 val |= mask;
   *               }
   *             }
   *           }
   *         }
   *       }

   *       return val;
   *     });
   */

  /* Attempt #3 */
  // over-approximation with multiple dilations
  dilate_6neighbor(int(std::round(r / std::min(dx(), std::min(dy(), dz())))));
}

void VoxelOctree::erode_6neighbor() {
  throw std::runtime_error("erode_6neighbor() unimplemented");
}

void VoxelOctree::erode_27neighbor() {
  throw std::runtime_error("erode_27neighbor() unimplemented");
}

void VoxelOctree::erode_sphere(double r) {
  UNUSED_VAR(r);
  throw std::runtime_error("erode_sphere() unimplemented");
}

bool VoxelOctree::collides(const Point &p) const {
  if (!is_in_domain(p[0], p[1], p[2])) { return false; }
  auto [ix, iy, iz] = nearest_cell(p);
  return cell(ix, iy, iz);
}

bool VoxelOctree::collides(const VoxelOctree &other) const {
  check_dims(_N, other._N);
  return visit_both(_tree, other._tree,
                    [](const auto &a, const auto &b) { return a.collides(b); },
                    true);
}

void VoxelOctree::remove_point(double x, double y, double z) {
  if (!is_in_domain(x, y, z)) { return; }
  auto [ix, iy, iz] = nearest_cell(x, y, z);
  set_cell(ix, iy, iz, false);
}

void VoxelOctree::remove_voxels(const VoxelOctree &other) {
  check_dims(_N, other._N);
  visit_both(_tree, other._tree,
             [](auto &a, const auto &b) { return a.remove_tree(b); });
}

void VoxelOctree::intersect_voxels(const VoxelOctree &other) {
  check_dims(_N, other._N);
  visit_both(_tree, other._tree,
             [](auto &a, const auto &b) { return a.intersect_tree(b); });
}

void VoxelOctree::visit_leaves(const ConstBlockVisitor &visitor) const {
  std::visit([&visitor] (const auto &tree) { tree.visit_leaves(visitor); },
             _tree);
}

void VoxelOctree::visit_blocks(const ConstBlockVisitor &visitor) const {
  std::visit([&visitor] (const auto &tree) { tree.visit_blocks(visitor); },
             _tree);
}

void VoxelOctree::visit_modify_blocks(const ModBlockVisitor &visitor) {
  std::visit([&visitor] (auto &tree) { tree.visit_modify_blocks(visitor); },
             _tree);
}

void VoxelOctree::visit_occupied_voxels(
    const ConstOccupiedVoxelVisitor &visitor) const
{
  // visit each leaf block and call visitor for each occupied voxel
  std::visit([&visitor] (const auto &tree) {
        auto wrapped =
            [&visitor](size_t bx, size_t by, size_t bz, uint64_t block) {
              for (u8 x = 0; x < 4; ++x) {
                for (u8 y = 0; y < 4; ++y) {
                  for (u8 z = 0; z < 4; ++z) {
                    if (block & bitmask(x, y, z)) {
                      visitor(4*bx + x, 4*by + y, 4*bz + z);
                    }
                  }
                }
              }
            };
        tree.visit_leaves(wrapped);
      }, _tree);
}

void VoxelOctree::visit_voxels(const ConstVoxelVisitor &visitor) const {
  // visit each block and call visitor for each voxel
  std::visit([&visitor] (const auto &tree) {
        auto wrapped =
            [&visitor](size_t bx, size_t by, size_t bz, uint64_t block) {
              for (u8 x = 0; x < 4; ++x) {
                for (u8 y = 0; y < 4; ++y) {
                  for (u8 z = 0; z < 4; ++z) {
                    bool is_occupied = block & bitmask(x, y, z);
                    visitor(4*bx + x, 4*by + y, 4*bz + z, is_occupied);
                  }
                }
              }
            };
        tree.visit_blocks(wrapped);
      }, _tree);
}

void VoxelOctree::visit_modify_voxels(const ModVoxelVisitor &visitor)
{
  // visit each block and call the visitor for each voxel, returning the new
  // block value
  std::visit([&visitor] (auto &tree) {
        auto wrapped =
            [&visitor](size_t bx, size_t by, size_t bz, uint64_t block) {
              uint64_t new_block {0};
              for (u8 x = 0; x < 4; ++x) {
                for (u8 y = 0; y < 4; ++y) {
                  for (u8 z = 0; z < 4; ++z) {
                    auto mask = bitmask(x, y, z);
                    bool is_occupied = block & bitmask(x, y, z);
                    bool newval = visitor(4*bx + x, 4*by + y, 4*bz + z, is_occupied);
                    new_block |= (newval * mask);
                  }
                }
              }
              return new_block;
            };
        tree.visit_modify_blocks(wrapped);
      }, _tree);
}

Mesh VoxelOctree::to_mesh() const {

  if (is_empty()) {
    return {};
  }

  // each voxel occupies the space of
  //   (xlim().first + ix*dx(), ...)
  // to
  //   (xlim().first + (ix+1)*dx(), ...)
  // we want to convert those boxes to vertices and triangles.
  // Note: for two adjacent voxels, we do NOT want to create faces between them.
  // It is probably fine to have vertices at each voxel position instead of
  // merging them.  You can load the mesh into a tool like blender to merge
  // faces along the same plane if you wish to make it more condensed.
  //
  // Therefore, we will create a vertex if it is on the border between an
  // occupied voxel and a free voxel.  That makes this algorithm very similar
  // to remove_interior().

  // iterate over the tree and add to the list of triangles
  // before the conversion to the workspace.  Let's stay in voxel integer space
  // for now.
  using Coord = stl_io_impl::CoordWithIndex<size_t, size_t>;
  std::vector<Coord> coords;
  std::vector<size_t> triangles;

  // append the vertex coordinate and return its index
  auto append_coord = [&coords](size_t x, size_t y, size_t z) {
    Coord c; c[0] = x; c[1] = y; c[2] = z;
    c.index = coords.size();
    coords.emplace_back(std::move(c));
    return coords.size() - 1;
  };

  enum Direction {
    LEFT,
    RIGHT,
    FRONT,
    BEHIND,
    BELOW,
    ABOVE,
  };

  auto append_quad = [&append_coord, &triangles]
    (size_t x, size_t y, size_t z, Direction d)
    {
      size_t c0, c1, c2, c3;
      switch(d) {
        case LEFT:
          c0 = append_coord(x, y, z);
          c1 = append_coord(x, y+1, z+1);
          c2 = append_coord(x, y, z+1);
          c3 = append_coord(x, y+1, z);
          break;

        case RIGHT:
          c0 = append_coord(x+1, y, z+1);
          c1 = append_coord(x+1, y+1, z);
          c2 = append_coord(x+1, y, z);
          c3 = append_coord(x+1, y+1, z+1);
          break;

        case FRONT:
          c0 = append_coord(x, y, z);
          c1 = append_coord(x+1, y, z+1);
          c2 = append_coord(x+1, y, z);
          c3 = append_coord(x, y, z+1);
          break;

        case BEHIND:
          c0 = append_coord(x, y+1, z+1);
          c1 = append_coord(x+1, y+1, z);
          c2 = append_coord(x+1, y+1, z+1);
          c3 = append_coord(x, y+1, z);
          break;

        case BELOW:
          c0 = append_coord(x, y, z);
          c1 = append_coord(x+1, y+1, z);
          c2 = append_coord(x, y+1, z);
          c3 = append_coord(x+1, y, z);
          break;

        case ABOVE:
          c0 = append_coord(x, y+1, z+1);
          c1 = append_coord(x+1, y, z+1);
          c2 = append_coord(x, y, z+1);
          c3 = append_coord(x+1, y+1, z+1);
          break;

        default: throw std::runtime_error("unimplemented: " + std::to_string(d));
      }

      // first triangle
      triangles.emplace_back(c0);
      triangles.emplace_back(c2);
      triangles.emplace_back(c1);

      // second triangle
      triangles.emplace_back(c0);
      triangles.emplace_back(c1);
      triangles.emplace_back(c3);
    };

  this->visit_leaves(
      [this, &append_quad](size_t bx, size_t by, size_t bz, u64 b)
  {

    // let's first implement in an easy to write and easy to read way.
    // we can try to optimize later
    for (size_t i = 0; i < 4; i++) {
      auto ix = 4*bx + i;
      for (size_t j = 0; j < 4; j++) {
        auto iy = 4*by + j;
        for (size_t k = 0; k < 4; k++) {
          auto iz = 4*bz + k;

          // get the current and the 6 neighbors
          bool current = bool(b & bitmask(i, j, k));
          if (!current) { continue; }

          bool left   = (i  >  0)      ? bool(b & bitmask(i-1, j, k)) :
                        (ix == 0)      ? false : cell(ix-1, iy, iz);
          bool right  = (i  <  3)      ? bool(b & bitmask(i+1, j, k)) :
                        (ix == Nx()-1) ? false : cell(ix+1, iy, iz);
          bool front  = (j  >  0)      ? bool(b & bitmask(i, j-1, k)) :
                        (iy == 0)      ? false : cell(ix, iy-1, iz);
          bool behind = (j  <  3)      ? bool(b & bitmask(i, j+1, k)) :
                        (iy == Ny()-1) ? false : cell(ix, iy+1, iz);
          bool below  = (k  >  0)      ? bool(b & bitmask(i, j, k-1)) :
                        (iz == 0)      ? false : cell(ix, iy, iz-1);
          bool above  = (k  <  3)      ? bool(b & bitmask(i, j, k+1)) :
                        (iz == Nz()-1) ? false : cell(ix, iy, iz+1);

          // draw each face that has an empty neighbor
          if (!left)   { append_quad(ix, iy, iz, LEFT  ); }
          if (!right)  { append_quad(ix, iy, iz, RIGHT ); }
          if (!front)  { append_quad(ix, iy, iz, FRONT ); }
          if (!behind) { append_quad(ix, iy, iz, BEHIND); }
          if (!below)  { append_quad(ix, iy, iz, BELOW ); }
          if (!above)  { append_quad(ix, iy, iz, ABOVE ); }
        }
      }
    }
  });

  // after the visiting, we have the coords and triangles vectors populated
  // now to remove duplicate coordinates
  std::vector<size_t> points;
  stl_io_impl::remove_doubles(points, triangles, coords);

  // populate a mesh object
  Mesh mesh;
  mesh.triangles.reserve(triangles.size() / 3);
  mesh.vertices.reserve(points.size() / 3);

  for (size_t tri_idx = 0; tri_idx < triangles.size(); tri_idx += 3) {
    mesh.triangles.emplace_back(
        triangles[tri_idx],
        triangles[tri_idx + 1],
        triangles[tri_idx + 2]
        );
  }

  for (size_t pnt_idx = 0; pnt_idx < points.size(); pnt_idx += 3) {
    // convert from voxel index to world coordinates
    mesh.vertices.emplace_back(
        xlim().first + dx() * points[pnt_idx],
        ylim().first + dy() * points[pnt_idx + 1],
        zlim().first + dz() * points[pnt_idx + 2]
        );
  }

  return mesh;
}

VoxelOctree::ItkImagePtr VoxelOctree::to_itk_image() const {
  using IndexType     = typename ItkImageType::IndexType;
  using SizeType      = typename ItkImageType::SizeType;
  using RegionType    = typename ItkImageType::RegionType;
  using SpacingType   = typename ItkImageType::SpacingType;
  using PointType     = typename ItkImageType::PointType;
  using DirectionType = typename ItkImageType::DirectionType;
  using IteratorType  = itk::ImageRegionIteratorWithIndex<ItkImageType>;

  typename ItkImageType::Pointer image = ItkImageType::New();

  IndexType start;
  start[0] = 0; start[1] = 0; start[2] = 0;

  SizeType size;
  size[0] = _N; size[1] = _N; size[2] = _N;

  RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);

  SpacingType spacing;
  spacing[0] = _dx; spacing[1] = _dy; spacing[2] = _dz;
  image->SetSpacing(spacing);

  PointType origin;
  origin[0] = _xmin + _dx/2;
  origin[1] = _ymin + _dy/2;
  origin[2] = _zmin + _dz/2;
  image->SetOrigin(origin);

  DirectionType orientation;
  orientation.SetIdentity();
  image->SetDirection(orientation);

  image->Allocate();

  // populate image
  IteratorType it (image, image->GetLargestPossibleRegion());
  for (; !it.IsAtEnd(); ++it) {
    auto &idx = it.GetIndex();
    it.Set(cell(idx[0], idx[1], idx[2]) ? 1 : 0);
  }

  return image;
}

VoxelOctree VoxelOctree::from_itk_image(VoxelOctree::ItkImagePtr image) {
  auto region = image->GetLargestPossibleRegion();
  auto size = region.GetSize();
  auto spacing = image->GetSpacing();
  auto origin = image->GetOrigin();

  VoxelOctree v(
      VoxelOctree::to_supported_size(std::max({size[0], size[1], size[2]})));

  v.set_xlim(origin[0], origin[0] + v.Nx() * spacing[0]);
  v.set_ylim(origin[1], origin[1] + v.Ny() * spacing[1]);
  v.set_zlim(origin[2], origin[2] + v.Nz() * spacing[2]);
  // move by - 1/2 voxel place since this voxel octree is center at the
  // bottom-left corner rather than the center, like in ITK images
  v.set_xlim(v.xlim().first - v.dx()/2, v.xlim().second - v.dx()/2);
  v.set_ylim(v.ylim().first - v.dy()/2, v.ylim().second - v.dy()/2);
  v.set_zlim(v.zlim().first - v.dz()/2, v.zlim().second - v.dz()/2);

  // populate image
  using IteratorType = itk::ImageRegionConstIteratorWithIndex<ItkImageType>;
  IteratorType it (image, image->GetLargestPossibleRegion());
  for (; !it.IsAtEnd(); ++it) {
    auto &idx = it.GetIndex();
    if (it.Get()) {
      v.set_cell(idx[0], idx[1], idx[2]);
    }
  }

  return v;
}


void VoxelOctree::to_nrrd(const std::string &fname, bool compress) const {
  using WriterType = itk::ImageFileWriter<ItkImageType>;
  typename itk::NrrdImageIO::Pointer io = itk::NrrdImageIO::New();
  typename WriterType::Pointer writer = WriterType::New();
  writer->SetInput(this->to_itk_image());
  writer->SetImageIO(io);
  writer->SetFileName(fname);
  if (compress) {
    writer->UseCompressionOn();
#if ITK_VERSION_MAJOR >= 5
    writer->SetCompressionLevel(4);
#endif
  }
  writer->Update(); // do the actual writing
}

VoxelOctree VoxelOctree::from_nrrd(const std::string &fname) {
  using ReaderType = itk::ImageFileReader<ItkImageType>;
  typename ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(fname);
  reader->Update();  // read the image
  return VoxelOctree::from_itk_image(reader->GetOutput());
}

nlohmann::json VoxelOctree::to_json() const {
  // populate the data
  nlohmann::json blocks;
  this->visit_leaves([&blocks] (size_t bx, size_t by, size_t bz, u64 val){
    blocks.push_back({bx, by, bz, val});
  });

  nlohmann::json container;
  container = {
    {"VoxelOctree", {
      {"dimension", _N},
      {"xlimits", {_xmin, _xmax}},
      {"ylimits", {_ymin, _ymax}},
      {"zlimits", {_zmin, _zmax}},
      {"data", blocks},
    }},
  };

  return container;
}

VoxelOctree VoxelOctree::from_json(const nlohmann::json &obj) {
  auto vobj = obj["VoxelOctree"];
  VoxelOctree voxels(vobj["dimension"].get<size_t>());

  voxels.set_xlim(vobj["xlimits"][0], vobj["xlimits"][1]);
  voxels.set_ylim(vobj["ylimits"][0], vobj["ylimits"][1]);
  voxels.set_zlim(vobj["zlimits"][0], vobj["zlimits"][1]);

  // read the voxel data
  for (auto &block : vobj["data"]) {
    u64 val;
    block[3].get_to(val);
    voxels.set_block(block[0], block[1], block[2], val);
  }

  return voxels;
}

std::shared_ptr<cpptoml::table> VoxelOctree::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("VoxelOctree", tbl);
  tbl->insert("dimension", _N);

  auto xlim_array = cpptoml::make_array();
  tbl->insert("xlimits", xlim_array);
  xlim_array->push_back(_xmin);
  xlim_array->push_back(_xmax);

  auto ylim_array = cpptoml::make_array();
  tbl->insert("ylimits", ylim_array);
  ylim_array->push_back(_ymin);
  ylim_array->push_back(_ymax);

  auto zlim_array = cpptoml::make_array();
  tbl->insert("zlimits", zlim_array);
  zlim_array->push_back(_zmin);
  zlim_array->push_back(_zmax);

  // populate the data
  auto data_array = cpptoml::make_array();
  tbl->insert("data", data_array);
  this->visit_leaves([&data_array] (size_t bx, size_t by, size_t bz, u64 val){
    auto arr = cpptoml::make_array();
    data_array->push_back(arr);
    arr->push_back(bx);
    arr->push_back(by);
    arr->push_back(bz);
    arr->push_back(uint32_t(val >> 32));
    arr->push_back(uint32_t(val));
  });

  return container;
}

VoxelOctree VoxelOctree::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // check if the container is inside of tbl, otherwise assume we are already in
  if (tbl->contains("VoxelOctree")) {
    auto container = tbl->get("VoxelOctree")->as_table();
    if (!container) {
      throw cpptoml::parse_exception (
          "Wrong type detected for 'VoxelOctree': not a table");
    }
    tbl = container;
  }

  auto dimension = tbl->get("dimension")->as<int64_t>();
  auto xlimits   = tbl->get_array_of<double>("xlimits");
  auto ylimits   = tbl->get_array_of<double>("ylimits");
  auto zlimits   = tbl->get_array_of<double>("zlimits");
  auto data      = tbl->get_array_of<cpptoml::array>("data");
  if (!(dimension && xlimits && ylimits && zlimits && data)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }

  VoxelOctree voxels(dimension->get());
  voxels.set_xlim((*xlimits)[0], (*xlimits)[1]);
  voxels.set_ylim((*ylimits)[0], (*ylimits)[1]);
  voxels.set_zlim((*zlimits)[0], (*zlimits)[1]);

  // read the voxel data
  for (auto &datum : *data) {
    u64 upper_val = datum->at(3)->as<int64_t>()->get();
    u64 lower_val = datum->at(4)->as<int64_t>()->get();
    u64 val = (upper_val << 32) | lower_val;  // bit or them to combine
    voxels.set_block(
        datum->at(0)->as<int64_t>()->get(),
        datum->at(1)->as<int64_t>()->get(),
        datum->at(2)->as<int64_t>()->get(),
        val);
  }

  return voxels;
}

void VoxelOctree::to_file(const std::string &fname) {
  if (util::endswith(fname, ".nrrd")) {
    to_nrrd(fname);
  } else if (util::endswith(fname, ".toml") ||
             util::endswith(fname, ".toml.gz"))
  {
    cpptoml::to_file(this->to_toml(), fname);
  } else {
    util::write_json(fname, this->to_json());
  }
}

VoxelOctree VoxelOctree::from_file(const std::string &fname) {
  if (util::endswith(fname, ".nrrd")) {
    return VoxelOctree::from_nrrd(fname);
  } else if (util::endswith(fname, ".toml") ||
             util::endswith(fname, ".toml.gz"))
  {
    return cpptoml::from_file<VoxelOctree>(fname);
  } else {
    return VoxelOctree::from_json(util::read_json(fname));
  }
}

// one in the given place, zeros everywhere else
// for x, y, z within a block (each should be 0 <= x < 4)
u64 VoxelOctree::bitmask(u8 x, u8 y, u8 z) {
  return u64(1) << (x*16 + y*4 + z);
}

bool VoxelOctree::is_in_domain(double x, double y, double z) const {
  return (_xmin <= x && x <= _xmax)
      && (_ymin <= y && y <= _ymax)
      && (_zmin <= z && z <= _zmax);
}

void VoxelOctree::domain_check(double x, double y, double z) const {
  if (x < _xmin || _xmax < x) {
    throw std::domain_error("x is out of the voxel dimensions");
  }
  if (y < _ymin || _ymax < y) {
    throw std::domain_error("y is out of the voxel dimensions");
  }
  if (z < _zmin || _zmax < z) {
    throw std::domain_error("z is out of the voxel dimensions");
  }
}

void VoxelOctree::limit_check(const VoxelOctree &other) const {
  double eps = std::numeric_limits<double>::epsilon();
  auto dbl_eq_check = [eps](const std::string &name, double val1, double val2) {
    if (std::abs(val1 - val2) >= eps) {
      throw std::domain_error(name + " does not match");
    }
  };
  dbl_eq_check("xmin", this->_xmin, other._xmin);
  dbl_eq_check("ymin", this->_ymin, other._ymin);
  dbl_eq_check("zmin", this->_zmin, other._zmin);
  dbl_eq_check("xmax", this->_xmax, other._xmax);
  dbl_eq_check("ymax", this->_ymax, other._ymax);
  dbl_eq_check("zmax", this->_zmax, other._zmax);
}

} // end of namespace collision
