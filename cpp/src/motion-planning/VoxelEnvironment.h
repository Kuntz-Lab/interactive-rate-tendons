#ifndef VOXEL_ENVIRONMENT_H
#define VOXEL_ENVIRONMENT_H

#include <tendon/TendonRobot.h>
#include <collision/VoxelOctree.h>

#include <Eigen/Core>

#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stack>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace cpptoml {
class table;
}

namespace itk {
template <typename T> class SmartPointer;
template <typename P, unsigned int D> class Image;
}

namespace motion_planning {

struct VoxelEnvironment {
  using PixelType = unsigned char;
  using ImageType = itk::Image<PixelType, 3>;
  using ImagePtr  = itk::SmartPointer<itk::Image<PixelType, 3>>;
  using VoxelPtr  = std::shared_ptr<collision::VoxelOctree>;
  // args: (a, b, t, out): out = a + t*(b-a)
  using InterpFunc = std::function<void(const std::vector<double>&, const std::vector<double>&,
                                        double, std::vector<double>&)>;
  using FkFunc    = std::function<tendon::TendonResult(const std::vector<double>)>;
  using ValidFunc = std::function<bool(const std::vector<double>&, const tendon::TendonResult&)>;

  std::string filename;          // voxel file containing the 3D voxel obstacles
  double scaling = 1.0;          // how to scale the voxel file to be in meters
  Eigen::Vector3d translation =
    Eigen::Vector3d::Zero();     // how to translate the voxel file to the robot
                                 // frame
  Eigen::Matrix3d inv_rotation =
    Eigen::Matrix3d::Identity(); // how to rotate the robot frame to the voxel
                                 // image
  std::string interior_fname;    // voxel file containing the free space

  void set_obstacle_cache(VoxelPtr obstacles) {
    _obstacle_cache = obstacles;
  }

  /// loads the VoxelOctree from filename the first time, then caches it
  VoxelPtr get_obstacles() const {
    if (!_obstacle_cache) {
      _obstacle_cache = std::make_shared<collision::VoxelOctree>(
          collision::VoxelOctree::from_file(this->filename));
    }
    return _obstacle_cache;
  }

  VoxelPtr get_interior() const {
    if (!_interior_cache) {
      _interior_cache = std::make_shared<collision::VoxelOctree>(
          collision::VoxelOctree::from_file(this->interior_fname));
    }
    return _interior_cache;
  }

  bool operator==(const VoxelEnvironment &other) const {
    return filename     == other.filename
        && scaling      == other.scaling
        && translation  == other.translation
        && inv_rotation == other.inv_rotation;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static VoxelEnvironment from_toml(std::shared_ptr<cpptoml::table> table);

  /// Bring the image origin to the robot origin
  void translate_and_scale_image(ImagePtr image) const;

  /// Rotates the point in the robot frame to the image frame
  Eigen::Vector3d rotate_point(const Eigen::Vector3d &point) const;

  /// Rotate points in the robot frame to the image frame in-place
  void rotate_points(std::vector<Eigen::Vector3d> &points) const;

  /** Voxelize the robot backbone motion from start to end
   *
   * This version checks that the robot discretization is appropriate for the
   * voxel space.
   */
  collision::VoxelOctree voxelize_backbone_motion(
      const collision::VoxelOctree &reference,
      const tendon::TendonRobot &robot,
      const std::vector<double> &start,
      const std::vector<double> &end,
      double rel_threshold = 1e-5) const;

  /** Voxelize the motion of fk from start to end
   *
   * fk is a function (or anything that implements operator()()).  It takes a
   * configuration the same size as start (and stop, since they're both
   * required to be the same size) and returns a std::vector<collision::Point>
   * set of points to voxelize.
   *
   * TODO: do I want another function for voxelizing what fk returns?
   *
   * This does NOT check that the fk discretization is appropriate for the
   * voxel space.  That would require checking the spacing between each point
   * in EVERY call to fk(), which would be too costly.  Just ensure this is the
   * case before calling this version of the function.
   *
   * @param rel_threshold: relative threshold for considering a subdivision.  A
   *   value of 0.5 would mean no subdivision is considered because the first
   *   subdivision will be halfway between each endpoint.  It's a percentage
   *   distance between two divisions.  A subdivision is considered if its
   *   config distance is greater than or equal to
   *   rel_threshold * distance(start, end).
   *
   * This version is a convenience for voxelize_valid_backbone_motion() where
   * the valid state checker is set to always return true.
   */
  collision::VoxelOctree voxelize_backbone_motion(
      const collision::VoxelOctree &reference,
      const InterpFunc &interp,
      const FkFunc &fk,
      const std::vector<double> &start,
      const std::vector<double> &end,
      double rel_threshold = 1e-5) const;

  struct PartialVoxelization {
    bool is_fully_valid;
    double t;
    std::vector<double> last_valid;
    std::vector<collision::Point> last_backbone;
    collision::VoxelOctree voxels {4};
  };

  /** Like voxelize_backbone_motion(), but returns the portion before collision
   *
   * You must also provide a function to check for state validity.
   *
   * Returns four values
   * - is_fully_valid: true means the edge spans from start to end
   * - last_valid_t: t in [0, 1] specifying how far from start to end it went
   * - last_valid_config: the last valid configuration state from start to end
   *   if is_full, then this will be equal to end.
   * - voxels: the voxelized edge from start to last_valid.
   *
   * checker is a function (or anything that implements operator()()).  It
   * takes a single configuration (like start) and the value returned by fk,
   * returning true or false.
   */
  PartialVoxelization voxelize_valid_backbone_motion(
      const collision::VoxelOctree &reference,
      const InterpFunc &interp,
      const FkFunc &fk,
      const ValidFunc &checker,
      const std::vector<double> &start,
      const std::vector<double> &end,
      double rel_threshold = 1e-5) const;

private:
  mutable VoxelPtr _obstacle_cache;
  mutable VoxelPtr _interior_cache;
}; // end of VoxelEnvironment struct declaration

} // end of namespace motion_planning

#endif // VOXEL_ENVIRONMENT_H
