#ifndef VOXEL_BACKBONE_VALIDITY_CHECKER_H
#define VOXEL_BACKBONE_VALIDITY_CHECKER_H

#include "AbstractVoxelValidityChecker.h"
#include "VoxelEnvironment.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>

#include <ompl/base/SpaceInformation.h>

#include <algorithm>  // for std::max
#include <sstream>
#include <stdexcept>  // for std::invalid_argument

namespace motion_planning {

/** Validity checker of robot backbone against a voxel environment
 *
 * This only checks collision against the backbone of the robot and ignores the
 * robot's radius.  It is assumed that the voxel octree has already been
 * dilated by the robot's radius.
 *
 * It is necessary and checked that the robot's length discretization is less
 * than or equal to the smallest voxel dimension
 *   (i.e., robot.specs.dL < max(voxel.dx(), voxel.dy(), voxel.dz()).
 */
class VoxelBackboneValidityChecker : public AbstractVoxelValidityChecker {
public:
  VoxelBackboneValidityChecker(const ompl::base::SpaceInformationPtr &si,
                               const tendon::TendonRobot &robot,
                               const VoxelEnvironment &venv,
                               collision::VoxelOctree voxels)
    : AbstractVoxelValidityChecker(si, robot, venv, voxels)
  {
    // check the assumption that the backbone is discretized small enough
    if (robot.specs.dL > std::max({voxels.dx(), voxels.dy(), voxels.dz()})) {
      auto max_dim = std::max({voxels.dx(), voxels.dy(), voxels.dz()});
      std::ostringstream msgbuilder;
      msgbuilder
          << "robot.specs.dL is larger than expected by "
             "VoxelBackboneValidityChecker ("
          << robot.specs.dL << " > " << max_dim << ")";
      throw std::invalid_argument(msgbuilder.str());
    }
  }

protected:
  virtual collision::VoxelOctree voxelize_impl(
      const tendon::TendonResult &fk_shape) const override
  {
    auto rotated_points = fk_shape.p;
    _venv.rotate_points(rotated_points);
    auto fk_voxels = _voxels.empty_copy();
    fk_voxels.add_piecewise_line(rotated_points);
    return fk_voxels;
  }
};

} // end of namespace motion_planning

#endif // VOXEL_BACKBONE_VALIDITY_CHECKER_H
