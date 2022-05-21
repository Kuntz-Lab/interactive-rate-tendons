#ifndef VOXEL_BACKBONE_MOTION_VALIDATOR_H
#define VOXEL_BACKBONE_MOTION_VALIDATOR_H

#include "AbstractValidityChecker.h"
#include "AbstractVoxelMotionValidator.h"
#include "VoxelEnvironment.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonRobot.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>

#include <algorithm>
#include <memory>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace ob = ompl::base;

namespace motion_planning {

/** Validity checker of robot backbone motion against a voxel environment
 *
 * This only checks collision against the backbone of the robot and ignores the
 * robot's radius.  It is assumed that the voxel octree has already been
 * dilated by the robot's radius.  This generates the swept volume and compares
 * against the voxel environment for collision.
 *
 * It is necessary and checked that the robot's length discretization is less
 * than or equal to the smallest voxel dimension
 *   (i.e., robot.specs.dL < max(voxel.dx(), voxel.dy(), voxel.dz()).
 */
class VoxelBackboneMotionValidator : public AbstractVoxelMotionValidator {
public:
  using RobotState = std::vector<double>;
  using ShapeType = tendon::TendonResult;

public:
  VoxelBackboneMotionValidator(const ompl::base::SpaceInformationPtr &si,
                               const tendon::TendonRobot &robot,
                               const VoxelEnvironment &venv,
                               collision::VoxelOctree voxels);

protected:
  using CollisionFunc = std::function<bool(const ShapeType&)>;
  using FkFunc        = std::function<ShapeType(const RobotState&)>;
  using IsValidFunc   = std::function<bool(const RobotState&, const ShapeType&)>;

  virtual PartialVoxelization generic_voxelize(
      const RobotState &a,
      const RobotState &b,
      const CollisionFunc &is_collision_func) const;

  virtual PartialVoxelization generic_voxelize(
      const RobotState &a,
      const RobotState &b,
      const FkFunc &fk_func,
      const IsValidFunc &is_valid_func) const;

  virtual PartialVoxelization voxelize_impl(
      const RobotState &a, const RobotState &b) const override;

  virtual PartialVoxelization voxelize_until_invalid_impl(
      const RobotState &a, const RobotState &b) const override;
}; // end of class VoxelBackboneMotionValidator

} // end of namespace motion_planning

#endif // VOXEL_BACKBONE_MOTION_VALIDATOR_H
