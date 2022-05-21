#ifndef VOXEL_BACKBONE_DISCRETE_MOTION_VALIDATOR_H
#define VOXEL_BACKBONE_DISCRETE_MOTION_VALIDATOR_H

#include "AbstractValidityChecker.h"
#include "VoxelBackboneMotionValidator.h"
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
 * Uses the discrete motion validator to voxelize the motion.
 */
class VoxelBackboneDiscreteMotionValidator : public VoxelBackboneMotionValidator {
public:
  using RobotState = std::vector<double>;
  using ShapeType = tendon::TendonResult;

public:
  // use base-class constructor
  using VoxelBackboneMotionValidator::VoxelBackboneMotionValidator;

protected:
  using CollisionFunc = std::function<bool(const ShapeType&)>;
  using FkFunc        = std::function<ShapeType(const RobotState&)>;
  using IsValidFunc   = std::function<bool(const RobotState&, const ShapeType&)>;

  virtual PartialVoxelization generic_voxelize(
      const RobotState &a,
      const RobotState &b,
      const FkFunc &fk_func,
      const IsValidFunc &is_valid_func) const override;
}; // end of class VoxelBackboneDiscreteMotionValidator

} // end of namespace motion_planning

#endif // VOXEL_BACKBONE_DISCRETE_MOTION_VALIDATOR_H
