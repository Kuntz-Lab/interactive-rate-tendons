#ifndef VOXEL_BACKBONE_MOTION_VALIDATOR_AND_LOGGER_H
#define VOXEL_BACKBONE_MOTION_VALIDATOR_AND_LOGGER_H

#include "VoxelBackboneMotionValidator.h"
#include "VoxelEnvironment.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonRobot.h>

#include <ompl/base/SpaceInformation.h>

#include <string>
#include <vector>
#include <mutex>
#include <limits>

namespace motion_planning {

/** Same as VoxelBackboneMotionValidator but logs minimum discretizations.
 *
 * This is only used in experimenting because it is possible that it could
 * incur significant overhead.
 */
class VoxelBackboneMotionValidatorAndLogger : public VoxelBackboneMotionValidator {
public:
  using BaseClass           = VoxelBackboneMotionValidator;
  using RobotState          = BaseClass::RobotState;
  using ShapeType           = BaseClass::ShapeType;
  using PartialVoxelization = BaseClass::PartialVoxelization;

public:
  VoxelBackboneMotionValidatorAndLogger(
      const ompl::base::SpaceInformationPtr &si,
      const tendon::TendonRobot &robot,
      const VoxelEnvironment &venv,
      collision::VoxelOctree voxels,
      const std::string &logname
    )
    : VoxelBackboneMotionValidator(si, robot, venv, voxels)
    , logname_(logname)
  { }

  virtual ~VoxelBackboneMotionValidatorAndLogger() override;

protected:
  std::string logname_;

  // cached values that are able to be modified from "const" methods
  mutable std::mutex smallest_mutex_; //< to still allow multithreading
  mutable double smallest_dtau_ = std::numeric_limits<double>::infinity();
  mutable double smallest_drot_ = std::numeric_limits<double>::infinity();
  mutable double smallest_dret_ = std::numeric_limits<double>::infinity();
  mutable size_t max_segments_ = 0;
  mutable RobotState max_seg_a_;
  mutable RobotState max_seg_b_;

protected:
  using CollisionFunc = BaseClass::CollisionFunc;
  using FkFunc        = BaseClass::FkFunc;
  using IsValidFunc   = BaseClass::IsValidFunc;

protected:
  virtual PartialVoxelization generic_voxelize(
      const RobotState &a,
      const RobotState &b,
      const FkFunc &fk_func,
      const IsValidFunc &is_valid_func)
    const override;
}; // end of class VoxelBackboneMotionValidatorAndLogger

} // end of namespace motion_planning

#endif // VOXEL_BACKBONE_MOTION_VALIDATOR_AND_LOGGER_H
