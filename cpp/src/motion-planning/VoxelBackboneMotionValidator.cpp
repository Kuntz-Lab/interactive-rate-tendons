#include "motion-planning/VoxelBackboneMotionValidator.h"
#include <util/macros.h>

namespace motion_planning {

using VBMV = VoxelBackboneMotionValidator;

VBMV::VoxelBackboneMotionValidator(
    const ompl::base::SpaceInformationPtr &si,
    const tendon::TendonRobot &robot,
    const VoxelEnvironment &venv,
    collision::VoxelOctree voxels)
  : AbstractVoxelMotionValidator(si, robot, venv, voxels)
{
  _timers.emplace("swept-volume-fk-call", util::FunctionTimer());
  _timers.emplace("swept-volume-state-collision-check", util::FunctionTimer());
}

VBMV::PartialVoxelization VBMV::generic_voxelize(
    const RobotState &a,
    const RobotState &b,
    const CollisionFunc &is_collision_func) const
{
  auto fk_func = [this] (const RobotState &state) {
    auto [fk_shape, home_shape] = this->_vc->fk(state);
    UNUSED_VAR(home_shape);
    return fk_shape;
  }; // end of lambda fk()

  auto is_valid_func = [this, &is_collision_func]
    (const RobotState &state, const ShapeType &shape)
  {
    auto home_shape = this->_robot.home_shape(state);
    return this->_vc->is_valid_shape(shape, home_shape)
        && !is_collision_func(shape);
  }; // end of lambda is_valid_func()

  return this->generic_voxelize(a, b, fk_func, is_valid_func);
}

VBMV::PartialVoxelization VBMV::generic_voxelize(
    const RobotState &a,
    const RobotState &b,
    const FkFunc &fk_func,
    const IsValidFunc &is_valid_func) const
{
  auto space = si_->getStateSpace();
  ob::ScopedState<> s_a(space), s_b(space), s_c(space);
  s_a = a;
  s_b = b;

  // set rel_threshold such that taken steps are half of the
  // DiscreteMotionValidator's step size at the very smallest, but allow for
  // bigger steps depending on the voxelization
  auto nseg = space->validSegmentCount(s_a.get(), s_b.get());
  double rel_threshold = 1.0 / double(nseg);

  auto interp = [&s_a, &s_b, &s_c, &space]
    (const std::vector<double> &a, const std::vector<double> &b, double t,
     std::vector<double> &out)
  {
    s_a = a;
    s_b = b;
    space->interpolate(s_a.get(), s_b.get(), t, s_c.get());
    space->copyToReals(out, s_c.get());
  };

  auto fk_wrap = _timers["swept-volume-fk-call"].wrap(fk_func);
  auto is_valid_wrap =
      _timers["swept-volume-state-collision-check"].wrap(is_valid_func);

  return _venv.voxelize_valid_backbone_motion(
      _voxels, interp, fk_wrap, is_valid_wrap, a, b, rel_threshold);
}

VBMV::PartialVoxelization VBMV::voxelize_impl(
    const RobotState &a, const RobotState &b) const
{
  auto do_nothing = [] (const ShapeType&) { return false; };
  return this->generic_voxelize(a, b, do_nothing);
}

VBMV::PartialVoxelization VBMV::voxelize_until_invalid_impl(
    const RobotState &a, const RobotState &b) const
{
  auto is_collision_func = [this] (const ShapeType &shape) {
    return this->_vc->collides(shape);
  };

  return this->generic_voxelize(a, b, is_collision_func);
}

} // end of namespace motion_planning
