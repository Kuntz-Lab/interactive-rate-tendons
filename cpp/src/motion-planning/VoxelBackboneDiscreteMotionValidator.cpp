#include "VoxelBackboneDiscreteMotionValidator.h"

#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

namespace motion_planning {

VoxelBackboneDiscreteMotionValidator::PartialVoxelization
VoxelBackboneDiscreteMotionValidator::generic_voxelize(
      const RobotState &a,
      const RobotState &b,
      const FkFunc &fk_func,
      const IsValidFunc &is_valid_func) const
{
  auto fk_wrap = _timers["swept-volume-fk-call"].wrap(fk_func);
  auto is_valid_wrap =
      _timers["swept-volume-state-collision-check"].wrap(is_valid_func);

  PartialVoxelization v;
  v.t = 0;
  v.last_valid = a;
  v.voxels = _voxels.empty_copy();
  auto shape = fk_wrap(a);
  v.last_backbone = shape.p;
  v.is_fully_valid = is_valid_wrap(a, shape);
  _venv.rotate_points(v.last_backbone);
  v.voxels.add_piecewise_line(v.last_backbone);

  auto space = si_->getStateSpace();
  ob::ScopedState<> sa(space);
  ob::ScopedState<> sb(space);
  sa = a;
  sb = b;

  // implementation mostly copied from
  // ompl::base::DiscreteMotionValidator::checkMotion() (with lastValid)

  auto nd = space->validSegmentCount(sa.get(), sb.get());

  // check points between a and b
  if (nd > 1) {
    ob::ScopedState<> stest(space);
    auto test = a;
    for (decltype(nd) i = 1; v.is_fully_valid && i < nd; ++i) {
      double t = double(i) / double(nd);
      space->interpolate(sa.get(), sb.get(), t, stest.get());
      space->copyToReals(test, stest.get());
      auto shape = fk_wrap(test);
      v.is_fully_valid = is_valid_wrap(test, shape);
      if (v.is_fully_valid) {
        v.t = t;
        v.last_valid = test;
        v.last_backbone = shape.p;
        _venv.rotate_points(v.last_backbone);
        v.voxels.add_piecewise_line(v.last_backbone);
      }
    }
  }

  // check b
  if (v.is_fully_valid) {
    shape = fk_wrap(b);
    v.is_fully_valid = is_valid_wrap(b, shape);
    if (v.is_fully_valid) {
      v.t = 1;
      v.last_valid = b;
      v.last_backbone = shape.p;
      _venv.rotate_points(v.last_backbone);
      v.voxels.add_piecewise_line(v.last_backbone);
    }
  }

  return v;
  // TODO: finish voxelizing with nd discrete steps
  // TODO: add this class and VoxelBackboneMotionValidator to python interface
  // TODO: add voxelize edge to voxel_ops.py
  // TODO: make figure with edge made from this class
}

} // end of namespace motion_planning
