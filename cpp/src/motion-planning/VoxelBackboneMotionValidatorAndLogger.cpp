#include "VoxelBackboneMotionValidatorAndLogger.h"
#include "VoxelEnvironment.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonRobot.h>
#include <util/openfile_check.h>
#include <util/vector_ops.h>

#include <ompl/base/SpaceInformation.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>

namespace motion_planning {

// alias shorthands
namespace ob              = ompl::base;
using VBMVL               = VoxelBackboneMotionValidatorAndLogger;
using BaseClass           = VBMVL::BaseClass;
using RobotState          = VBMVL::RobotState;
using ShapeType           = VBMVL::ShapeType;

using Locker              = std::lock_guard<std::mutex>;

using util::operator<<;

VBMVL::~VoxelBackboneMotionValidatorAndLogger() {
  double dtau, drot, dret;
  {
    Locker lock(smallest_mutex_);
    dtau = smallest_dtau_;
    drot = smallest_drot_;
    dret = smallest_dret_;
  }

  // log the minimum distances seen during my lifetime
  try {
    std::ofstream logstream;
    util::openfile_check(logstream, logname_);
    logstream << std::setprecision(std::numeric_limits<double>::digits10 + 1)
      << "dtau,drot,dret\n"
      << dtau << ","
      << drot << ","
      << dret << std::endl;
  } catch (...) {
    std::cerr << "Error in ~VoxelBackboneMotionValidatorAndLogger()" << std::endl;
  }
  OMPL_DEBUG("VoxelBackboneMotionValidatorAndLogger: dtau=%lf, drot=%lf, dret=%lf",
             dtau, drot, dret);

  std::cout <<
    "VoxelBackboneMotionValidatorAndLogger:\n"
    "  max segments: " << max_segments_ << "\n"
    "  a:            " << max_seg_a_    << "\n"
    "  b:            " << max_seg_b_    << "\n"
    "\n";
}

VBMVL::PartialVoxelization VBMVL::generic_voxelize(
    const RobotState &a, const RobotState &b,
    const FkFunc &fk_func,
    const IsValidFunc &is_valid_func)
  const
{
  const auto N = _robot.tendons.size();
  auto space = si_->getStateSpace();
  auto compound_space = space->as<ob::CompoundStateSpace>();
  auto bcopy = b;

  // calc s from the dimension of largest difference between a and b
  std::vector<double> svals;
  size_t sdim;
  double ai, di;

  // prep svals, sdim, ai, and di for the next voxelization run
  auto prep_svals = [&svals, &sdim, &ai, &di](const auto &a, const auto &b) {
    double maxdiff = std::abs(a[0] - b[0]);
    sdim = 0;
    for (size_t i = 0; i < a.size(); i++) {
      double diff = std::abs(a[i] - b[i]);
      if (diff > maxdiff) {
        sdim = i;
        maxdiff = diff;
      }
    }
    ai = a[sdim];
    di = maxdiff;
    svals.clear();
  };

  // wrap fk_func() to add to svals
  auto fkwrap = [this, &svals, &sdim, &ai, &di, &fk_func]
    (const RobotState &state)
    {
      svals.emplace_back(std::abs(state[sdim] - ai) / di);
      return fk_func(state);
    };

  // convenience lambda to calculate min ds
  auto calc_ds = [&svals]() {
    std::sort(svals.begin(), svals.end());
    double min_ds = svals[1] - svals[0];
    for (size_t i = 2; i < svals.size(); i++) {
      double ds = svals[i] - svals[i-1];
      min_ds = std::min(min_ds, ds);
    }
    return min_ds;
  };

  auto distance = [space](const auto &a, const auto &b) {
    ob::ScopedState<> sa(space);
    ob::ScopedState<> sb(space);
    sa = a;
    sb = b;
    return space->distance(sa.get(), sb.get());
  };

  std::copy(a.begin()+N, a.end(), bcopy.begin()+N); // copy rot and/or ret
  auto tau_space = compound_space->getSubspace(0);
  if (distance(a, bcopy) > 2 * tau_space->getLongestValidSegmentLength()) {
    // set ds_tau
    prep_svals(a, bcopy);
    BaseClass::generic_voxelize(a, bcopy, fkwrap, is_valid_func);
    auto ds_tau = calc_ds();
    Locker lock(smallest_mutex_);
    double square_dist = 0.0;
    for (size_t i = N; i-->0; ) {
      double diff = a[i] - b[i];
      square_dist += diff * diff;
    }
    smallest_dtau_ = std::min(smallest_dtau_,
                              ds_tau * std::sqrt(square_dist));
  }

  // set ds_rot
  if (_robot.enable_rotation) {
    bcopy = a;
    bcopy[N] = b[N]; // only rotation from b
    auto rot_space = compound_space->getSubspace(1);
    if (distance(a, bcopy) > 2 * rot_space->getLongestValidSegmentLength()) {
      prep_svals(a, bcopy);
      BaseClass::generic_voxelize(a, bcopy, fkwrap, is_valid_func);
      auto ds_rot = calc_ds();

      // update this->smallest_drot_
      Locker lock(smallest_mutex_);
      smallest_drot_ = std::min(smallest_drot_,
                                ds_rot * std::abs(a[N] - b[N]));
    }
  }

  // set ds_ret
  if (_robot.enable_retraction) {
    bcopy = a;
    bcopy.back() = b.back(); // only retraction from b
    auto ret_space = compound_space->getSubspace(
                         compound_space->getSubspaceCount()-1);
    if (distance(a, bcopy) > 2 * ret_space->getLongestValidSegmentLength()) {
      prep_svals(a, bcopy);
      BaseClass::generic_voxelize(a, bcopy, fkwrap, is_valid_func);
      auto ds_ret = calc_ds();

      // update this->smallest_dret_
      Locker lock(smallest_mutex_);
      smallest_dret_ = std::min(smallest_dret_,
                                ds_ret * std::abs(a.back() - b.back()));
    }
  }

  // call once regularly
  prep_svals(a, b);
  auto partial = BaseClass::generic_voxelize(a, b, fkwrap, is_valid_func);
  {
    Locker lock(smallest_mutex_);
    if (svals.size() > max_segments_) {
      max_segments_ = svals.size();
      max_seg_a_ = a;
      max_seg_b_ = b;
    }
  }

  return partial;
}

} // end of namespace motion_planning
