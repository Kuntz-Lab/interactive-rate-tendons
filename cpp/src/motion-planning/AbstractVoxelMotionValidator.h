#ifndef ABSTRACT_VOXEL_MOTION_VALIDATOR_H
#define ABSTRACT_VOXEL_MOTION_VALIDATOR_H

#include "AbstractValidityChecker.h"
#include "VoxelEnvironment.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonRobot.h>
#include <util/FunctionTimer.h>
#include <util/macros.h>

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace motion_planning {

/** Validity checker of robot motion against a voxel environment
 *
 * This abstract class allows for different strategies of voxelization by
 * subclassing and implementing voxelize_impl().
 *
 * It is only supported to use this motion validator with a planner where the
 * state validity checker has already been set to an instance of the
 * AbstractValidityChecker (currently it does not require
 * AbstractVoxelValidityChecker).
 */
class AbstractVoxelMotionValidator : public ompl::base::DiscreteMotionValidator {
public:
  using PartialVoxelization = VoxelEnvironment::PartialVoxelization;

public:
  AbstractVoxelMotionValidator(const ompl::base::SpaceInformationPtr &si,
                               const tendon::TendonRobot &robot,
                               const VoxelEnvironment &venv,
                               collision::VoxelOctree voxels)
    : ompl::base::DiscreteMotionValidator(si)
    , _timers(4)
    , _num_voxelize_errors(0)
    , _robot(robot)
    , _venv(venv)
    , _voxels(std::move(voxels))
    , _vc(std::dynamic_pointer_cast<AbstractValidityChecker>(
            si_->getStateValidityChecker()))
  {
    _timers.emplace("voxelize-swept-volume", util::FunctionTimer());
    _timers.emplace("voxelize-valid-swept-volume", util::FunctionTimer());
    _timers.emplace("collision-swept-volume", util::FunctionTimer());
    if (_vc == nullptr) {
      throw std::invalid_argument("Planner does not use AbstractValidityChecker");
    }
  }

  virtual ~AbstractVoxelMotionValidator() {}

  /** Timing stuff.
   *
   * Derived classes can add more timers by just adding to _timers.
   */
  void clear_timing() const {
    for (auto &[name, timer] : _timers) {
      UNUSED_VAR(name);
      timer.clear();
    }
  }
  const std::unordered_map<std::string, util::FunctionTimer>& timers() const
  { return _timers; }
  const util::FunctionTimer& timer(const std::string &name) const
  { return _timers.at(name); }
  const auto& timing(const std::string &name) const
  { return this->timer(name).get_times(); }
  size_t calls(const std::string &name) const
  { return this->timing(name).size(); }

  size_t num_voxelize_errors() const { return _num_voxelize_errors; }

  /// return the cached validity checker
  std::shared_ptr<AbstractValidityChecker> validity_checker() const
  { return _vc; }

  /// helper function to convert OMPL state to robot state
  std::vector<double> extract_from_state(const ompl::base::State *state) const
  { return _vc->extract_from_state(state); }

  /** Voxelize the swept volume between the two states (with timing)
   *
   * If an invalid configuration is found in the motion during voxelization
   * (i.e., length or tension limits, or self-collision), then the returned
   * voxelization is only partial.
   *
   * Only self-collision is checked when voxelizing, not full collision.  If
   * that is what you want, call voxelize_until_invalid().
   */
  virtual PartialVoxelization voxelize(const std::vector<double> &a,
                                       const std::vector<double> &b) const
  {
    auto partial = _timers["voxelize-swept-volume"].time(
        [this, &a, &b]() {
          return this->voxelize_impl(a, b);
        });
    if (!partial.is_fully_valid) { _num_voxelize_errors++; }
    return partial;
  }

  /** Voxelize from a to b, stopping at the first invalid state
   *
   * Returns four values
   * - is_fully_valid: true means the edge spans from start to end
   * - last_valid_t: t in [0, 1] specifying how far from start to end it went
   * - last_valid_config: the last valid configuration state from start to end
   *   if is_full, then this will be equal to end.
   * - voxels: the voxelized edge from start to last_valid.
   */
  virtual 
  PartialVoxelization
  voxelize_until_invalid(const std::vector<double> &a,
                         const std::vector<double> &b) const
  {
    return _timers["voxelize-valid-swept-volume"].time(
        [this, &a, &b]() {
          return this->voxelize_until_invalid_impl(a, b);
        });
  }

  /** Times and checks collision of the swept volume against voxel obstacles */
  bool collides(const collision::VoxelOctree &swept_volume) const {
    return _timers["collision-swept-volume"].time(
        [this, &swept_volume]() {
          return this->_voxels.collides(swept_volume);
        });
  }

  /** Check if the full path is free of collision and valid
   *
   * Voxelizes the swept volume and checks that.
   *
   * Derived classes should typically not need to reimplement this.
   */
  virtual bool checkMotion(const ompl::base::State *s1,
                           const ompl::base::State *s2) const override
  {
    auto start = this->extract_from_state(s1);
    auto end   = this->extract_from_state(s2);
    auto partial = this->voxelize(start, end);

    return partial.is_fully_valid && !this->collides(partial.voxels);
  }

  virtual bool checkMotion(const ompl::base::State *s1,
                           const ompl::base::State *s2,
                           std::pair<ompl::base::State*, double> &last_valid)
                          const override
  {
    auto start = this->extract_from_state(s1);
    auto end   = this->extract_from_state(s2);
    auto partial = this->voxelize_until_invalid(start, end);

    auto space = si_->getStateSpace();
    last_valid.second = partial.t;
    if (last_valid.first != nullptr) {
      space->interpolate(s1, s2, last_valid.second, last_valid.first);
    }

    return partial.is_fully_valid;
  }

protected:
  /** Voxelize and check shape and control validity along motion
   *
   * If there is an invalid configuration discovered in the motion while
   * voxelizing, this method should set the partial voxelization's
   * is_fully_valid to false.
   */
  virtual PartialVoxelization voxelize_impl(
      const std::vector<double> &a, const std::vector<double> &b) const = 0;

  virtual PartialVoxelization voxelize_until_invalid_impl(
      const std::vector<double> &a, const std::vector<double> &b) const = 0;

protected:
  mutable std::unordered_map<std::string, util::FunctionTimer> _timers;
  /// count of voxelize() calls that aren't fully valid
  mutable size_t _num_voxelize_errors;

  const tendon::TendonRobot &_robot;  ///< robot specification
  const VoxelEnvironment    &_venv;   ///< convert robot -> voxel space
  collision::VoxelOctree     _voxels; ///< voxel obstacles
  std::shared_ptr<AbstractValidityChecker> _vc; ///< validity checker
}; // end of class AbstractVoxelMotionValidator

} // end of namespace motion_planning

#endif // ABSTRACT_VOXEL_MOTION_VALIDATOR_H
