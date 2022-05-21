#ifndef ABSTRACT_VALIDITY_CHECKER_H
#define ABSTRACT_VALIDITY_CHECKER_H

#include "tendon/TendonRobot.h"
#include "tendon/TendonResult.h"
#include "util/FunctionTimer.h"
#include "util/macros.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace motion_planning {

class AbstractValidityChecker : public ompl::base::StateValidityChecker {
public:
  using FkFuncType = std::function<tendon::TendonResult(const std::vector<double>&)>;

  static std::string tension_state_space_name()
  { return "TensionStateSpace"; }

  static std::string retraction_state_space_name()
  { return "RetractionStateSpace"; }

  static std::string rotation_state_space_name()
  { return "RotationStateSpace"; }

  AbstractValidityChecker(const ompl::base::SpaceInformationPtr &si,
                          const tendon::TendonRobot &robot);

  virtual ~AbstractValidityChecker() {}

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

  const tendon::TendonRobot& robot() const { return _robot; }

  // getter/setter FK function.  If empty, then defaults to robot.shape()
  FkFuncType fk_func() const { return _fk_func; }
  void set_fk_func(FkFuncType fk_func) { _fk_func = std::move(fk_func); }

  /// helper function to convert OMPL state to robot state
  std::vector<double> extract_from_state(const ompl::base::State *state) const;

  /** Runs forward kinematics and captures the timing
   *
   * @return two things:
   *   - fk_shape: the robot shape at robot_state
   *   - home_shape: the robot shape at robot_state with tensions set to zero
   */
  std::pair<tendon::TendonResult, tendon::TendonResult>
  fk(const std::vector<double> &robot_state) const;

  /** Checks validity of the shape without considering collisions
   *
   * fk_shape and home_shape are the same values returned from fk().
   */
  virtual bool is_valid_shape(const tendon::TendonResult &fk_shape,
                              const tendon::TendonResult &home_shape) const;

  /// calls and times collides_impl()
  bool collides(const tendon::TendonResult &fk_shape) const;

  /** Don't override this.
   *
   * Instead override is_valid_shape() and collides_impl()
   *
   * Simply calls extract_from_state(), then fk(), then is_valid_shape(), and
   * finally collides().
   */
  virtual bool isValid(const ompl::base::State *state) const override;

protected:
  virtual bool collides_impl(const tendon::TendonResult &fk_shape) const = 0;

protected:
  const tendon::TendonRobot &_robot;      ///< robot specification
  const tendon::TendonResult _home_shape; ///< cached home shape (s_start = 0)
  FkFuncType _fk_func;
  int _tau_index;
  int _retraction_index;
  int _rotation_index;

  // timing and other capturing
  mutable std::unordered_map<std::string, util::FunctionTimer> _timers;
};


} // end of namespace motion_planning

#endif // ABSTRACT_VALIDITY_CHECKER_H
