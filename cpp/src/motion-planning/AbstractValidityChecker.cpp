#include "AbstractValidityChecker.h"
#include <util/FunctionTimer.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace motion_planning {

AbstractValidityChecker::AbstractValidityChecker(
    const ob::SpaceInformationPtr &si,
    const tendon::TendonRobot &robot)
  : ob::StateValidityChecker(si)
  , _robot(robot)
  , _home_shape(robot.home_shape())
  , _fk_func()
  , _tau_index(-1)
  , _retraction_index(-1)
  , _rotation_index(-1)
{
  _timers.emplace("fk", util::FunctionTimer());
  _timers.emplace("collision", util::FunctionTimer());
  _timers.emplace("self_collision", util::FunctionTimer());

  auto *state_space = si->getStateSpace()->as<ob::CompoundStateSpace>();
  if (state_space->hasSubspace(tension_state_space_name())) {
    _tau_index = state_space->getSubspaceIndex(tension_state_space_name());
  } else {
    throw std::invalid_argument("state space does not have "
        + tension_state_space_name());
  }

  if (state_space->hasSubspace(retraction_state_space_name())) {
    _retraction_index =
        state_space->getSubspaceIndex(retraction_state_space_name());
  }

  if (state_space->hasSubspace(rotation_state_space_name())) {
    _rotation_index =
        state_space->getSubspaceIndex(rotation_state_space_name());
  }
}

std::vector<double>
AbstractValidityChecker::extract_from_state(const ob::State *state) const {
  const auto &compound_state =
      *state->as<ob::CompoundStateSpace::StateType>();

  // get tensions from the state
  const auto &tau_state =
      compound_state[_tau_index]->as<ob::RealVectorStateSpace::StateType>();
  std::vector<double> robot_state(tau_state->values,
                                  tau_state->values + _robot.tendons.size());

  // get theta from the state if it is there
  if (_rotation_index >= 0) {
    const auto &rotation_state = compound_state[_rotation_index]
                                 ->as<ob::SO2StateSpace::StateType>();
    robot_state.emplace_back(rotation_state->value);
  }

  // get s_start from the state if it is there
  double s_start = 0.0;
  if (_retraction_index >= 0) {
    const auto &retraction_state = compound_state[_retraction_index]
                                  ->as<ob::RealVectorStateSpace::StateType>();
    s_start = retraction_state->values[0];
    robot_state.emplace_back(s_start);
  }

  return robot_state;
}

std::pair<tendon::TendonResult, tendon::TendonResult>
AbstractValidityChecker::fk(const std::vector<double> &robot_state) const {
  tendon::TendonResult fk_shape, home_shape;
  _timers["fk"].time(
      [this, &fk_shape, &home_shape, &robot_state]() {
        if (this->_fk_func) {
          fk_shape = this->_fk_func(robot_state);
        } else {
          fk_shape = this->_robot.shape(robot_state);
        }
        if (this->_robot.enable_retraction) {
          home_shape = this->_robot.home_shape(robot_state);
        } else {
          home_shape = this->_home_shape;
        }
      });
  return {std::move(fk_shape), std::move(home_shape)};
}

bool AbstractValidityChecker::is_valid_shape(
    const tendon::TendonResult &fk_shape,
    const tendon::TendonResult &home_shape) const
{
  if (!fk_shape.converged || !home_shape.converged) {
    return false; // only convergent FK results are valid
  }

  bool is_len_okay = _robot.is_within_length_limits(
      _robot.calc_dl(home_shape.L_i, fk_shape.L_i));
  if (!is_len_okay) { return false; } // avoid self-collision check

  bool self_collides = _timers["self_collision"].time(
      [this, &fk_shape]() { return _robot.collides_self(fk_shape); });
  return !self_collides;
}

bool AbstractValidityChecker::collides(
    const tendon::TendonResult &fk_shape) const
{
  return _timers["collision"].time([this, &fk_shape]() {
        return this->collides_impl(fk_shape);
      });
}

bool AbstractValidityChecker::isValid(const ompl::base::State *state) const {
  auto robot_state = this->extract_from_state(state);
  auto [fk_shape, home_shape] = this->fk(robot_state);

  if (!this->is_valid_shape(fk_shape, home_shape)) {
    return false;
  }

  return ! this->collides(fk_shape);
}

} // end of namespace motion_planning
