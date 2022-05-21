#include "motion-planning/StraightLinePlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning {

ob::PlannerStatus StraightLinePlanner::solve(
    const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();

  if (_solved) { return {true, _straight_line_is_valid}; }

  // get the start and goal states
  auto start = pis_.nextStart();
  auto goal = pis_.nextGoal();
  if (start == nullptr || pis_.haveMoreStartStates()) {
    return ob::PlannerStatus::INVALID_START;
  } else if (goal == nullptr || pis_.haveMoreGoalStates()) {
    return ob::PlannerStatus::INVALID_GOAL;
  }

  if (ptc) { return ob::PlannerStatus::TIMEOUT; }

  // check the straight line between start and goal
  _straight_line_is_valid = si_->checkMotion(start, goal);
  _solved = true;

  // return solution, either exact or approximate
  if (_straight_line_is_valid) {
    auto path = std::make_shared<og::PathGeometric>(si_, start, goal);
    pdef_->addSolutionPath(path, false, 0.0, name_);
    return ob::PlannerStatus::EXACT_SOLUTION;
  } else {
    auto path = std::make_shared<og::PathGeometric>(si_, start);
    pdef_->addSolutionPath(path, true, si_->distance(start, goal), name_);
    return ob::PlannerStatus::APPROXIMATE_SOLUTION;
  }
}

} // end of namespace motion_planning
