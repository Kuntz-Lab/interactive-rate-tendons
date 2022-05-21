#ifndef STRAIGHT_LINE_PLANNER_H
#define STRAIGHT_LINE_PLANNER_H

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/PathGeometric.h>

#include <string>

namespace motion_planning {

/** Simply tries to plan in a straight line in configuration space
 *
 * Currently only supports one start and one goal state
 */
class StraightLinePlanner : public ompl::base::Planner {
public:

  StraightLinePlanner(ompl::base::SpaceInformationPtr si)
    : ompl::base::Planner(si, "StraightLinePlanner")
  { }

  ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition &ptc) override;

  void clear() override {
    _solved = false;
    _straight_line_is_valid = false;
  }

private:
  bool _solved = false; // true means solve() was already called
  bool _straight_line_is_valid = false;
};

} // end of namespace motion_planning

#endif // STRAIGHT_LINE_PLANNER_H
