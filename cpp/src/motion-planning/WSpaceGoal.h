#ifndef W_SPACE_GOAL_H
#define W_SPACE_GOAL_H

#include "AbstractValidityChecker.h"
#include <tendon/TendonRobot.h>
#include <util/macros.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalRegion.h>

#include <Eigen/Core>

namespace motion_planning {

/** Goal type for workspace-specified desired tip position */
class WSpaceGoal : public ompl::base::GoalRegion {
public:
  WSpaceGoal(const ompl::base::SpaceInformationPtr &si,
             const Eigen::Vector3d desiredTip)
    : ompl::base::GoalRegion(si)
    , desiredTip_(desiredTip)
  { }

  double distanceGoal(const ompl::base::State *st) const override {
    auto checker = std::dynamic_pointer_cast<
        AbstractValidityChecker>(si_->getStateValidityChecker());
    if (!checker) {
      OMPL_ERROR("motion_planning::WSpaceGoal: "
          "state validity checker is not a "
          "motion_planning::AbstractValidityChecker");
    }
    auto robot_state = checker->extract_from_state(st);
    auto [fk_shape, home_shape] = checker->fk(robot_state);
    UNUSED_VAR(home_shape);
    auto &tip = fk_shape.p.back();
    return (tip - desiredTip_).norm();
  }

  void setDesiredTip(const Eigen::Vector3d &desiredTip) {
    desiredTip_ = desiredTip;
  }

  Eigen::Vector3d& getDesiredTip()             { return desiredTip_; }
  const Eigen::Vector3d& getDesiredTip() const { return desiredTip_; }

  void print(std::ostream &out = std::cout) const override {
    out <<
      "WSpaceGoal("
         "tip = [" << desiredTip_.transpose() << "], "
         "threshold = " << threshold_ << ", "
         "address = " << this << ")\n";
  }

private:
  Eigen::Vector3d desiredTip_;

}; // end of class WSpaceGoal

} // end of namespace motion_planning


#endif // W_SPACE_GOAL_H
