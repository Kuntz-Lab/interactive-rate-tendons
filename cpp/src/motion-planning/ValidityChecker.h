#ifndef VALIDITY_CHECKER_H
#define VALIDITY_CHECKER_H

#include "AbstractValidityChecker.h"
#include "Environment.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonResult.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <string>

namespace motion_planning {

class ValidityChecker : public AbstractValidityChecker {
public:
  ValidityChecker(const ompl::base::SpaceInformationPtr &si,
                  const tendon::TendonRobot &robot,
                  const Environment &env)
    : AbstractValidityChecker(si, robot)
    , _env(env)
  { }

protected:
  virtual bool collides_impl(
      const tendon::TendonResult &fk_shape) const override
  {
    collision::CapsuleSequence robot_shape {fk_shape.p, _robot.r};
    return _env.collides(robot_shape);
  }

protected:
  const Environment &_env;  ///< obstacle environment
};


} // end of namespace motion_planning

#endif // VALIDITY_CHECKER_H
