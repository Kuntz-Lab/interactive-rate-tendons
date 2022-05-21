#ifndef OCTOMAP_VALIDITY_CHECKER_H
#define OCTOMAP_VALIDITY_CHECKER_H

#include "AbstractValidityChecker.h"
#include "Environment.h"
#include "VoxelValidityChecker.h"
#include "collision/OctomapWrap.h"
#include "tendon/TendonRobot.h"
#include "tendon/TendonResult.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <string>

namespace motion_planning {

class OctomapValidityChecker : public AbstractValidityChecker {
public:
  OctomapValidityChecker(const ompl::base::SpaceInformationPtr &si,
                  const tendon::TendonRobot &robot,
                  const VoxelEnvironment &venv,
                  const collision::VoxelOctree &voxels)
    : AbstractValidityChecker(si, robot)
    , _vchk(si, robot, venv, voxels)
    , _octobs(voxels)
  {
    _timers.emplace("octowrap-wrap", util::FunctionTimer());
    _timers.emplace("octowrap-collides", util::FunctionTimer());
  }

protected:
  virtual bool collides_impl(
      const tendon::TendonResult &fk_shape) const override
  {
    auto voxels = _vchk.voxelize(fk_shape);
    auto octvox = _timers["octowrap-wrap"].time([&voxels]() {
          return collision::OctomapWrap(voxels);
        });
    return _timers["octowrap-collides"].time([this, &octvox]() {
          return this->_octobs.collides(octvox);
        });
  }

protected:
  VoxelValidityChecker _vchk; ///< voxelize with this
  collision::OctomapWrap _octobs; ///< converted env voxels
};


} // end of namespace motion_planning

#endif // OCTOMAP_VALIDITY_CHECKER_H
