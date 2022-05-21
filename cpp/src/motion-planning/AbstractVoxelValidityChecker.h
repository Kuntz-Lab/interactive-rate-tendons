#ifndef ABSTRACT_VOXEL_VALIDITY_CHECKER_H
#define ABSTRACT_VOXEL_VALIDITY_CHECKER_H

#include "AbstractValidityChecker.h"
#include "VoxelEnvironment.h"
#include <collision/Sphere.h>
#include <collision/VoxelOctree.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonResult.h>
#include <util/FunctionTimer.h>

#include <ompl/base/SpaceInformation.h>

namespace motion_planning {

/** Validity checker for voxel environments */
class AbstractVoxelValidityChecker : public AbstractValidityChecker {
public:
  AbstractVoxelValidityChecker(const ompl::base::SpaceInformationPtr &si,
                               const tendon::TendonRobot &robot,
                               const VoxelEnvironment &venv,
                               collision::VoxelOctree voxels)
    : AbstractValidityChecker(si, robot)
    , _venv(venv)
    , _voxels(std::move(voxels))
  {
    _timers.emplace("voxelize", util::FunctionTimer());
    _timers.emplace("collision-without-voxelizing", util::FunctionTimer());
  }

  virtual ~AbstractVoxelValidityChecker() {}

  collision::VoxelOctree voxelize(const tendon::TendonResult &fk_shape) const {
    // capture voxelizing timing here
    return _timers["voxelize"].time([this, &fk_shape] () {
          return this->voxelize_impl(fk_shape);
        });
  }

  using AbstractValidityChecker::collides;

  /** an overload of collides() where the voxelized robot is given
   *
   * times separately from regular collision calls since it doesn't have to
   * voxelize.  This function will be called from collides_impl() as well to
   * get better timing of collision checking separately from voxelization.
   */
  bool collides(const collision::VoxelOctree &voxelized) const {
    return _timers["collision-without-voxelizing"].time([this, &voxelized]() {
          return this->_voxels.collides(voxelized);
        });
  }

protected:
  virtual bool collides_impl(const tendon::TendonResult &fk_shape) const override {
    return this->collides(this->voxelize(fk_shape));
  }

  virtual collision::VoxelOctree voxelize_impl(
      const tendon::TendonResult &fk_shape) const = 0;

protected:
  const VoxelEnvironment &_venv;   ///< convert robot -> voxel space
  collision::VoxelOctree  _voxels; ///< voxel obstacles
};

} // end of namespace motion_planning

#endif // ABSTRACT_VOXEL_VALIDITY_CHECKER_H
