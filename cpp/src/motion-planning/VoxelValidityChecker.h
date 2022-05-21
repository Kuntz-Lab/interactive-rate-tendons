#ifndef VOXEL_VALIDITY_CHECKER_H
#define VOXEL_VALIDITY_CHECKER_H

#include "AbstractVoxelValidityChecker.h"
#include <collision/Sphere.h>
#include <collision/VoxelOctree.h>
#include <tendon/TendonResult.h>

namespace motion_planning {

/** Validity checker for voxel environments */
class VoxelValidityChecker : public AbstractVoxelValidityChecker {
public:
  // reuse base class constructor
  using AbstractVoxelValidityChecker::AbstractVoxelValidityChecker;

protected:
  virtual collision::VoxelOctree voxelize_impl(
      const tendon::TendonResult &fk_shape) const override
  {
    auto rotated_points = fk_shape.p;
    _venv.rotate_points(rotated_points);
    auto fk_voxels = _voxels.empty_copy();
    for (auto &pos : rotated_points) { fk_voxels.add_sphere({pos, _robot.r}); }
    return fk_voxels;
  }
};

} // end of namespace motion_planning

#endif // VOXEL_VALIDITY_CHECKER_H
