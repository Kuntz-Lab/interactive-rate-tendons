#ifndef MOTION_PLANNING_IO_ROADMAP_WRITER_H
#define MOTION_PLANNING_IO_ROADMAP_WRITER_H

#include <Eigen/Core>

#include <optional>
#include <memory>

namespace collision{
  class VoxelOctree;
}

namespace motion_planning {
namespace io{

class RoadmapWriter {
public:
  virtual ~RoadmapWriter() = default;
  
  virtual void write_vertex(
      uint32_t index,
      const std::vector<double> state,
      std::optional<Eigen::Vector3d> tip_pos,
      std::shared_ptr<collision::VoxelOctree> voxels) = 0;

  virtual void write_edge(
      uint32_t source,
      uint32_t target,
      double weight,
      std::shared_ptr<collision::VoxelOctree> voxels) = 0;
};

}} // end of namespace motion_planning::io


#endif // MOTION_PLANNING_IO_ROADMAP_WRITER_H
