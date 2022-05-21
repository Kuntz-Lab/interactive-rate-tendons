#ifndef MOTION_PLANNING_IO_ROADMAP_PARSER_H
#define MOTION_PLANNING_IO_ROADMAP_PARSER_H

namespace motion_planning {
namespace io{

struct ParsedVertex {
  uint32_t index;
  std::vector<double> state;
  std::optional<Eigen::Vector3d> tip_pos;
  std::shared_ptr<collision::VoxelOctree> voxels;
};

struct ParsedEdge {
  uint32_t source;
  uint32_t target;
  double weight;
  std::shared_ptr<collision::VoxelOctree> voxels;
};

enum class ParsedType {
  NONE   = 0,
  VERTEX = 1,
  EDGE   = 2,
  DONE   = 3,
};

/** abstract class for file parsing
 *
 * Example:
 *   ifstream in("roadmap.dat");
 *   LazyDatParser parser(in);
 *   while (parser) {
 *     switch (parser.next()) {
 *       case ParsedType::VERTEX:
 *         parser.populate_voxels();
 *         auto v = parser.current_vertex();
 *         // use it
 *         break;
 *       case ParsedType::EDGE:
 *         parser.populate_voxels();
 *         auto e = parser.current_edge();
 *         // use it
 *         break;
 *       case ParsedType::DONE:
 *         break;
 *       default:
 *         throw std::runtime_error("unsupported ParsedType");
 *     }
 *   }
 */
class RoadmapParser {
public:

  virtual ~RoadmapParser() = default;

  operator bool() { return _current_type != ParsedType::DONE; }

  ParsedType current_type() const { return _current_type; }
  ParsedVertex current_vertex() const { return _current_vertex; }
  ParsedEdge current_edge() const { return _current_edge; }

  virtual ParsedType next() = 0;

  /** Populate the current parsed object's voxels (if handles_voxels_lazily())
   *
   * This is not required of parsers.  The default is to do nothing which means
   * the voxels are populated by next().
   *
   * Parsers may be lazy with generating the voxels object until requested.
   * Before trying to use the voxels object of current_vertex() or
   * current_edge(), call parse_voxels().  This will only update the current
   * parsed object, and will disappear if you go to the next parsed object.
   */
  virtual void populate_voxels() { }
  virtual bool handles_voxels_lazily() const { return false; }

protected:
  ParsedType   _current_type = ParsedType::NONE;
  ParsedVertex _current_vertex;
  ParsedEdge   _current_edge;
}; // end of class RoadmapParser

}} // end of namespace motion_planning::io


#endif // MOTION_PLANNING_IO_ROADMAP_PARSER_H
