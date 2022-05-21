/*********************************************************************
* Copied from OMPL 1.5.0 source code from LazyPRM.cpp on 01 Dec 2020
*
*   src/ompl/geometric/planners/prm/LazyPRM.h
*
* Git repo:
*
*   https://github.com/ompl/ompl
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Henning Kayser */

#ifndef VOXEL_CACHED_LAZY_PRM_H
#define VOXEL_CACHED_LAZY_PRM_H

#include <cpptoml/cpptoml.h>
#include <tendon/TendonRobot.h>
#include <util/FunctionTimer.h>
#include <util/json_io.h>
#include <util/macros.h>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>

#include <Eigen/Core>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

// forward declarations
namespace motion_planning {
class AbstractVoxelMotionValidator;
class AbstractVoxelValidityChecker;
}
namespace collision { class VoxelOctree; }
namespace ompl { namespace base { OMPL_CLASS_FORWARD(OptimizationObjective); } }

namespace motion_planning {

namespace io {
class RoadmapParser; // forward declaration of private class
class RoadmapWriter; // forward declaration of private class
} // end of namespace motion_planning::io

/**
   @anchor gLazyPRM
   @par Short description
   LazyPRM is a planner that constructs a roadmap of milestones
   that approximate the connectivity of the state space, just like PRM does.
   The difference is that the planner uses lazy collision checking.
   @par External documentation
   R. Bohlin and L.E. Kavraki
   Path Planning Using Lazy PRM
   <em>IEEE International Conference on Robotics and Automation</em>, San Francisco, pp. 521–528, 2000.
   DOI: [10.1109/ROBOT.2000.844107](http://dx.doi.org/10.1109/ROBOT.2000.844107)<br>
   [[more]](http://www.kavrakilab.org/robotics/lazyprm.html)
*/

/** \brief Lazy Probabilistic RoadMap planner */
class VoxelCachedLazyPRM : public ompl::base::Planner {
public:
  using IKControllerFunc =
    std::function<
      std::vector<double>(
        const std::vector<double>&,
        const Eigen::Vector3d&)>;

  struct vertex_state_t {
    using kind = boost::vertex_property_tag;
  };

  struct vertex_voxel_cache_t {
    using kind = boost::vertex_property_tag;
  };

  struct vertex_tip_position_t {
    using kind = boost::vertex_property_tag;
  };

  struct vertex_flags_t {
    using kind = boost::vertex_property_tag;
  };

  struct vertex_component_t {
    using kind = boost::vertex_property_tag;
  };

  struct edge_voxel_cache_t {
    using kind = boost::edge_property_tag;
  };

  struct edge_flags_t {
    using kind = boost::edge_property_tag;
  };

  /** @brief The type for a vertex in the roadmap. */
  using Vertex =
    boost::adjacency_list_traits<boost::vecS, boost::listS,
                                 boost::undirectedS>::vertex_descriptor;

  using VoxelPtr    = std::shared_ptr<collision::VoxelOctree>;
  using TipPosition = std::optional<Eigen::Vector3d>;
  using size_type   = uint32_t;

  /**
   @brief The underlying roadmap graph.

   @par Any BGL graph representation could be used here. Because we
   expect the roadmap to be sparse (m<n^2), an adjacency_list is more
   appropriate than an adjacency_matrix. We use listS for the vertex list
   because vertex descriptors are invalidated by remove operations if using vecS.

   @par Obviously, a ompl::base::State* vertex property is required.
   The incremental connected components algorithm requires
   vertex_predecessor_t and vertex_rank_t properties.
   If boost::vecS is not used for vertex storage, then there must also
   be a boost:vertex_index_t property manually added.

   @par Edges should be undirected and have a weight property.

   @par Voxel cache for the vertex and edge will be set to nullptr if they are
   not yet calculated.  Otherwise, they will be set to a non-nullptr value and
   will be cleaned up in freeMemory() and in the destructor of this class.
   */
  using Graph = boost::adjacency_list<
    boost::vecS,         // out edges data structure (one per vertex)
    boost::listS,        // vertex data structure
    boost::undirectedS,  // the graph is undirected
    boost::property<vertex_state_t, ompl::base::State *,
      boost::property<vertex_voxel_cache_t, VoxelPtr,
      boost::property<vertex_tip_position_t, TipPosition,
      boost::property<boost::vertex_index_t, size_type,
      boost::property<vertex_flags_t, unsigned int,
      boost::property<vertex_component_t, size_type,
      boost::property<boost::vertex_predecessor_t, Vertex,
      boost::property<boost::vertex_rank_t, size_type>>>>>>>>,
    boost::property<boost::edge_weight_t, ompl::base::Cost,
      boost::property<edge_voxel_cache_t, VoxelPtr,
      boost::property<edge_flags_t, unsigned int>>>>;

  /** @brief The type for an edge in the roadmap. */
  using Edge = boost::graph_traits<Graph>::edge_descriptor;

  /** @brief A nearest neighbors data structure for roadmap vertices. */
  using RoadmapNeighbors = std::shared_ptr<ompl::NearestNeighbors<Vertex> >;

  /** @brief element type for TipNeighbors
   *
   * Those added to the nearest neighbor data structure have both set.  But,
   * since the distance function only uses the vector, query points only need
   * vec to be set.
   */
  struct VertexAndTip {
    Vertex vertex = Vertex{};
    Eigen::Vector3d vec = Eigen::Vector3d{};

    bool operator == (const VertexAndTip &other) const {
      return  vertex == other.vertex ||  // same vertex means same elemtn
             (vertex == Vertex{} &&    // default vertex and matching tip means equal
              vec    == other.vec);
    }

    bool operator != (const VertexAndTip &other) const {
      return !(*this == other);
    }
  };

  /** @brief A nearest neighbors data structure for tip position. */
  using TipNeighbors = std::shared_ptr<ompl::NearestNeighbors<VertexAndTip>>;

  /** @brief A function returning the milestones that should be
   * attempted to connect to. */
  using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;

  /** @brief A function that can reject connections.

   This is called after previous connections from the neighbor list
   have been added to the roadmap.
   */
  using ConnectionFilter = std::function<bool (const Vertex &, const Vertex &)>;

public:
  /** \brief Constructor */
  VoxelCachedLazyPRM(const ompl::base::SpaceInformationPtr &si,
                     bool starStrategy = false);

  /** \brief Constructor */
  VoxelCachedLazyPRM(const ompl::base::PlannerData &data,
                     bool starStrategy = false);

  ~VoxelCachedLazyPRM() override;

  const std::unordered_map<std::string, util::FunctionTimer>& timers() const
  { return timers_; }

  void clear_timing() {
    for (auto &[name, timer] : timers_) {
      UNUSED_VAR(name);
      timer.clear();
    }
  }

  const tendon::TendonRobot& robot();

  /** \brief Set the maximum length of a motion to be added to the roadmap. */
  void setRange(double distance);

  /** \brief Get the range the planner is using */
  double getRange() const { return maxDistance_; }

  /** \brief Set a different nearest neighbors datastructure */
  template <template <typename T> class NN>
  void setNearestNeighbors() {
    if (nn_ && nn_->size() == 0)
      OMPL_WARN("Calling setNearestNeighbors will clear all states.");

    clear();

    nn_ = std::make_shared<NN<Vertex>>();
    nnTip_ = std::make_shared<NN<VertexAndTip>>();
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
        return distanceFunction(a, b); });
    nnTip_->setDistanceFunction(
        [this](const VertexAndTip &a, const VertexAndTip &b) {
          return tipDistanceFunction(a, b);
        });

    if (!userSetConnectionStrategy_)
      setDefaultConnectionStrategy();

    if (isSetup())
      setup();
  }

  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef) override;

  /** \brief Set the connection strategy function that specifies the
   milestones that connection attempts will be make to for a
   given milestone.

   \par The behavior and performance of PRM can be changed drastically
   by varying the number and properties if the milestones that are
   connected to each other.

   \param pdef A function that takes a milestone as an argument and
   returns a collection of other milestones to which a connection
   attempt must be made. The default connection strategy is to connect
   a milestone's 10 closest neighbors.
   */
  void setConnectionStrategy(const ConnectionStrategy &connectionStrategy) {
    connectionStrategy_ = connectionStrategy;
    userSetConnectionStrategy_ = true;
  }

  /** Set the connection strategy to PRM star, even if not set in the
   * constructor
   *
   * Sets the state as having the star strategy.
   */
  void setStarConnectionStrategy();

  /** Set default strategy for connecting to nearest neighbors */
  void setDefaultConnectionStrategy();

  /** \brief Convenience function that sets the connection strategy to the
   default one with k nearest neighbors.
   */
  void setMaxNearestNeighbors(unsigned int k);

  /** \brief Set the function that can reject a milestone connection.

   \par The given function is called immediately before a connection
   is checked for collision and added to the roadmap. Other neighbors
   may have already been connected before this function is called.
   This allows certain heuristics that use the structure of the
   roadmap (like connected components or useful cycles) to be
   implemented by changing this function.

   \param connectionFilter A function that takes the new milestone,
   a neighboring milestone and returns whether a connection should be
   attempted.
   */
  void setConnectionFilter(const ConnectionFilter &connectionFilter) {
    connectionFilter_ = connectionFilter;
  }

  /** \brief set the IK controller function */
  void setIkController(IKControllerFunc &&func) {
    ikController_ = std::move(func);
  }

  /** Sets the default IK controller
   *
   * The version that does not specify a threshold can throw an ompl::Exception
   * if the state validity checker is not set using the setup() function
   *
   * @param max_iter: maximum number of iterations allowed to the underlying
   *     controller.
   * @param threshold: acceptable stopping threshold of tip position error.
   *     Default is robot.specs.dL / 5, where robot is obtained from the state
   *     validity checker.
   * @param mu_init: initial DLS damping factor.  May influence greatly the
   *     ability to converge.  Too small results in large step sizes, stopping
   *     at CSpace boundary points and quitting.  Too large results in small
   *     step sizes, potentially making slow progress in convergence.
   */
  void setDefaultIkController(int max_iter = 30) {
    const double default_threshold = this->robot().specs.dL / 5.0;
    setDefaultIkController(max_iter, default_threshold);
  }
  void setDefaultIkController(int max_iter, double threshold, double mu_init=1e3);

  /** \brief get the IK controller function */
  IKControllerFunc getIkController() const { return ikController_; }

  struct IKResult {
    std::vector<double> controls;  //< valid state either at or close to the
                                   //< desired goal position.
    Eigen::Vector3d tip_position;  //< tip position obtained by controls
    std::vector<double> neighbor;  //< where IK started from
    double error;                  //< achieved tip-position error
  };

  enum RoadmapIkOpts {
    RMAP_IK_SIMPLE   = 0x0,  // no extra features
    RMAP_IK_AUTO_ADD = 0x1,  // add IK result(s) to roadmap, and only return
                             // what could be connected to the roadmap.
    RMAP_IK_ACCURATE = 0x2,  // after IK config, try to connect from many
                             // nearest neighbors in the map rather than just
                             // the neighbor used for IK.
                             // If not RMAP_IK_AUTO_ADD, then this is only used
                             // if all IK solutions were either invalid or not
                             // within threshold.
    RMAP_IK_LAZY_ADD = 0x4,  // after finding one or more IK solutions that are
                             // good, we connect them to the roadmap.  This
                             // option makes those connections lazy, at least
                             // the ones that weren't already evaluated.
                             // Improves average runtime, but degrades the
                             // worst-case time.
                             // Only applicable if RMAP_IK_AUTO_ADD
  };

  /** Performs inverse kinematics (IK) to the requested tip position
   *
   * Python-like Pseudo-code:
   *
   *   neighbors = nearest_neighbors(request, k)
   *   iks = []
   *   for neighbor in neighbors:
   *       state, error = IK(neighbor.state, request)
   *       valid = is_valid(state)
   *       if error < tolerance and valid:
   *           if auto_add:
   *               add_to_roadmap(state)
   *           return state
   *       iks.append((neighbor, state, error, valid))
   *
   *   # none of the IK is both within tolerance and valid
   *   # fix invalid ones with closest valid
   *   for i, ik in enumerate(iks):
   *       if not ik.valid:
   *           (ik.state, ik.error) = closest_valid(ik.neighbor.state, ik.state)
   *           iks[i] = ik # update in the list
   *
   *   # return the one with the smallest error after correcting
   *   iks.sort(lambda x: x.error)
   *   closest = iks[0]
   *   if auto_add:
   *       add_to_roadmap(closest.state)
   *   return closest.state
   *
   * where closest_valid() has pseudo-code:
   *
   *   def closest_valid(a, b):
   *       diff = b - a
   *       d = delta * diff / diff.norm()
   *       while is_valid(a): # find the first invalid state
   *           a += d
   *       return (a - d, tip_error(a - d)) # return the last valid state
   *
   * Naturally, there are other details not captured here like computing voxel
   * caches, graph manipulation, etc.
   *
   * This method performs IK starting from up to k nearest neighbors in tip position
   * space from the roadmap.  Use the first one that is within tolerance, or
   * the closest one if none achieve tolerance.
   *
   * If the roadmap cannot produce any neighbors (e.g., if it's empty), then an
   * ompl::Exception is thrown.
   *
   * NOTE: this method is NOT reentrant
   *
   * @param request: desired tip position to be converted into desired
   *     configuration(s)
   * @param tolerance: (default = 1e-4) error threshold for IK stopping
   * @param k: (default = 5) number of nearest neighbors from the roadmap (in
   *     tip position space) to seed the IK to get valid goal configuration.
   *     This method will return at most this many configurations.
   * @param auto_add: (default = true) automatically add the found IK points to
   *     the roadmap along with any already computed voxelizations.  Also,
   *     precompute the voxelizations of the added edges in parallel.
   */
  std::optional<IKResult> roadmapIk(const Eigen::Vector3d &request,
                                    double tolerance = 1e-4,
                                    size_type k = 5,
                                    RoadmapIkOpts opt = RMAP_IK_SIMPLE);

  /** \brief Return the number of milestones currently in the graph */
  size_type milestoneCount() const { return boost::num_vertices(g_); }

  /** \brief Return the number of edges currently in the graph */
  size_type edgeCount() const { return boost::num_edges(g_); }

  void getPlannerData(ompl::base::PlannerData &data) const override;

  void setup() override;

  void clear() override;

  /** \brief Clear the query previously loaded from the ProblemDefinition.
   * Subsequent calls to solve() will reuse the previously computed roadmap,
   * but will clear the set of input states constructed by the previous call to
   * solve().  This enables multi-query functionality for VoxelCachedLazyPRM.
   */
  void clearQuery() override;

  // TODO: rename these to ALL_CAPS
  enum CreateRoadmapOption {
    LazyRoadmap      = 0x0,  // nothing is checked, just sampled configs and
                             // connecting edges
    VoxelizeVertices = 0x1,  // voxelize vertices, checking for valid shape,
                             // like self-collision
    ValidateVertices = 0x2,  // voxelize and validate vertices (validate
                             // without voxelize doesn't make sense)
    VoxelizeEdges    = 0x4,  // voxelize edges, checking for valid shape, like
                             // self-collision
    ValidateEdges    = 0x8,  // voxelize and validate edges (validate without
                             // voxelize doesn't make sense)
  };

  /** \brief create a roadmap to use with future queries of size N
   *
   * There are a number of options you can give to this.  Note that these
   * options only pertain to the added vertices and edges.  If the roadmap
   * already contains vertices and edges, then the given option will only apply
   * to the vertices and edges that are added to the roadmap as we try to bring
   * up the number of vertices up to N.
   *
   * The options are not mutually exclusive.  You can or them together to
   * combine multiple, like
   *   CreateRoadmapOption::VoxelizeVertices | CreateRoadmapOption::VoxelizeEdges
   *
   * The default is no options, which is equivalent to the Lazy option.
   */
  void createRoadmap(size_type N, CreateRoadmapOption opt = LazyRoadmap);

  /// Like createRoadmap(), but add N vertices instead of making it up to N
  // TODO: implement
  //void addToRoadmap(size_type N, CreateRoadmapOption opt = LazyRoadmap);

  /** \brief collision check all vertices and edges, removing those in collision */
  void precomputeVertexValidity();
  void precomputeEdgeValidity();
  void precomputeValidity();

  /** \brief change the validity flag of each node and edge to VALIDITY_UNKNOWN */
  void clearValidity();

  /** \brief remove vertices not connected to the largest connected component */
  void clearDisconnectedVertices();

  /** \brief compute and store missing voxelization of each vertex and edge */
  void precomputeVertexVoxelCache();
  void precomputeEdgeVoxelCache();
  void precomputeVoxelCache();

  /** \brief clear voxel cache of each vertex and edge */
  void clearVertexVoxelCache();
  void clearEdgeVoxelCache();
  void clearVoxelCache();

  ompl::base::PlannerStatus solve(
      const ompl::base::PlannerTerminationCondition &ptc) override;

  /** \brief solve with the existing roadmap - no new nodes besides start + goal
   * The given termination condition is in addition to finding an exact
   * solution (with an OR operator).
   */
  ompl::base::PlannerStatus solveWithRoadmap(
      const ompl::base::PlannerTerminationCondition &ptc
          = ompl::base::plannerNonTerminatingCondition());

  /** \brief same as above, but with a timeout in seconds */
  ompl::base::PlannerStatus solveWithRoadmap(double timeout);

  /** \brief save graph and cache to a json object */
  nlohmann::json toRoadmapJson() const;

  /** \brief save graph and cache to a json file */
  void saveRoadmapJson(const std::string &filename) const;

  /** \brief load graph and cache from a json object */
  void fromRoadmapJson(const nlohmann::json &data,
                       bool check_vertex_validity = true,
                       bool check_edge_validity = true);

  /** \brief load graph and cache from a json file */
  void loadRoadmapJson(const std::string &filename,
                       bool check_vertex_validity = true,
                       bool check_edge_validity = true);

  /** \brief save graph and cache to a toml object */
  std::shared_ptr<cpptoml::table> toRoadmapToml() const;

  /** \brief save graph and cache to a toml file */
  void saveRoadmapToml(const std::string &filename) const;

  /** \brief load graph and cache from a toml object */
  void fromRoadmapToml(std::shared_ptr<cpptoml::table> tbl,
                       bool check_vertex_validity = true,
                       bool check_edge_validity = true);

  /** \brief load graph and cache from a toml file */
  void loadRoadmapToml(const std::string &filename,
                       bool check_vertex_validity = true,
                       bool check_edge_validity = true);

  /** \brief save graph and cache to a custom type that can be streamed */
  void saveRoadmapDat(std::ostream &out) const;

  /** \brief load graph and cache from a custom type that can be streamed */
  void loadRoadmapDat(std::istream &in,
                      bool check_vertex_validity = true,
                      bool check_edge_validity = true);

  /** \brief save graph and cache to a file - based on file extension */
  void saveRoadmapToFile(const std::string &filename) const;

  /** \brief load graph and cache from a file - based on file extension */
  void loadRoadmapFromFile(const std::string &filename,
                           bool check_vertex_validity = true,
                           bool check_edge_validity = true);

  /** \brief go through each vertex and try connecting to neighbors */
  void addMissingEdges();

protected:
  void fromRoadmapParser(motion_planning::io::RoadmapParser &parser,
                         bool check_vertex_validity,
                         bool check_edge_validity);

  void toRoadmapWriter(motion_planning::io::RoadmapWriter &writer) const;

  bool computeVertexValidity(Vertex v);
  bool computeEdgeValidity(Edge e);

  /** \brief Flag indicating validity of an edge of a vertex */
  static const unsigned int VALIDITY_UNKNOWN = 0;

  /** \brief Flag indicating validity of an edge of a vertex */
  static const unsigned int VALIDITY_TRUE = 1;

  ///////////////////////////////////////
  // Planner progress property functions
  std::string getIterationCount() const { return std::to_string(iterations_); }
  std::string getBestCost() const { return std::to_string(bestCost_.value()); }
  std::string getMilestoneCountString() const
  { return std::to_string(milestoneCount()); }
  std::string getEdgeCountString() const { return std::to_string(edgeCount()); }

  /** \brief Free all the memory allocated by the planner */
  void freeMemory();

  /** \brief Construct a milestone for a given state (\e state), store it in
   * the nearest neighbors data structure and then connect it to the roadmap in
   * accordance to the connection strategy.
   *
   * If ok is given, then it will be set to true if state was successfully
   * added, and false otherwise (meaning it was already in the graph and state
   * is now deleted).
   */
  Vertex addMilestone(ompl::base::State *state, bool connect = true,
                      bool *ok = nullptr);

  void uniteComponents(Vertex a, Vertex b);

  void markComponent(Vertex v, size_type newComponent);

  /** \brief Check if any pair of a start state and goal state are part of the
   * same connected component.  If so, return the id of that component.
   * Otherwise, return -1.
   */
  long int solutionComponent(
      std::pair<size_type, size_type> *startGoalPair) const;

  /** \brief Given two milestones from the same connected component, construct
   * a path connecting them and set it as the solution
   */
  ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

  /** \brief Compute distance between two milestones (this is simply distance
   * between the states of the milestones)
   */
  double distanceFunction(const Vertex a, const Vertex b) const {
    return si_->distance(stateProperty_[a], stateProperty_[b]);
  }

  /** \brief Compute distance between two tip positions. */
  double tipDistanceFunction(const VertexAndTip &a, const VertexAndTip &b) const {
    return (b.vec - a.vec).norm();
  }

  /** \brief Given two vertices, returns a heuristic on the cost of the path
   * connecting them.  This method wraps
   * OptimizationObjective::motionCostHeuristic
   */
  ompl::base::Cost costHeuristic(Vertex u, Vertex v) const;

  /** \brief Add to graph only if not already there.  Return Vertex + if new.
   * If a new vertex, only sets the state property.
   */
  std::pair<Vertex, bool> tryAddToGraph(ompl::base::State *state);

  /** \brief Voxelize vertex if not already voxelized.  Return true if valid shape */
  bool voxelizeVertex(const Vertex v);

  std::optional<Edge> getEdge(const Vertex a, const Vertex b);

  std::optional<Edge> connectVertices(const Vertex a, const Vertex b);
  std::optional<Edge> connectVertices(const Vertex a, const Vertex b,
                                      ompl::base::Cost weight);

  /** \brief Voxelize edge if not already voxelized.  Return true if valid shape */
  bool voxelizeEdge(const Edge e);

  /** \brief Remove a set of vertices from the graph, returning neighbors */
  std::set<Vertex> removeVertices(const std::set<Vertex> &vertices);

  /** \brief Remove a single edge from the graph */
  void removeEdge(const Edge e);

  /** \brief renumber vertex indices from 0..N-1 */
  void renumberIndices();

  using PredecessorType = boost::property_map<Graph, boost::vertex_predecessor_t>::type;
  PredecessorType astarSearch(const Vertex start, const Vertex goal);

  /** \brief common prep for solve() and solveWithRoadmap()
   *
   * The goal_out is an out parameter of a dynamic pointer cast of the goal type.
   * If the return value is ompl::base::APPROXIMATE_SOLUTION, then the solve
   * should continue.  Any other value should be returned immediately.
   */
  ompl::base::PlannerStatus solvePrep(
      const ompl::base::PlannerTerminationCondition &ptc);

protected:

  /** \brief Flag indicating whether the default connection strategy is the
   * Star strategy
   */
  bool starStrategy_;

  /** \brief Function that returns the milestones to attempt connections with */
  ConnectionStrategy connectionStrategy_;

  /** \brief Function that can reject a milestone connection */
  ConnectionFilter connectionFilter_;

  /** \brief Flag indicating whether the employed connection strategy was set
   * by the user (or defaults are assumed)
   */
  bool userSetConnectionStrategy_{false};

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_{0.};

  /** \brief Sampler user for generating random in the state space */
  ompl::base::StateSamplerPtr sampler_;

  /** \brief Nearest neighbors data structure */
  RoadmapNeighbors nn_;

  /** \brief Nearest neighbors data structure for tip position */
  TipNeighbors nnTip_;

  /** \brief State checker from si_ */
  std::shared_ptr<AbstractVoxelValidityChecker> voxelStateChecker_;

  /** \brief Motion validator from si_ */
  std::shared_ptr<AbstractVoxelMotionValidator> voxelMotionValidator_;

  /** \brief Connectivity graph */
  Graph g_;

  /** \brief Array of start milestones */
  std::vector<Vertex> startM_;

  /** \brief Array of goal milestones */
  std::vector<Vertex> goalM_;

  /** \brief Access to the index for each Vertex */
  boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;

  /** \brief Access to the internal ompl::base::state at each Vertex */
  boost::property_map<Graph, vertex_state_t>::type stateProperty_;

  /** \brief Access to the internal voxel object at each Vertex (or nullptr) */
  boost::property_map<Graph, vertex_voxel_cache_t>::type vertexVoxelsProperty_;

  /** \brief Access to the internal tip position at each Vertex (or nullopt) */
  boost::property_map<Graph, vertex_tip_position_t>::type tipPositionProperty_;

  /** \brief Access to the weights of each Edge */
  boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

  /** \brief Access the connected component of a vertex */
  boost::property_map<Graph, vertex_component_t>::type vertexComponentProperty_;

  /** \brief Access the validity state of a vertex */
  boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;

  /** \brief Access to the internal VoxelOctree* at each Edge (or nullptr) */
  boost::property_map<Graph, edge_voxel_cache_t>::type edgeVoxelsProperty_;

  /** \brief Access the validity state of an edge */
  boost::property_map<Graph, edge_flags_t>::type edgeValidityProperty_;

  /** \brief Number of connected components created so far. This is used as an
   * ID only, does not represent the actual number of components currently in
   * the graph. */
  size_type componentCount_{0};

  /** \brief The number of elements in each component in the VoxelCachedLazyPRM
   * roadmap. */
  std::map<size_type, size_type> componentSize_;

  /** \brief Objective cost function for PRM graph edges */
  ompl::base::OptimizationObjectivePtr opt_;

  ompl::base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

  size_type iterations_{0};

  // timing
  mutable std::unordered_map<std::string, util::FunctionTimer> timers_;

  IKControllerFunc ikController_;

}; // end of class VoxelCachedLazyPRM

ENUM_FLAG_OPERATORS(VoxelCachedLazyPRM::CreateRoadmapOption)
ENUM_FLAG_OPERATORS(VoxelCachedLazyPRM::RoadmapIkOpts)

} // end of namespace motion_planning


#endif // VOXEL_CACHED_LAZY_PRM_H
