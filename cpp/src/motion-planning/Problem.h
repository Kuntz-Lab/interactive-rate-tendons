#ifndef PROBLEM_H
#define PROBLEM_H

#include "Environment.h"
#include "VoxelBackboneMotionValidator.h"
#include "VoxelBackboneValidityChecker.h"
#include "VoxelEnvironment.h"
#include "VoxelValidityChecker.h"
#include "WSpaceGoal.h"
#include <collision/VoxelOctree.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>

#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cpptoml {
class table;
}

namespace motion_planning {

class Environment;

using Options = std::map<std::string, std::string>;

/** Checks for state validity
 *
 * Is valid means not in collision and within tension and length limits.
 *
 * @param state: control vector
 *     (tension + rotation (if enabled) + retraction (if enabled))
 * @param robot: robot description
 * @param env: collision environment
 * @param home_shape: shape at zero tension, but the rest of the state
 * @param fk_shape: shape at state
 *
 * @return true if it is a valid state
 */
bool is_valid_state(const std::vector<double> &state,
                    const tendon::TendonRobot &robot,
                    const Environment &env,
                    const tendon::TendonResult &home_shape,
                    const tendon::TendonResult &fk_shape);

struct Problem {
  using PlanType = std::vector<std::vector<double>>;

  tendon::TendonRobot robot;
  Environment env;
  VoxelEnvironment venv;            // only used when voxels are enabled
  std::vector<double> start;        // starting tension for each tendon
  std::vector<double> goal;         // goal tension for each tendon
  double min_tension_change = 0.02; // min tension change before needing to redo
                                    // collision checking
  double min_rotation_change = 0.01;     // min drot between collision checks
  double min_retraction_change = 0.0001; // min dret between collision checks
  double start_rotation   = 0.0;    // start state rotation   (if enabled)
  double start_retraction = 0.0;    // start state retraction (if enabled)
  double goal_rotation    = 0.0;    // goal  state rotation   (if enabled)
  double goal_retraction  = 0.0;    // goal  state retraction (if enabled)
  bool sample_like_sphere = true;   // planner to sample retraction like r of sphere

  /// full start state including rotation and retraction (if enabled)
  std::vector<double> start_state() const {
    auto state = start;
    if (robot.enable_rotation  ) { state.emplace_back(start_rotation);   }
    if (robot.enable_retraction) { state.emplace_back(start_retraction); }
    return state;
  }

  /// full goal state including rotation and retraction (if enabled)
  std::vector<double> goal_state() const {
    auto state = goal;
    if (robot.enable_rotation  ) { state.emplace_back(goal_rotation);   }
    if (robot.enable_retraction) { state.emplace_back(goal_retraction); }
    return state;
  }

  /// start shape with retraction and rotation (if enabled)
  tendon::TendonResult start_shape() const {
    return robot.shape(start, start_rotation, start_retraction);
  }

  /// goal shape with retraction and rotation (if enabled)
  tendon::TendonResult goal_shape() const {
    return robot.shape(goal, goal_rotation, goal_retraction);
  }

  /// start shape with retraction (if enabled)
  tendon::TendonResult start_home_shape() const {
    double retract = robot.enable_retraction ? start_retraction : 0.0;
    return robot.home_shape(retract);
  }

  /// goal shape with retraction (if enabled)
  tendon::TendonResult goal_home_shape() const {
    double retract = robot.enable_retraction ? goal_retraction : 0.0;
    return robot.home_shape(retract);
  }

  /** Checks for state validity
   *
   * Is valid means not in collision and within tension and length limits.
   * Many overloads provided to help not recompute shapes if already available
   *
   * @param state: tension control vector + rotation + retraction, in that order
   *     The rotation and retraction are only to be present if enabled.
   *
   * @return true if it is a valid state
   */
  bool is_valid(const std::vector<double> &state) const {
    return is_valid(state, robot.home_shape(state), robot.shape(state));
  }

  /** Checks for state validity
   *
   * Is valid means not in collision and within tension and length limits.
   * Overloaded for convenience and performance.
   *
   * @param state: tension control vector + rotation + retraction, in that order
   *     The rotation and retraction are only to be present if enabled.
   * @param home_shape: home shape already computed at state
   *
   * @return true if it is a valid state
   */
  bool is_valid(const std::vector<double> &state,
                const tendon::TendonResult &home_shape) const
  { return is_valid(state, home_shape, robot.shape(state)); }

  /** Checks for state validity
   *
   * Is valid means not in collision and within tension and length limits.
   * Overloaded for convenience and performance.
   *
   * @param state: tension control vector + rotation + retraction, in that order
   *     The rotation and retraction are only to be present if enabled.
   * @param home_shape: shape computed at s_start but not yet rotated
   * @param shape: current state shape at s_start but not yet rotated
   *
   * @return true if it is a valid state
   */
  bool is_valid(const std::vector<double> &state,
                const tendon::TendonResult &home_shape,
                const tendon::TendonResult &shape) const;

  ompl::base::SpaceInformationPtr create_space_information() const;

  /** Creates a planner for the current problem
   *
   * The state validator used is motion_planning::ValidityChecker.  The motion
   * validator is OMPL's default motion validator.  Feel free to replace them
   * yourself on the returned planner with
   *
   *   planner->getSpaceInformation()->setValidtyChecker(...)
   *
   * In reality, the validity checker and motion validator are set in
   * create_space_information() which this function uses.
   *
   * This class has convenience functions for replacing the validity checker
   * and motion validator in either a planner or a space information,
   * set_voxel_state_checker() and set_voxel_swept_motion_checker().
   */
  ompl::base::PlannerPtr create_planner(
      const std::string &planner_name = "RRTConnect",
      const Options &planner_options = {},
      bool quiet = false) const;

  /// replaces the validity checker with a voxel-based one
  void set_voxel_state_checker(ompl::base::SpaceInformationPtr si,
                               const collision::VoxelOctree &voxels) const
  {
    si->setStateValidityChecker(std::make_shared<VoxelValidityChecker>(
          si, this->robot, this->venv, voxels));
  }

  void set_voxel_state_checker(ompl::base::PlannerPtr planner,
                               const collision::VoxelOctree &voxels) const
  {
    set_voxel_state_checker(planner->getSpaceInformation(), voxels);
  }

  /// replaces the validity checker with a voxel-based one on the backbone only
  void set_voxel_backbone_state_checker(ompl::base::SpaceInformationPtr si,
                                        const collision::VoxelOctree &voxels) const
  {
    si->setStateValidityChecker(std::make_shared<VoxelBackboneValidityChecker>(
          si, this->robot, this->venv, voxels));
  }

  void set_voxel_backbone_state_checker(ompl::base::PlannerPtr planner,
                                        const collision::VoxelOctree &voxels) const
  {
    set_voxel_backbone_state_checker(
        planner->getSpaceInformation(), voxels);
  }

  /// replaces the motion checker with a voxel-based backbone swept volume
  void set_voxel_swept_motion_checker(ompl::base::SpaceInformationPtr si,
                                      const collision::VoxelOctree &voxels) const
  {
    si->setMotionValidator(std::make_shared<VoxelBackboneMotionValidator>(
          si, this->robot, this->venv, voxels));
  }

  void set_voxel_swept_motion_checker(ompl::base::PlannerPtr planner,
                                      const collision::VoxelOctree &voxels) const
  {
    set_voxel_swept_motion_checker(
        planner->getSpaceInformation(), voxels);
  }

  /** Helper to load the voxel environment and set validators
   *
   * Loads the voxel object from this->venv.filename and calls the associated
   * functions (depending on backbone and swept_volume values) to set the
   * appropriate state and motion validators.
   *
   * The filename will be loaded based on the filename extension.  For example,
   * if it ends in ".nrrd", then we assume it is a NRRD file.  If it ends with
   * ".json", we assume it is stored as it is created from
   * VoxelOctree<N>::to_json().
   *
   * @param planner: OMPL planner like from create_planner()
   * @param backbone: true means only check against the backbone.  This means
   *   the voxel object loaded from this->venv.filename is already dilated by
   *   the robot radius.
   * @param swept_volume: true means to use the swept motion checker.  Note,
   *   you cannot set this to true if backbone is false, the swept volume on
   *   the full robot with radius is not currently implemented.
   */
  void update_to_voxel_validators(ompl::base::PlannerPtr planner,
                                  bool backbone,
                                  bool swept_volume) const;

  std::shared_ptr<WSpaceGoal>
  update_to_wspace_goal(ompl::base::PlannerPtr planner,
                        const Eigen::Vector3d &tip_desired,
                        double threshold = 5e-4) const;

  double plan_cost(const PlanType &plan) const;

  bool operator==(const Problem &other) const {
    return
      robot              == other.robot              &&
      env                == other.env                &&
      venv               == other.venv               &&
      start              == other.start              &&
      goal               == other.goal               &&
      min_tension_change == other.min_tension_change &&
      start_rotation     == other.start_rotation     &&
      start_retraction   == other.start_retraction   &&
      goal_rotation      == other.goal_rotation      &&
      goal_retraction    == other.goal_retraction;
  }

  /** Extends another plan to this one, making sure continuous rotation
   *
   * If the extension starts with the same state as the end of the full_plan,
   * then it is detected and not duplicated.
   */
  void extend_plan(PlanType &full_plan, const PlanType &extension) const;

  /** Fix the angle part of the plan to be continuous and remove duplicates */
  void make_plan_continuous(PlanType &plan) const;

  static PlanType read_plan(std::istream &in);
  void write_plan(std::ostream &out, const PlanType &plan) const;
  static PlanType load_plan(const std::string &plan_csv);
  void save_plan(const std::string &plan_csv, const PlanType &plan) const;

  std::shared_ptr<cpptoml::table> to_toml() const;
  static Problem from_toml(std::shared_ptr<cpptoml::table> tbl);
};

} // end of namespace motion_planning

#endif // PROBLEM_H
