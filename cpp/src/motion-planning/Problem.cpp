#include "motion-planning/Problem.h"
#include "motion-planning/RetractionSampler.h"
#include "motion-planning/ValidityChecker.h"
#include "motion-planning/WSpaceGoal.h"
#include "motion-planning/ompl_planners.h"
#include <collision/VoxelOctree.h>
#include <collision/collision.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <csv/Csv.h>
#include <util/angles.h>
#include <util/openfile_check.h>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <Eigen/Core>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cmath>

namespace motion_planning {

namespace E = Eigen;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

// must call space->freeState(returned_state) when done with it
ob::State* to_ompl_state(ob::StateSpacePtr space,
                         std::vector<double> state)
{
  auto ostate = space->allocState();
  space->copyFromReals(ostate, state);
  return ostate;
}

// unused
//std::vector<double> from_ompl_state(ob::StateSpacePtr space, ob::State *state) {
//  std::vector<double> vals;
//  space->copyToReals(vals, state);
//  return vals;
//}

ob::OptimizationObjectivePtr create_optimization_objective(
    ob::SpaceInformationPtr si)
{
  // set the objective to finding a successful path of any cost
  // this will cause optimizing planners to return the first solution they find
  auto opt(std::make_shared<ob::PathLengthOptimizationObjective>(si));
  opt->setCostThreshold(opt->infiniteCost());
  return opt;
}

inline void set_options(ob::PlannerPtr planner, const Options &options) {
  auto &params = planner->params();
  for (auto &kv : options) {
    bool success = params.setParam(kv.first, kv.second);
    if (!success) {
      throw std::invalid_argument("Invalid option for planner: "
                                  + kv.first);
    }
  }
}

} // end of unnamed namespace

bool is_valid_state(const std::vector<double> &state,
                    const tendon::TendonRobot &robot,
                    const Environment &env,
                    const tendon::TendonResult &home_shape,
                    const tendon::TendonResult &fk_shape)
{
  if (!robot.is_valid(state, home_shape, fk_shape)) {
    return false;
  }
  collision::CapsuleSequence robot_shape {fk_shape.p, robot.r};
  return !env.collides(robot_shape);
}

bool Problem::is_valid(const std::vector<double> &state,
                       const tendon::TendonResult &home_shape,
                       const tendon::TendonResult &fk_shape) const
{
  return is_valid_state(state, robot, env, home_shape, fk_shape);
}

ob::SpaceInformationPtr Problem::create_space_information() const {
  //
  // create the compound state space
  //
  auto space(std::make_shared<ob::CompoundStateSpace>());
  space->setName("CompoundStateSpace");

  // TODO: update these LongestValidSegmentFraction values to be minimum change
  //       seen in the dynamic one used to voxelize the backbone's swept volume.

  // tendon state space
  auto tendon_space(std::make_shared<ob::RealVectorStateSpace>());
  tendon_space->setName(ValidityChecker::tension_state_space_name());
  space->addSubspace(tendon_space, 1.0);
  for (auto &tendon : robot.tendons) {
    tendon_space->addDimension(0.0, tendon.max_tension);
  }
  const auto tendon_extent = tendon_space->getMaximumExtent();
  tendon_space->setLongestValidSegmentFraction(
      min_tension_change / tendon_extent);

  // rotation state space
  if (robot.enable_rotation) {
    auto rotation_space(std::make_shared<ob::SO2StateSpace>());
    rotation_space->setName(ValidityChecker::rotation_state_space_name());
    // one rotation is the same distance as half of the largest tension dim
    // change (rotation is relatively cheap to do)
    double weight = tendon_extent / (4.0 * M_PI);
    space->addSubspace(rotation_space, weight);
    rotation_space->setLongestValidSegmentFraction(
        min_rotation_change / (2 * M_PI));
  }

  // retraction state space
  if (robot.enable_retraction) {
    auto retraction_space(std::make_shared<ob::RealVectorStateSpace>());
    retraction_space->setName(ValidityChecker::retraction_state_space_name());
    // fully retracting is twice the distance of the largest possible tension
    // change (retraction is relatively expensive to do)
    double weight = 2.0 * tendon_extent / robot.specs.L;
    space->addSubspace(retraction_space, weight);
    retraction_space->addDimension(0.0, robot.specs.L);
    retraction_space->setLongestValidSegmentFraction(
        std::min(0.01, min_retraction_change / robot.specs.L));
    if (sample_like_sphere) {
      // set a custom state space sampler
      retraction_space->setStateSamplerAllocator(
          [](const ob::StateSpace* space) -> ob::StateSamplerPtr {
            return std::make_shared<RetractionSampler>(space);
          });
    }
  }

  space->setup();

  // space information
  auto si(std::make_shared<ob::SpaceInformation>(space));
  si->setStateValidityChecker(
      std::make_shared<ValidityChecker>(si, robot, env));
  si->setup();

  return si;
}

ob::PlannerPtr Problem::create_planner(
    const std::string &plannerName,
    const Options &plannerOptions,
    bool quiet) const
{
  const size_t N = robot.tendons.size();

  // validate input
  if (start.size() != N) {
    throw std::out_of_range("tendons and start are not the same length");
  }
  if (goal.size() != N) {
    throw std::out_of_range("start and goal are not the same length");
  }

  auto si = create_space_information();
  auto space = si->getStateSpace()->as<ob::CompoundStateSpace>();

  // start and goal
  ob::ScopedState<> start_state(si);
  ob::ScopedState<> goal_state(si);
  for (size_t i = 0; i < N; i++) {
    start_state[i] = start[i];
    goal_state[i] = goal[i];
  }
  if (robot.enable_retraction) {
    auto ext_idx = space->getSubspaceIndex(
        ValidityChecker::retraction_state_space_name());
    start_state->as<ob::CompoundStateSpace::StateType>()
               ->as<ob::RealVectorStateSpace::StateType>(ext_idx)
               ->values[0]
               = start_retraction;
    goal_state ->as<ob::CompoundStateSpace::StateType>()
               ->as<ob::RealVectorStateSpace::StateType>(ext_idx)
               ->values[0]
               = goal_retraction;
  }
  if (robot.enable_rotation) {
    auto rot_idx = space->getSubspaceIndex(
        ValidityChecker::rotation_state_space_name());
    start_state->as<ob::CompoundStateSpace::StateType>()
               ->as<ob::SO2StateSpace::StateType>(rot_idx)
               ->value = start_rotation;
    goal_state ->as<ob::CompoundStateSpace::StateType>()
               ->as<ob::SO2StateSpace::StateType>(rot_idx)
               ->value = goal_rotation;
  }

  // create the problem definition
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));
  pdef->setStartAndGoalStates(start_state, goal_state);

  pdef->setOptimizationObjective(create_optimization_objective(si));

  // create planner
  auto planner = make_planner(si, plannerName);
  set_options(planner, plannerOptions);
  if (!quiet) {
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
  }
  planner->setProblemDefinition(pdef);
  //planner->setup(); // let this happen later since the user may change it

  return planner;
}

void Problem::update_to_voxel_validators(ob::PlannerPtr planner,
                                         bool backbone,
                                         bool swept_volume) const
{
  auto voxels = this->venv.get_obstacles();
  if (backbone) {
    this->set_voxel_backbone_state_checker(planner, *voxels);
  } else {
    this->set_voxel_state_checker(planner, *voxels);
  }
  if (swept_volume) {
    this->set_voxel_swept_motion_checker(planner, *voxels);
  }
}

std::shared_ptr<WSpaceGoal>
Problem::update_to_wspace_goal(ob::PlannerPtr planner,
                               const E::Vector3d &tip_desired,
                               double threshold) const
{
  auto si       = planner->getSpaceInformation();
  auto pdef     = planner->getProblemDefinition();
  auto new_goal = std::make_shared<motion_planning::WSpaceGoal>(si, tip_desired);
  new_goal->setThreshold(threshold);
  new_goal->setDesiredTip(tip_desired);
  pdef->setGoal(new_goal);
  return new_goal;
}

double Problem::plan_cost(const Problem::PlanType &plan) const {
  auto si    = create_space_information();
  auto space = si->getStateSpace();
  auto opt   = create_optimization_objective(si);
  og::PathGeometric path(si);

  struct StateDeleter {
    ob::StateSpacePtr space;
    ob::State *state;
    bool owns;
    StateDeleter(ob::StateSpacePtr space_, ob::State *state_)
      : space(space_), state(state_), owns(true) {}
    StateDeleter(const StateDeleter &other) = delete; // disable copy
    StateDeleter(StateDeleter &&other) {
      space = std::move(other.space);
      state = other.state;
      owns = other.owns;
      other.owns = false;
    }
    StateDeleter& operator=(const StateDeleter &other) = delete;
    StateDeleter& operator=(StateDeleter &&other) {
      if (owns) { space->freeState(state); }
      space = std::move(other.space);
      state = other.state;
      owns = other.owns;
      other.owns = false;
      return *this;
    }
    ~StateDeleter() { if (owns) { space->freeState(state); } }
  };

  std::vector<StateDeleter> states;
  for (auto config : plan) {
    auto state = to_ompl_state(space, config);
    states.emplace_back(space, state);
    path.append(state);
  }

  return path.cost(opt).value();
}

void Problem::extend_plan(Problem::PlanType &full_plan,
                          const Problem::PlanType &extension) const
{
  // copy the extension to the full plan
  full_plan.insert(full_plan.end(), extension.begin(), extension.end());

  // fix plan
  this->make_plan_continuous(full_plan);
}

void Problem::make_plan_continuous(Problem::PlanType &plan) const {
  if (plan.empty()) { return; }

  // make angles continuous
  if (this->robot.enable_rotation) {
    size_t rotation_index = plan[0].size() - 1;
    if (this->robot.enable_retraction) {
      rotation_index -= 1;
    }
    double last_angle = plan[0][rotation_index];
    for (size_t i = 1; i < plan.size(); ++i) {
      double angle = plan[i][rotation_index];
      angle = util::angle_close_to(angle, last_angle);
      plan[i][rotation_index] = angle;
      last_angle = angle;
    }
  }

  // matches the implementation from OMPL of equalStates() which is flawed
  auto equal_vals = [](const std::vector<double> &a, const std::vector<double> &b) {
    if (a.size() != b.size()) { return false; }
    for (size_t i = 0; i < a.size(); ++i) {
      double diff = std::abs(a[i] - b[i]);
      if (diff > std::numeric_limits<double>::epsilon() * 2.0) {
        return false;
      }
    }
    return true;
  };

  // remove duplicates
  PlanType new_plan;
  new_plan.emplace_back(std::move(plan[0]));
  for (size_t i = 1; i < plan.size(); ++i) {
    if (!equal_vals(new_plan.back(), plan[i])) {
      new_plan.emplace_back(std::move(plan[i]));
    }
  }
  plan = std::move(new_plan);  // overwrite
}

Problem::PlanType Problem::read_plan(std::istream &in) {
  csv::CsvReader reader(in);

  // get indices of control values
  auto *header = reader.header();
  std::vector<size_t> indices;
  size_t idx;
  for (size_t i = 1; (idx = header->index("tau_" + std::to_string(i))) != header->npos; i++) {
    indices.emplace_back(idx);
  }
  if ((idx = header->index("theta")) != header->npos) {
    indices.emplace_back(idx);
  }
  if ((idx = header->index("s_start")) != header->npos) {
    indices.emplace_back(idx);
  }

  // read the rows
  std::vector<std::vector<double>> plan;
  csv::CsvRow row;
  while (reader >> row) {
    std::vector<double> config;
    for (auto i : indices) {
      config.emplace_back(std::stod(row.at(i)));
    }
    plan.emplace_back(std::move(config));
  }
  return plan;
}

void Problem::write_plan(std::ostream &out, const Problem::PlanType &plan) const {
  csv::CsvWriter writer(out);

  // print header
  writer << "i";
  size_t N = robot.tendons.size();
  for (size_t i = 1; i <= N; i++) { writer << "tau_" + std::to_string(i); }
  if (robot.enable_rotation)      { writer << "theta"; }
  if (robot.enable_retraction)    { writer << "s_start"; }
  writer.new_row();

  // print plan
  for (size_t i = 0; i < plan.size(); i++) {
    const auto &config = plan[i];
    writer << i + 1;
    for (const auto &val : config) {
      writer << val;
    }
    writer.new_row();
  }
}

Problem::PlanType Problem::load_plan(const std::string &plan_csv) {
  std::ifstream in;
  util::openfile_check(in, plan_csv);
  return read_plan(in);
}

void Problem::save_plan(const std::string &plan_csv,
                        const Problem::PlanType &plan) const
{
  std::ofstream out;
  util::openfile_check(out, plan_csv);
  write_plan(out, plan);
}


std::shared_ptr<cpptoml::table> Problem::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("problem", tbl);

  tbl->insert("start", cpptoml::to_toml(start));
  tbl->insert("goal", cpptoml::to_toml(goal));
  tbl->insert("min_tension_change", min_tension_change);
  tbl->insert("min_rotation_change", min_rotation_change);
  tbl->insert("min_retraction_change", min_retraction_change);
  tbl->insert("start_rotation", start_rotation);
  tbl->insert("start_retraction", start_retraction);
  tbl->insert("goal_rotation", goal_rotation);
  tbl->insert("goal_retraction", goal_retraction);
  tbl->insert("sample_like_sphere", sample_like_sphere);

  // we want to flatten the toml output.  So take the contents of robot and env
  // and put them on the top-level (i.e., merge their containers with this one)
  std::vector<cpptoml::table_ptr> sub_tables
      {robot.to_toml(), env.to_toml(), venv.to_toml()};
  for (auto &sub_tbl : sub_tables) {
    for (auto &[key, val] : *sub_tbl) {
      container->insert(key, val);
    }
  }

  return container;
}

Problem Problem::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto problem_tbl = tbl->get("problem")->as_table();
  if (!problem_tbl) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'problem': not a table");
  }

  auto start              = problem_tbl->get("start")->as_array();
  auto goal               = problem_tbl->get("goal")->as_array();
  auto min_tension_change = problem_tbl->get("min_tension_change")->as<double>();
  if (!(start && goal && min_tension_change)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }

  Problem problem;
  problem.robot              = tendon::TendonRobot::from_toml(tbl);
  problem.env                = Environment::from_toml(tbl);
  problem.start              = cpptoml::to_stdvec<double>(start);
  problem.goal               = cpptoml::to_stdvec<double>(goal);
  problem.min_tension_change = min_tension_change->get();

  // all or nothing with voxel environment
  if (tbl->contains("voxel_environment")) {
    problem.venv = VoxelEnvironment::from_toml(tbl);
  }

  //
  // optional fields
  //

  // we want to read start and goal rotation every time, but only require them
  // when rotation is enabled.

  if (problem_tbl->contains("start_rotation")) {
    auto start_rotation = problem_tbl->get("start_rotation")->as<double>();
    if (!start_rotation) {
      throw cpptoml::parse_exception("Wrong type detected for start_rotation");
    } else {
      problem.start_rotation = start_rotation->get();
    }
  } else if (problem.robot.enable_rotation) {
    throw std::out_of_range(
        "Must specify start_rotation if rotation is enabled");
  }

  if (problem_tbl->contains("goal_rotation")) {
    auto goal_rotation = problem_tbl->get("goal_rotation")->as<double>();
    if (!goal_rotation) {
      throw cpptoml::parse_exception("Wrong type detected for goal_rotation");
    } else {
      problem.goal_rotation = goal_rotation->get();
    }
  } else if (problem.robot.enable_rotation) {
    throw std::out_of_range(
        "Must specify goal_rotation if rotation is enabled");
  }

  if (problem_tbl->contains("min_rotation_change")) {
    auto min_rotation_change = problem_tbl->get("min_rotation_change")->as<double>();
    if (!min_rotation_change) {
      throw cpptoml::parse_exception("Wrong type detected for min_rotation_change");
    } else {
      problem.min_rotation_change = min_rotation_change->get();
    }
  } else if (problem.robot.enable_rotation) {
    std::cerr << "Warning: min_rotation_change not found in problem toml" << std::endl;
  }

  if (problem_tbl->contains("start_retraction")) {
    auto start_retraction = problem_tbl->get("start_retraction")->as<double>();
    if (!start_retraction) {
      throw cpptoml::parse_exception(
          "Wrong type detected for start_retraction");
    } else {
      problem.start_retraction = start_retraction->get();
    }
  } else if (problem.robot.enable_retraction) {
    throw std::out_of_range(
        "Must specify start_retraction if retraction is enabled");
  }

  if (problem_tbl->contains("goal_retraction")) {
    auto goal_retraction = problem_tbl->get("goal_retraction")->as<double>();
    if (!goal_retraction) {
      throw cpptoml::parse_exception("Wrong type detected for goal_retraction");
    } else {
      problem.goal_retraction = goal_retraction->get();
    }
  } else if (problem.robot.enable_retraction) {
    throw std::out_of_range(
        "Must specify goal_retraction if retraction is enabled");
  }

  if (problem_tbl->contains("min_retraction_change")) {
    auto min_retraction_change = problem_tbl->get("min_retraction_change")->as<double>();
    if (!min_retraction_change) {
      throw cpptoml::parse_exception("Wrong type detected for min_retraction_change");
    } else {
      problem.min_retraction_change = min_retraction_change->get();
    }
  } else if (problem.robot.enable_rotation) {
    std::cerr << "Warning: min_retraction_change not found in problem toml" << std::endl;
  }

  if (problem_tbl->contains("sample_like_sphere")) {
    auto sample_like_sphere = problem_tbl->get("sample_like_sphere")->as<bool>();
    if (!sample_like_sphere) {
      throw cpptoml::parse_exception("Wrong type detected for sample_like_sphere");
    }
    problem.sample_like_sphere = sample_like_sphere->get();
  }

  return problem;
}

} // end of namespace motion_planning
