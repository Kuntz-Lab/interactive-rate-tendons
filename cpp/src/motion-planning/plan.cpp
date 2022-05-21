#include "plan.h"

#include "Problem.h"
#include "Environment.h"
#include "ompl_planners.h"

#include <collision/collision.h>
#include <csv/Csv.h>
#include <util/angles.h>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Time.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace motion_planning {

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace otime = ompl::time;

namespace {

const std::string tension_state_space_name    = "TensionStateSpace";
const std::string retraction_state_space_name = "RetractionStateSpace";
const std::string rotation_state_space_name   = "RotationStateSpace";

} // end of unnamed namespace

ob::Cost solution_cost(ob::PlannerPtr planner) {
  const auto pdef = planner->getProblemDefinition();
  const auto opt = pdef->getOptimizationObjective();
  const auto path = pdef->getSolutionPath();
  if (path) {
    const auto gpath = path->as<og::PathGeometric>();
    return gpath->cost(opt);
  } else {
    return opt->infiniteCost();
  }
}

Problem::PlanType get_solution(const ob::PlannerPtr planner) {
  // get path states
  const auto pdef = planner->getProblemDefinition();
  const auto si = pdef->getSpaceInformation();
  const auto space = si->getStateSpace()->as<ob::CompoundStateSpace>();
  const auto path = pdef->getSolutionPath();
  if (!path) {
    std::cerr << "Warning: no solution available\n";
    return {};
  }
  const auto gpath = path->as<og::PathGeometric>();
  const auto states = gpath->getStates();

  Problem::PlanType best_path;
  double prev_rotation = 0.0;
  for (auto &state : states) {
    const auto &cstate = state->as<ob::CompoundStateSpace::StateType>();
    const auto tau_idx = space->getSubspaceIndex(tension_state_space_name);
    const auto &tau_state =
        cstate->as<ob::RealVectorStateSpace::StateType>(tau_idx);
    const auto &tau_space = space->as<ob::RealVectorStateSpace>(tau_idx);
    std::vector<double> config;
    tau_space->copyToReals(config, tau_state);
    if (space->hasSubspace(rotation_state_space_name)) {
      const auto rot_idx = space->getSubspaceIndex(rotation_state_space_name);
      const auto &rot_state = cstate->as<ob::SO2StateSpace::StateType>(rot_idx);
      const auto &rot_value = rot_state->value;
      config.emplace_back(util::angle_close_to(rot_value, prev_rotation));
      prev_rotation = rot_value;
    }
    if (space->hasSubspace(retraction_state_space_name)) {
      const auto ret_idx = space->getSubspaceIndex(retraction_state_space_name);
      const auto &ret_state =
          cstate->as<ob::RealVectorStateSpace::StateType>(ret_idx);
      config.emplace_back(ret_state->values[0]);
    }
    best_path.emplace_back(config);
  }

  return best_path;
}

Problem::PlanType
plan(const Problem &problem,
     const std::string &outfile_base,
     const std::string &planner_name,
     double timeout,
     const Options &planner_options,
     bool optimize)
{
  auto planner = problem.create_planner(planner_name, planner_options);
  WritePlanFunc write_plan
      = [&problem] (const std::string &fname, const Problem::PlanType &path) {
          problem.save_plan(fname, path);
        };
  return plan(planner, outfile_base, write_plan, timeout, optimize);
}

Problem::PlanType plan(
    ompl::base::PlannerPtr planner,
    const std::string &outfile_base,
    std::function<void(const std::string&,
                       const Problem::PlanType&)> &write_func,
    double timeout,
    bool optimize)
{
  auto remaining_time = timeout;
  bool is_opt = optimize && planner->getSpecs().optimizingPaths;
  if (optimize && !is_opt) {
    std::cerr << "Warning: optimized planning requested but "
              << planner->getName() << " is not an anytime algorithm\n";
  }

  int iteration = 0;
  do {
    iteration++;

    // solve
    auto before = otime::now();
    auto solve_status = planner->ob::Planner::solve(remaining_time);
    auto time_diff = otime::seconds(otime::now() - before);
    remaining_time -= time_diff;
    std::cout << "\n"
                 "Solution obtained after " << time_diff << " seconds\n"
                 "  Cost: " << solution_cost(planner) << "\n"
                 "\n";

    // save without counting the time
    auto filename = outfile_base;
    if (is_opt) { filename += "-" + std::to_string(iteration); }
    filename += ".csv";

    switch(ob::PlannerStatus::StatusType(solve_status)) {
      case ob::PlannerStatus::UNKNOWN:
        std::cerr << "Warning: unknow problem occured in planning\n";
        break;
      case ob::PlannerStatus::INVALID_START:
        std::cerr << "Warning: invalid start\n";
        break;
      case ob::PlannerStatus::INVALID_GOAL:
        std::cerr << "Warning: invalid goal\n";
        break;
      case ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
        std::cerr << "Warning: unrecognized goal type\n";
        break;
      case ob::PlannerStatus::TIMEOUT:
        std::cerr << "Warning: timeout\n";
        break;
      case ob::PlannerStatus::APPROXIMATE_SOLUTION:
        std::cerr << "Warning: could only find approximate solution\n";
        write_func(filename, get_solution(planner));
        break;
      case ob::PlannerStatus::EXACT_SOLUTION:
        std::cout << "Exact solution obtained\n";
        write_func(filename, get_solution(planner));
        break;
      case ob::PlannerStatus::CRASH:
        std::cerr <<
          "Warning: motion-planner seems to have crashed your system";
        break;
      case ob::PlannerStatus::ABORT:
        std::cerr << "Warning: motion-planner was aborted.";
        break;
      case ob::PlannerStatus::TYPE_COUNT:
        std::cerr << "Warning: motion-planner had a miscellaneous error.";
        break;
    }

    // update the optimization objective
    auto pdef = planner->getProblemDefinition();
    auto *path = static_cast<og::PathGeometric*>(pdef->getSolutionPath().get());
    auto opt = pdef->getOptimizationObjective();
    opt->setCostThreshold(ob::Cost(
          path->cost(opt).value() * (1.0 - std::numeric_limits<double>::epsilon())));

  } while (is_opt && remaining_time > 0);

  return get_solution(planner);
}

} // end of namespace motion_planning
