#include "cliparser/CliParser.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "motion-planning/VoxelCachedLazyPRM.h"
#include "motion-planning/plan.h"
#include "util/FunctionTimer.h"
#include "util/StopWatch.h"
#include "util/macros.h"
#include "util/ompl_logging.h"
#include "util/openfile_check.h"
#include "util/time_function_call.h"
#include "util/vector_ops.h"

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalStates.h>

#include <Eigen/Core>

#include <boost/filesystem.hpp>

#include <iostream>
#include <memory>
#include <string>

#include <cstdio>  // for std::snprintf()
#include <cstdlib> // for std::exit()

namespace E = Eigen;
namespace bfs = boost::filesystem;
namespace ob = ompl::base;

namespace {

namespace defaults {
  const std::string output         = "chained-plan.csv";
  const std::string log            = "chained-plan-log.csv";
  const std::string ompl_log_level = "DEBUG";
  const std::string fk_type        = "fast";
  const size_t      ik_neighbors   = 5;
  const double      ik_tolerance   = 0.0005;
  const size_t      ik_max_iters   = 10;
  const double      ik_mu_init     = 1.0;
  const double      timeout        = 20; // seconds
} // end of namespace defaults

void populate_parser (CliParser &parser) {
  parser.set_program_description(
        "Plan using the VoxelCachedLazyPRM with a roadmap on a sequence of\n"
      "  W-Space waypoints.  This will generate a plan between each waypoint.\n"
      "  You can keep the intermediate plans by specifying an intermediate\n"
      "  plan output directory.  Either way, this will output a single file\n"
      "  containing the combined plan.\n"
      "\n"
      "  By default, the current step starts from the ending configuration\n"
      "  of the previous step (i.e., the last configuration of the last\n"
      "  step's found plan).  This can be changed with --common-start to\n"
      "  have each W-Space waypoint to be planned from the same start\n"
      "  configuration, specifically the one specified in the problem toml.\n"
      "\n"
      "  Note: this app only supports the VoxelCachedLazyPRM planner and\n"
      "  only supports it with the backbone specification of only doing\n"
      "  collision checking on the shape of the backbone portion of the\n"
      "  robot.  It also only supports the swept volume option where a swept\n"
      "  volume of the backbone is calculated and voxelized.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem toml description file.\n"
      "                The start configuration is used as the starting\n"
      "                point, but the goal configuration is ignored.\n"
      "                Instead the waypoints are used, and after the final\n"
      "                waypoint, it will return back to the start\n"
      "                configuration.");

  parser.add_positional("roadmap");
  parser.set_required("roadmap");
  parser.set_description("roadmap", "PRM roadmap file\n"
      "                Supports toml, toml.gz, json, bson, cbor, msgpack,\n"
      "                and ubjson.");

  parser.add_positional("waypoints");
  parser.set_required("waypoints");
  parser.set_description("waypoints",
                      "CSV file containing tip positions to plan to.\n"
      "                Expected columns are:\n"
      "                - tip_x\n"
      "                - tip_y\n"
      "                - tip_z\n"
      "                Extra columns are ignored.\n");

  parser.add_flag("-c", "--common-start");
  parser.set_description("--common-start",
                      "Have each step's starting configuration be from the\n"
      "                configuration given in the problem toml.  By default,\n"
      "                just the first step uses the problem's start\n"
      "                configuration, and then they daisy-chain off of each\n"
      "                other.");

  parser.add_argflag("-i", "--intermediate-dir");
  parser.set_description("--intermediate-dir",
                      "By default, the plans between waypoints are thrown\n"
      "                away after being merged into the full plan at the\n"
      "                end.  However, if the user would like to keep these\n"
      "                intermediate plans, they can specify this directory\n"
      "                and all plans will be stored here as 'NN-plan.csv'\n"
      "                where NN is replaced by the waypoint index (starting\n"
      "                with 1) from the waypoints file that is used as the\n"
      "                goal.");
  
  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                      "CSV file output for the final plan.\n"
      "                (default is '" + defaults::output + "')");

  parser.add_argflag("-l", "--log"); // CSV for timing events
  parser.set_description("--log",
                      "CSV file containing raw profiling data with computed\n"
      "                statistics at the end.  There are only two columns,\n"
      "                - name\n"
      "                - milestone: which milestone this applies to\n"
      "                - value\n"
      "                (default is '" + defaults::log + "')");

  parser.add_argflag("--ompl-log-level");
  parser.set_description("--ompl-log-level",
                      "Set the log level used in OMPL.  Choices (in order of\n"
      "                most to least verbose) are 'DEV2', 'DEV1', 'DEBUG',\n"
      "                'INFO', 'WARN', 'ERROR', 'NONE'.\n"
      "                (default is '" + defaults::ompl_log_level + "')");

  parser.add_flag("--skip-roadmap-vertex-check");
  parser.set_description("--skip-roadmap-vertex-check",
                      "Skip the step of checking for vertex collisions at\n"
      "                the time of loading the roadmap.  This causes\n"
      "                vertices to be checked during planning lazily.");

  parser.add_flag("--skip-roadmap-edge-check");
  parser.set_description("--skip-roadmap-edge-check",
                      "Skip the step of checking for edge collisions at\n"
      "                the time of loading the roadmap.  This causes\n"
      "                edges to be checked during planning lazily.");

  parser.add_argflag("-k", "--ik-neighbors");
  parser.set_description("--ik-neighbors",
                      "Number of neighbors from the roadmap to start IK\n"
      "                (default is "
                       + std::to_string(defaults::ik_neighbors) + ")");

  parser.add_argflag("--ik-tolerance");
  parser.set_description("--ik-tolerance",
                      "Tip error stopping tolerance for IK.\n"
      "                (default is "
                        + std::to_string(defaults::ik_tolerance) + " meters)");

  parser.add_argflag("--ik-max-iters");
  parser.set_description("--ik-max-iters",
                      "Maximum number of iterations for IK.\n"
      "                (default is "
                        + std::to_string(defaults::ik_max_iters) + ")");

  parser.add_argflag("--ik-mu-init");
  parser.set_description("--ik-mu-init",
                      "initial damping factor for Levenberg-Marquardt (LM)\n"
      "                (also known as damped least squares (DLS).\n"
      "                Convergence may depend heavily on this.  A larger\n"
      "                value means smaller iteration step sizes, a smaller\n"
      "                value means larger iteration step sizes (roughly).\n"
      "                (default is "
                        + std::to_string(defaults::ik_mu_init) + ")");

  parser.add_flag("--ik-lazy-add");
  parser.set_description("--ik-lazy-add",
                      "After finding one or more IK solutions that are\n"
      "                good, we connect them to the roadmap.  This\n"
      "                option makes those connections lazy, at least\n"
      "                the ones that weren't already evaluated.\n"
      "                Improves average runtime, but degrades the\n"
      "                worst-case time.");

  // TODO: remove --ik-auto-add, it doesn't make sense to have here
  parser.add_flag("--ik-auto-add");
  parser.set_description("--ik-auto-add",
                      "When doing roadmapIk, do not add to the roadmap,\n"
      "                but instead add as a goal state if/when used in\n"
      "                planning.");

  parser.add_flag("--ik-accurate");
  parser.set_description("--ik-accurate",
                      "After IK config, try to connect from many nearest\n"
      "                neighbors in the map rather than just the neighbor\n"
      "                used for IK.  If using --ik-lazy-add, then this is\n"
      "                only used if all IK solutions were either invalid or\n"
      "                not within threshold.");

  parser.add_argflag("-t", "--timeout");
  parser.set_description("--timeout", "Timeout for planning in seconds\n"
      "                (default is " + std::to_string(defaults::timeout) + ")");

  parser.add_flag("--keep-disconnected-vertices");
  parser.set_description("--keep-disconnected-vertices",
                      "Usually, the disconnected vertices are removed after\n"
      "                loading the roadmap.  We usually keep only the\n"
      "                largest connected component after collision check.\n"
      "                Using this flag keeps the vertices that are\n"
      "                disconnected from the largest connected component.");

  parser.add_flag("--fk");
  parser.set_description("--fk",
                      "Forward kinematics (FK) implementation to use.\n"
      "                Valid values are:\n"
      "                - 'fast': our fast FK model that does one integration\n"
      "                - 'slow-forward': the single-shoot method (wrapped in\n"
      "                    levmar optimization) using forward-differences\n"
      "                - 'slow-central': single-shoot with central-differences.\n"
      "                Default is 'fast'.\n");
}

void my_assert(bool value, const std::string &msg = "failed assert") {
  if (!value) {
    std::cerr << "\n\nmy_assert: " << msg
      << std::endl
      << std::endl
      << std::endl; // really really flush before exiting
    //std::exit(1);
    throw std::runtime_error(msg);
  }
}

void print_solve_status(ob::PlannerStatus::StatusType status) {
  using PS = ob::PlannerStatus;
  switch (status) {
    case PS::UNKNOWN:
      std::cerr << "Warning: unknown problem occurred in planning" << std::endl;
      break;
    case PS::INVALID_START:
      std::cerr << "Warning: invalid start" << std::endl;
      break;
    case PS::INVALID_GOAL:
      std::cerr << "Warning: invalid goal" << std::endl;
      break;
    case PS::UNRECOGNIZED_GOAL_TYPE:
      std::cerr << "Warning: unrecognized goal type" << std::endl;
      break;
    case PS::TIMEOUT:
      std::cerr << "Warning: timeout" << std::endl;
      break;
    case PS::APPROXIMATE_SOLUTION:
      std::cerr << "Warning: could only find approximate solution" << std::endl;
      break;
    case PS::EXACT_SOLUTION:
      std::cout << "Exact solution obtained" << std::endl;
      break;
    case PS::CRASH:
      std::cerr <<
        "Warning: motion-planner seems to have crashed your system" << std::endl;
      break;
    case PS::ABORT:
      std::cerr << "Warning: motion-planner was aborted." << std::endl;
      break;
    case PS::TYPE_COUNT:
      std::cerr << "Warning: motion-planner had a miscellaneous error." << std::endl;
      break;
  }
}

} // end of unnamed namespace

int main (int arg_count, char* arg_list[]) {
  using util::operator<<;

  CliParser parser;
  populate_parser(parser);

  parser.parse(arg_count, arg_list);

  const std::string planner_name = "VoxelCachedLazyPRM";
  const bool backbone     = true;
  const bool swept_volume = true;

  auto problem            =
      cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  auto roadmap            = parser["roadmap"];
  auto waypoints          = parser["waypoints"];
  bool common_start       = parser.has("--common-start");
  auto keep_intermediate  = parser.has("--intermediate-dir");
  auto intermediate_dir   = parser.get("--intermediate-dir", std::string());
  auto outfile            = parser.get("--output", defaults::output);
  auto logfile            = parser.get("--log", defaults::log);
  auto ompl_log_level     = parser.get("--ompl-log-level", defaults::ompl_log_level);
  auto check_vert_validity = !parser.has("--skip-roadmap-vertex-check");
  auto check_edge_validity = !parser.has("--skip-roadmap-edge-check");
  auto fk_type            = parser.get("--fk", defaults::fk_type);
  auto ik_N               = parser.get("--ik-neighbors", defaults::ik_neighbors);
  auto ik_tol             = parser.get("--ik-tolerance", defaults::ik_tolerance);
  auto ik_maxit           = parser.get("--ik-max-iters", defaults::ik_max_iters);
  auto ik_mu_i            = parser.get("--ik-mu-init", defaults::ik_mu_init);
  bool ik_lazy_add        = parser.has("--ik-lazy-add");
  bool ik_accurate        = parser.has("--ik-accurate");
  bool ik_auto_add        = parser.has("--ik-auto-add");
  auto timeout            = parser.get("--timeout", defaults::timeout);
  bool remove_disconnected_vertices = !parser.has("--keep-disconnected-vertices");

  util::set_ompl_log_level(ompl_log_level);

  if (keep_intermediate) {
    bfs::create_directories(intermediate_dir);
  }

  std::ofstream logfile_out;
  util::openfile_check(logfile_out, logfile);
  csv::CsvWriter log_writer(logfile_out);
  log_writer << "name" << "milestone" << "value";
  log_writer.new_row();
  auto log_event = [&log_writer](const auto &name, const auto &value) {
    log_writer << name << "N/A" << value;
    log_writer.new_row();
  };

  // save the settings to the log
  log_event("settings:problem",                parser["problem"]);
  log_event("settings:roadmap",                roadmap);
  log_event("settings:waypoints",              waypoints);
  log_event("settings:--planner-name",         planner_name);
  log_event("settings:--common-start",         common_start);
  if (keep_intermediate) {
    log_event("settings:--intermediate-dir",   intermediate_dir);
  }
  log_event("settings:--output",               outfile);
  log_event("settings:--log",                  logfile);
  log_event("settings:--ompl-log-level",       ompl_log_level);
  log_event("settings:--skip-roadmap-vertex-check", !check_vert_validity);
  log_event("settings:--skip-roadmap-edge-check",   !check_edge_validity);
  log_event("settings:--fk",                   fk_type);
  log_event("settings:--ik-neighbors",         ik_N);
  log_event("settings:--ik-tolerance",         ik_tol);
  log_event("settings:--ik-max-iters",         ik_maxit);
  log_event("settings:--ik-mu-init",           ik_mu_i);
  log_event("settings:--ik-lazy-add",          ik_lazy_add);
  log_event("settings:--ik-auto-add",          ik_auto_add);
  log_event("settings:--ik-accurate",          ik_accurate);
  log_event("settings:--timeout",              timeout);
  log_event("settings:--keep-disconnected-vertices", !remove_disconnected_vertices);

  using Planner = motion_planning::VoxelCachedLazyPRM;
  auto planner = problem.create_planner(planner_name);
  problem.update_to_voxel_validators(planner, backbone, swept_volume);
  auto vplanner =
      std::dynamic_pointer_cast<Planner>(planner);
  if (!vplanner) {
    std::cerr
      << "Error: planner is not an instance of motion_planning::VoxelCachedLazyPRM"
      << std::endl;
    return 1;
  }

  // prepare IK option
  auto ikopt = Planner::RMAP_IK_SIMPLE;
  if (ik_auto_add) { ikopt |= Planner::RMAP_IK_AUTO_ADD; }
  if (ik_lazy_add) { ikopt |= Planner::RMAP_IK_LAZY_ADD; }
  if (ik_accurate) { ikopt |= Planner::RMAP_IK_ACCURATE; }

  auto si    = planner->getSpaceInformation();
  auto space = si->getStateSpace();
  auto pdef  = planner->getProblemDefinition();
  auto checker = std::dynamic_pointer_cast<
      motion_planning::AbstractValidityChecker>(si->getStateValidityChecker());
  auto motion_validator = std::dynamic_pointer_cast<
      motion_planning::AbstractVoxelMotionValidator>(si->getMotionValidator());
  my_assert(pdef->getStartStateCount() == 1, "# start states != 1");
  my_assert(bool(checker), "State checker is not an AbstractValidityChecker");
  my_assert(bool(motion_validator),
            "Motion validator is not an AbstractVoxelMotionValidator");

  if (fk_type == "fast") {
    // do nothing
  } else {
    // use the slow FK function, set it in the checker
    double finite_difference_delta = 1e-8;
    if (fk_type == "slow-forward") {
      // do nothing
    } else if (fk_type == "slow-central") {
      // levmar says negative FD delta value results in central differences
      finite_difference_delta *= -1;
    } else {
      throw std::runtime_error("unsupported fk_type: " + fk_type);
    }

    // TODO: expose these parameters to cli
    auto zero = E::Vector3d{0, 0, 0};
    auto u_guess = zero;
    auto v_guess = E::Vector3d{0, 0, 1};
    auto empty_vec_func = [](auto,auto) { return E::Vector3d{0, 0, 0}; };
    auto maxiter = 500;
    auto mu_init = 1e-3;
    auto stop_threshold_JT_err_inf = 1e-12;
    auto stop_threshold_Dp = 1e-100;
    auto verbose = false;

    // set the slow FK function
    checker->set_fk_func([=, &robot=problem.robot]
        (const std::vector<double> &state) {
          return robot.general_shape(
              state,
              empty_vec_func,
              empty_vec_func,
              zero,
              zero,
              u_guess,
              v_guess,
              maxiter,
              mu_init,
              stop_threshold_JT_err_inf,
              stop_threshold_Dp,
              finite_difference_delta,
              verbose
          );
        });
  }


  std::ifstream waypoints_in;
  util::openfile_check(waypoints_in, waypoints);
  csv::CsvReader reader(waypoints_in);

  motion_planning::Problem::PlanType total_plan;

  //
  // helper lambda functions
  //
  auto state2vec = [space](ob::State* state) {
    std::vector<double> vec;
    space->copyToReals(vec, state);
    return vec;
  };

  auto vec2newstate = [si, space](const std::vector<double> &vec) {
    auto state = si->allocState();
    space->copyFromReals(state, vec);
    space->enforceBounds(state);
    return state;
  };

  auto state_vecs_equal = [si] (auto a, auto b) {
    // note: for the rotation space, the equalStates() method assumes the
    // rotation value is already in [-pi,pi].  We need to truncate it there
    // first.  Even then, they have too strict of a threshold.

    // use si to truncate a and b vectors within bounds
    ob::ScopedState<> tmp(si);
    my_assert(a.size() == b.size(), "vecs not the same sizes (1)");
    tmp = a; si->enforceBounds(tmp.get()); a = tmp.reals();
    tmp = b; si->enforceBounds(tmp.get()); b = tmp.reals();
    my_assert(a.size() == b.size(), "vecs not the same sizes (2)");

    double threshold = 1e-8; // FIXME: could do better than this
    for (size_t i = 0; i < a.size(); ++i) {
      if (std::abs(a[i] - b[i]) > threshold) { return false; }
    }
    return true;
  };

  // space-separated values for an iterable
  auto vec2str = [](const auto &v) {
    std::ostringstream builder;
    bool first = false;
    for (decltype(v.size()) i = 0; i < v.size(); ++i) {
      if (!first) { builder << " "; }
      first = false;
      builder << v[i];
    }
    return builder.str();
  };

  int milestone = 1;
  auto log_milestone_event = [&log_writer, &milestone]
    (const auto &name, const auto &value) {
      log_writer << name;
      if (milestone > 0) { log_writer << milestone; }
      else               { log_writer << "N/A"; }
      log_writer << value;
      log_writer.new_row();
      std::cout << milestone << ": Milestone Event: " << name << ": " << value << "\n";
    };

  // TODO: capture statistics here to congregate them later
  auto log_and_clear_timers = [&log_milestone_event](auto &timed_object) {
    for (const auto &[name, timer] : timed_object->timers()) {
      log_milestone_event("calls:" + name, timer.get_times().size());
      if (timer.get_times().size() > 0) {
        auto timename = "time:" + name;
        auto stats = util::calc_stats(timer.get_times());
        log_milestone_event(timename + "-total",  stats.total);
        log_milestone_event(timename + "-min",    stats.min);
        log_milestone_event(timename + "-mean",   stats.mean);
        log_milestone_event(timename + "-median", stats.median);
        log_milestone_event(timename + "-max",    stats.max);
      }
    }
    timed_object->clear_timing();
  };

  std::cout << "\nLoading roadmap..." << std::endl;
  float timing;
  util::time_function_call([&]() {
      vplanner->loadRoadmapFromFile(roadmap,
                                    check_vert_validity,
                                    check_edge_validity);
    }, timing);
  log_event("time:loadRoadmapFromFile", timing);

  if (remove_disconnected_vertices) {
    util::time_function_call([&vplanner]() {
        vplanner->clearDisconnectedVertices();
      }, timing);
    log_event("time:clearDisconnectedVertices", timing);
  }
  std::cout << "  done loading roadmap" << std::endl;
  milestone = -1;
  log_and_clear_timers(vplanner);
  log_and_clear_timers(checker);
  log_and_clear_timers(motion_validator);

  vplanner->setDefaultIkController(ik_maxit, ik_tol, ik_mu_i);

  util::FunctionTimer ik_timer, solve_timer;
  util::StopWatch milestone_watch;

  ob::State* current_state = si->cloneState(pdef->getStartState(0));
  milestone = 1;
  for (csv::CsvRow row; reader >> row; ++milestone) {
    log_milestone_event("start-milestone", milestone);
    milestone_watch.start();
    E::Vector3d goal_tip {
        std::stod(row["tip_x"]),
        std::stod(row["tip_y"]),
        std::stod(row["tip_z"])
    };

    planner->clearQuery();
    // My planner calls this in clearQuery(), but other planners do not
    // It's not a problem to call it more than once :)
    pdef->clearSolutionPaths();
    if (!common_start) {
      // set the start state to the current configuration
      pdef->clearStartStates();
      pdef->addStartState(current_state);
    }

    // perform inverse kinematics
    std::cout << "\nRunning IK for [" << goal_tip.transpose() << "]" << std::endl;
    auto ik_results = ik_timer.time(
        [&vplanner, &goal_tip, ik_tol, ik_N, &ikopt]
        () -> std::vector<Planner::IKResult>
        {
          auto ans = vplanner->roadmapIk(goal_tip, ik_tol, ik_N, ikopt);
          // convert std::optional<> to std::vector<>
          if (ans) { return {*ans}; }
          return {};
        });
    log_milestone_event("time:roadmapIk", ik_timer.get_times().back());
    my_assert(ik_results.size() > 0, "no IK results returned");
    std::cout << "\nFor tip [" << goal_tip.transpose() << "], returned "
              << ik_results.size() << " ik results" << std::endl;

    // log the IK results
    log_milestone_event("ik:goal", vec2str(goal_tip));
    log_milestone_event("ik:count", ik_results.size());
    for (auto &result : ik_results) {
      log_milestone_event("ik:neighbor", vec2str(result.neighbor));
      log_milestone_event("ik:solution", vec2str(result.controls));
      log_milestone_event("ik:tip-error", result.error);
    }

    // assert the IK results are actually valid
    for (auto &result : ik_results) {
      ob::ScopedState<> state(si);
      state = result.controls;
      my_assert(si->getStateValidityChecker()->isValid(state.get()),
                "IK result is not valid");
    }

    // set the goal to the found IK locations
    auto goals(std::make_shared<ob::GoalStates>(si));
    for (auto &result : ik_results) {
      auto goal = vec2newstate(result.controls);
      goals->addState(goal);
    }
    pdef->setGoal(goals);

    // run the planner
    auto solve_status = solve_timer.time(
        [&vplanner, timeout]() {
          return vplanner->solveWithRoadmap(timeout);
        });
    log_milestone_event("time:solveWithRoadmap", solve_timer.get_times().back());
    print_solve_status(solve_status);
    log_milestone_event("status:solveWithRoadmap", solve_status.asString());

    log_and_clear_timers(vplanner);
    log_and_clear_timers(checker);
    log_and_clear_timers(motion_validator);
    auto local_plan = motion_planning::get_solution(planner);

    milestone_watch.stop();

    if (solve_status != ob::PlannerStatus::EXACT_SOLUTION) {
      if (local_plan.empty()) {
        // no solution found: make plan to just stay put
        OMPL_WARN("Could not reach goal, no solution");
        log_milestone_event("solution:status", "empty");
        local_plan.emplace_back(state2vec(current_state));
      } else {
        OMPL_WARN("Could not reach goal, approximate solution");
        log_milestone_event("approx-milestone", milestone);
      }
    } else {
      log_milestone_event("solution:status", "exact");
    }

    my_assert(local_plan.size() > 0, "local plan is empty");

    // log the found solution
    {
      auto fk_shape  = problem.robot.forward_kinematics(local_plan.back());
      auto tip_error = (goal_tip - fk_shape.back()).norm();
      auto path_cost = motion_planning::solution_cost(planner).value();
      auto start_str = vec2str(local_plan.front());
      auto goal_str  = vec2str(local_plan.back());
      auto goaltip_str  = vec2str(goal_tip);
      auto tip_str   = vec2str(fk_shape.back());
      OMPL_DEBUG("Problem:  REQ. TIP    = [%s]", goaltip_str.c_str());
      OMPL_DEBUG("Solution: REACHED TIP = [%s]", tip_str.c_str());
      OMPL_DEBUG("Solution: TIP ERROR   =  %lf", tip_error);
      OMPL_DEBUG("Solution: START STATE = [%s]", start_str.c_str());
      OMPL_DEBUG("Solution: GOAL STATE  = [%s]", goal_str.c_str());
      OMPL_DEBUG("Solution: WAYPOINTS   =  %u",  local_plan.size());
      OMPL_DEBUG("Solution: PATH COST   =  %lf", path_cost);
      log_milestone_event("solution:tip-error",  tip_error);
      log_milestone_event("solution:waypoints",  local_plan.size());
      log_milestone_event("solution:cost",       path_cost);
    }

    // write out the local plan (if requested by the user)
    if (keep_intermediate) {
      char prefix[12];
      std::snprintf(prefix, 12, "%02d", milestone);
      auto fname = intermediate_dir + "/" + prefix + "-plan.csv";
      std::cout << "\nsaving plan #" << milestone << " to " << fname
                << std::endl;
      problem.save_plan(fname, local_plan);
    }


    if (!common_start) {
      my_assert(total_plan.size() == 0 ||
                  state_vecs_equal(total_plan.back(), local_plan.front()),
                "local plan does not begin at the end of our current plan");
      // set current_state to the end of the current plan
      current_state = vec2newstate(local_plan.back());
    }

    // concatenate the local plan to the end of the current plan
    problem.extend_plan(total_plan, local_plan);

    // TODO: append to the file instead of overwriting it over and over again
    // Let's skip writing this for now, as it is done after all milestones have
    // finished anyway
    //problem.save_plan(outfile, total_plan);

    // Note: we moved the stop for the milestone_watch earlier because saving
    // the plans to file were significantly adding to the "milestone" time.
    log_milestone_event("time:milestone", milestone_watch.get_times().back());

  }
  std::cout << "\nsaving full plan to " << outfile << std::endl;
  problem.save_plan(outfile, total_plan);

  // calculate timing and call statistics, and log them
  auto log_total_stats = [&log_event](const std::string &name, const auto &times) {
    auto stats = util::calc_stats(times);
    log_event(name + "-total",  stats.total);
    log_event(name + "-min",    stats.min);
    log_event(name + "-mean",   stats.mean);
    log_event(name + "-median", stats.median);
    log_event(name + "-max",    stats.max);
  };
  log_total_stats("time:ik", ik_timer.get_times());
  log_total_stats("time:solveWithRoadmap", solve_timer.get_times());
  log_total_stats("time:milestone", milestone_watch.get_times());

  return 0;
}
