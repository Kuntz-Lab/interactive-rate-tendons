#include "cliparser/CliParser.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "motion-planning/VoxelCachedLazyPRM.h"
#include "motion-planning/WSpaceGoal.h"
#include "motion-planning/ompl_planners.h"
#include "motion-planning/plan.h"
#include "tip-control/Controller.h"
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
  const std::string planner_name   = "RRTConnect";
  const std::string output         = "chained-plan.csv";
  const std::string log            = "chained-plan-log.csv";
  const std::string ompl_log_level = "DEBUG";
  const std::string fk_type        = "fast";
  const double tolerance           = 0.0005;
  const size_t ik_max_iters        = 100;
  const double ik_mu_init          = 1e3;
  const size_t ik_random_restarts  = 0;
  const double timeout             = 20; // seconds
} // end of namespace defaults

void populate_parser (CliParser &parser) {
  parser.set_program_description(
        "Plan using the any planner EXCEPT for the VoxelCachedLazyPRM on a\n"
      "  sequence of W-Space waypoints.  This will generate a plan between\n"
      "  each waypoint, getting as close as it can before moving to the next\n"
      "  one.  You can keep the intermediate plans by specifying an\n"
      "  intermediate plan output directory.  Either way, this will output a\n"
      "  single file containing the combined plan.\n"
      "\n"
      "  The goal by default is specified in OMPL as an\n"
      "  ompl::base::GoalRegion, which is a fancy class for a W-Space tip\n"
      "  point with an associated distance threshold.  Only planners that\n"
      "  can handle a REGION type of goal can be used with this app.\n"
      "  However, this can be changed by specifying the --goal-ik flag to\n"
      "  generate a single goal configuration using our IK method starting\n"
      "  from the current step's start configuration.  Using this flag will\n"
      "  allow you to use planners that require a goal state, like\n"
      "  bidirectional planners like RRTConnect.\n"
      "\n"
      "  By default, the current step starts from the ending configuration\n"
      "  of the previous step (i.e., the last configuration of the last\n"
      "  step's found plan).  This can be changed with --common-start to\n"
      "  have each W-Space waypoint to be planned from the same start\n"
      "  configuration, specifically the one specified in the problem toml.\n"
      "\n"
      "  Note: this app is NOT for the VoxelCachedLazyPRM planner.  The\n"
      "  roadmap_chained_plan app is for testing that planner, whereas this\n"
      "  app is for testing planners from OMPL.");

// the regular damped least squares IK based on the finite difference Jacobian 

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem toml description file.\n"
      "                The start configuration is used as the starting\n"
      "                point, but the goal configuration is ignored.\n"
      "                Instead the waypoints are used, and after the final\n"
      "                waypoint, it will return back to the start\n"
      "                configuration.");

  parser.add_positional("waypoints");
  parser.set_required("waypoints");
  parser.set_description("waypoints",
                      "CSV file containing tip positions to plan to.\n"
      "                Expected columns are:\n"
      "                - tip_x\n"
      "                - tip_y\n"
      "                - tip_z\n"
      "                Extra columns are ignored.\n");

  parser.add_argflag("-P", "--planner-name");
  parser.set_description("--planner-name", "Name of the panner to use."
                    "  Run 'query_planner --list-planners'\n"
      "                for available planners.\n"
      "                (default is " + defaults::planner_name + ")");

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

  parser.add_flag("--goal-ik");
  parser.set_description("--goal-ik",
                      "Instead of using a W-Space point as the goal, run\n"
      "                Inverse Kinematics (IK) starting from the current\n"
      "                step's start configuration.  The IK is using\n"
      "                Levenberg-Marquardt based on the finite difference\n"
      "                Jacobian.  The returned configuration from IK will be\n"
      "                used as a goal state for planning.  If this goal state\n"
      "                is invalid (e.g., in collision), we take small steps\n"
      "                backwards until we obtain a valid state.");

  parser.add_argflag("--tolerance");
  parser.set_description("--tolerance",
                      "Tip error stopping tolerance for motion planning.\n"
      "                When using --goal-ik, this tolerance is instead used\n"
      "                for IK.\n"
      "                (default is "
                        + std::to_string(defaults::tolerance) + " meters)");

  parser.add_argflag("--ik-max-iters");
  parser.set_description("--ik-max-iters",
                      "Maximum number of iterations for IK.\n"
      "                (only used if --goal-ik is specified).\n"
      "                (default is "
                        + std::to_string(defaults::ik_max_iters) + ")");

  parser.add_argflag("--ik-mu-init");
  parser.set_description("--ik-mu-init",
                      "initial damping factor for Levenberg-Marquardt (LM)\n"
      "                (also known as damped least squares (DLS).\n"
      "                Convergence may depend heavily on this.  A larger\n"
      "                value means smaller iteration step sizes, a smaller\n"
      "                value means larger iteration step sizes (roughly).\n"
      "                (only used if --goal-ik is specified).\n"
      "                (default is "
                        + std::to_string(defaults::ik_mu_init) + ")");

  parser.add_argflag("--ik-random-restarts");
  parser.set_description("--ik-random-restarts",
                      "If IK from current configuration doesn't work,\n"
      "                generate a random initial state and redo it.  This is\n"
      "                an initial state just for IK, not for planning -- the\n"
      "                purpose is just to find a good goal state.  Do that\n"
      "                up to this many number of times, using the best IK\n"
      "                answer if none reach tolerance.\n"
      "                (only used if --goal-ik is specified).\n"
      "                (default is "
                        + std::to_string(defaults::ik_random_restarts) + ")");

  parser.add_argflag("-t", "--timeout");
  parser.set_description("--timeout", "Timeout for planning in seconds\n"
      "                (default is " + std::to_string(defaults::timeout) + ")");

  parser.add_flag("-v", "--voxel");
  parser.set_description("--voxel", "Enable the use of voxels instead of the\n"
      "                Environment class.  This will ignore the [environment]\n"
      "                section of the toml file and will instead use the\n"
      "                [voxel_environment] section which expects a voxel\n"
      "                image file as the only obstacle.\n"
      "                It is assumed the voxel environment has already been\n"
      "                scaled and translated to be in the robot frame.\n"
      "                But, the rotation will be applied to the robot before\n"
      "                collision checking.");

  parser.add_flag("--backbone");
  parser.set_description("--backbone",
                      "Only applicable if --voxel is specified.\n"
      "                Only collision check against the backbone path of the\n"
      "                tendon robot instead of the full robot with radius\n"
      "                thickness.");

  parser.add_flag("--swept-volume");
  parser.set_description("--swept-volume",
                      "Only applicable if --voxel is specified.\n"
      "                Use the generated swept volume instead of the default\n"
      "                motion validator for the motion-planning edge\n"
      "                checker.  Also, can only be used with --backbone\n"
      "                since swept volume of full robot is not implemented.");

  parser.add_argflag("--fk");
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

  auto problem            =
      cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  auto waypoints          = parser["waypoints"];
  auto planner_name       = parser.get("--planner-name", defaults::planner_name);
  auto common_start       = parser.has("--common-start");
  auto keep_intermediate  = parser.has("--intermediate-dir");
  auto intermediate_dir   = parser.get("--intermediate-dir", std::string());
  auto outfile            = parser.get("--output", defaults::output);
  auto logfile            = parser.get("--log", defaults::log);
  auto ompl_log_level     = parser.get("--ompl-log-level", defaults::ompl_log_level);
  auto goal_ik            = parser.has("--goal-ik");
  auto tolerance          = parser.get("--tolerance", defaults::tolerance);
  auto ik_max_iters       = parser.get("--ik-max-iters", defaults::ik_max_iters);
  auto ik_mu_init         = parser.get("--ik-mu-init", defaults::ik_mu_init);
  auto ik_random_restarts =
      parser.get("--ik-random-restarts", defaults::ik_random_restarts);
  auto timeout            = parser.get("--timeout", defaults::timeout);
  auto fk_type            = parser.get("--fk", defaults::fk_type);
  bool voxel              = parser.has("--voxel");
  bool backbone           = parser.has("--backbone");
  bool swept_volume       = parser.has("--swept-volume");

  if (backbone && !voxel) {
    auto msg = "Error: --backbone can only be used with --voxel";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  if (swept_volume && !voxel) {
    auto msg = "Error: --swept-volume can only be used with --voxel";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

  if (swept_volume && !backbone) {
    auto msg = "Error: --swept-volume only supports --backbone mode";
    std::cerr << msg << std::endl;
    throw CliParser::ParseError(msg);
  }

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
  log_event("settings:waypoints",              waypoints);
  log_event("settings:--planner-name",         planner_name);
  log_event("settings:--common-start",         common_start);
  if (keep_intermediate) {
    log_event("settings:--intermediate-dir",   intermediate_dir);
  }
  log_event("settings:--output",               outfile);
  log_event("settings:--log",                  logfile);
  log_event("settings:--ompl-log-level",       ompl_log_level);
  log_event("settings:--goal-ik",              goal_ik);
  if (goal_ik) {
    log_event("settings:--ik-max-iters",       ik_max_iters);
    log_event("settings:--ik-mu-init",         ik_mu_init);
    log_event("settings:--ik-random-restarts", ik_random_restarts);
  }
  log_event("settings:--tolerance",            tolerance);
  log_event("settings:--timeout",              timeout);
  log_event("settings:--voxel",                voxel);
  log_event("settings:--backbone",             backbone);
  log_event("settings:--swept_volume",         swept_volume);
  log_event("settings:--fk",                   fk_type);

  auto available_options = motion_planning::planner_options(planner_name);
  motion_planning::Options options;
  if (available_options.find("thread_count") != available_options.end()) {
    options["thread_count"] = "16";
  }
  auto planner = problem.create_planner(planner_name, options);
  if (voxel) {
    problem.update_to_voxel_validators(planner, backbone, swept_volume);
  }
  std::shared_ptr<motion_planning::WSpaceGoal> wgoal;
  if (!goal_ik) {
    // using W-Space goal.  Here we initialize it, then we will update the
    // desired tip position later in the loop.
    wgoal = problem.update_to_wspace_goal(planner, {}, tolerance);
  }

  auto si    = planner->getSpaceInformation();
  auto space = si->getStateSpace();
  auto pdef  = planner->getProblemDefinition();
  auto checker = std::dynamic_pointer_cast<
      motion_planning::AbstractValidityChecker>(si->getStateValidityChecker());
  auto motion_validator = std::dynamic_pointer_cast<
      motion_planning::AbstractVoxelMotionValidator>(si->getMotionValidator());
  auto valid_sampler = si->allocValidStateSampler();
  valid_sampler->setNrAttempts(1000); // how many tries for rejection sampling
  my_assert(pdef->getStartStateCount() == 1, "# start states != 1");
  my_assert(bool(checker), "State checker is not an AbstractValidityChecker");

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

  ob::ScopedState<> sample_buffer(space);
  auto random_valid_statevec =
    [&valid_sampler, buf=sample_buffer.get(), &state2vec]() {
      valid_sampler->sample(buf);
      return state2vec(buf);
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

  tip_control::FKFunc fk_func = [&checker](const auto &state) {
    auto fk_result = checker->fk(state);
    return fk_result.first.p.back();
  };

  auto ik_func = [&problem, &fk_func, tolerance, ik_max_iters, ik_mu_init]
    (const auto &goal_tip, const auto &x0) {
      const double dtau_min                  = problem.min_tension_change;
      const double stop_threshold_JT_err_inf = dtau_min / 10.0; // gradient step
      const double stop_threshold_Dp         = dtau_min / 20.0; // delta p
      const double stop_threshold_err        = tolerance;       // f(p) - goal
      const double finite_difference_delta   = dtau_min / 4.0;
      const bool verbose = true;
      return tip_control::inverse_kinematics(
          fk_func,
          problem.robot,
          x0,
          goal_tip,
          ik_max_iters,
          ik_mu_init,
          stop_threshold_JT_err_inf,
          stop_threshold_Dp,
          stop_threshold_err,
          finite_difference_delta,
          verbose);
    };

  auto ik_fix_func = [&checker, &space, &si, &vec2str]
    (const auto &goal_tip, const auto &x0, const auto &ik_old)
       -> std::pair<bool, Controller::IKResult>
    {
      Controller::IKResult ik(ik_old); // copy
      auto old_state      = ik.state;
      auto fk_vals        = checker->fk(ik.state);
      auto &[fk_shape, home_shape] = fk_vals;
      ik.num_fk_calls++;

      // check if this state even needs to be fixed
      bool is_valid_shape = checker->is_valid_shape(fk_shape, home_shape);
      bool collides       = checker->collides(fk_shape);
      if (is_valid_shape && !collides) {
        OMPL_DEBUG("ik: first solution is good, no collisions!");
        return {false, ik};
      }
      OMPL_DEBUG("ik: is in collision, needs fixing");

      ob::ScopedState<> s(si), g(si), c(si); // start, goal, current
      s = x0;
      g = ik.state;
      auto Nseg = space->validSegmentCount(s.get(), g.get());
      for (int i = Nseg; i-->0;) { // count backwards from Nseg-1 to 0
        space->interpolate(s.get(), g.get(), double(i) / double(Nseg), c.get());
        space->copyToReals(ik.state, c.get());
        fk_vals = checker->fk(ik.state); // updates fk_shape and home_shape
        ik.num_fk_calls++;

        is_valid_shape = checker->is_valid_shape(fk_shape, home_shape);
        if (!is_valid_shape) { continue; } // no need to check collision

        collides       = checker->collides(fk_shape);
        if (collides)        { continue; }

        // if we get here, then we have our valid configuration
        break;
      }

      // if still invalid after the for loop, just use x0
      if (!is_valid_shape || collides) {
        ik.state = x0;
        fk_vals = checker->fk(ik.state);
        ik.num_fk_calls++;
      }

      // calculate the new error and return
      auto old_error = ik.error;
      ik.error = (fk_shape.p.back() - goal_tip).norm();

      auto a = vec2str(old_state);
      auto b = vec2str(ik.state);
      OMPL_DEBUG("ik: fixed state from [%s] to [%s] (error: %lf -> %lf)",
                 a.c_str(), b.c_str(), old_error, ik.error);
      return {true, ik};
    };

  int milestone = 1;
  auto log_milestone_event = [&log_writer, &milestone]
    (const auto &name, const auto &value) {
      log_writer << name << milestone << value;
      log_writer.new_row();
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

  util::FunctionTimer ik_restarts_timer, ik_timer, ik_fix_timer, solve_timer;
  util::StopWatch milestone_watch;

  ob::State* current_state = si->cloneState(pdef->getStartState(0));
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
    pdef->clearSolutionPaths();
    if (!common_start) {
      // set the start state to the current configuration
      pdef->clearStartStates();
      pdef->addStartState(current_state);
    }

    if (!goal_ik) { 
      wgoal->setDesiredTip(goal_tip);
    } else {
      //
      // run IK
      //

      std::cout << "\nRunning IK for [" << goal_tip.transpose() << "]" << std::endl;
      auto x0 = state2vec(current_state);
      std::vector<Controller::IKResult> ik_results;
      ik_results.reserve(ik_random_restarts + 1);

      // random restarts
      ik_restarts_timer.time([&]() {
        for (size_t i = 0; i <= ik_random_restarts; i++) {
          auto ik_result = ik_timer.time([&]() { return ik_func(goal_tip, x0); });
          log_milestone_event("time:ik", ik_timer.get_times().back());
          log_milestone_event("ik:goal", vec2str(goal_tip));

          log_milestone_event("ik:current-restart", i);
          log_milestone_event("ik:x0", vec2str(x0));
          log_milestone_event("ik:solution", vec2str(ik_result.state));
          log_milestone_event("ik:tip-error", ik_result.error);
          log_milestone_event("ik:iters", ik_result.iters);

          auto [was_fixed, fixed_ik] = ik_fix_timer.time(
              [&]() { return ik_fix_func(goal_tip, x0, ik_result); });
          ik_results.emplace_back(fixed_ik);

          // print ik fix results to console
          log_milestone_event("time:ik-fix", ik_fix_timer.get_times().back());
          log_milestone_event("ik:in-collision", was_fixed);
          if (was_fixed) {
            log_milestone_event("ik:fixed-solution", vec2str(fixed_ik.state));
            log_milestone_event("ik:fixed-tip-error", fixed_ik.error);
          }
          log_milestone_event("ik:fk-calls", fixed_ik.num_fk_calls);

          if (fixed_ik.error < tolerance) {
            OMPL_DEBUG("ik: Accepted fixed IK solution with error %lf",
                       fixed_ik.error);
            break;
          } else {
            OMPL_DEBUG("ik: Rejected fixed IK solution with error %lf%s",
                       fixed_ik.error,
                       (i < ik_random_restarts ? ", doing random restart" : ""));
            x0 = random_valid_statevec();
          }
        }
      });
      log_milestone_event("time:ik-with-restarts",
                          ik_restarts_timer.get_times().back());

      my_assert(!ik_results.empty(), "no IK results!  This can't be!");
      log_milestone_event("ik:restart-count", ik_results.size() - 1);
      for (auto &ikval : ik_results) {
        std::string tmpstr = vec2str(ikval.state);
        OMPL_DEBUG("  ik-result: state: [%s], error: %lf", tmpstr.c_str(), ikval.error);
      }

      auto best_ik_it = std::min_element(ik_results.begin(), ik_results.end(),
          [](const auto &a, const auto &b) { return a.error < b.error; });

      my_assert(best_ik_it != ik_results.end(), "could not find a best IK!");
      auto best_ik = *best_ik_it;
      log_milestone_event("ik:final-solution", vec2str(best_ik.state));
      log_milestone_event("ik:final-error", best_ik.error);

      {
        auto tmpstr = vec2str(best_ik.state);
        OMPL_DEBUG("ik: best solution [%s] with error %lf",
                   tmpstr.c_str(), best_ik.error);
      }

      // use this IK configuration as the goal state
      ob::ScopedState<> goal_state(si);
      space->copyFromReals(goal_state.get(), best_ik.state);
      pdef->setGoalState(goal_state);
    }

    // run the planner
    auto solve_status = solve_timer.time(
        [&planner, timeout]() { return planner->solve(timeout); });
    log_milestone_event("time:solve", solve_timer.get_times().back());
    std::cout << "Solve Status: ";
    print_solve_status(solve_status);
    log_milestone_event("status:solve", solve_status.asString());

    log_and_clear_timers(checker);
    if (motion_validator) {
      log_and_clear_timers(motion_validator);
    }

    auto local_plan = motion_planning::get_solution(planner);

    milestone_watch.stop();

    if (solve_status == ob::PlannerStatus::EXACT_SOLUTION) {
      log_milestone_event("solution:status", "exact");
    } else {
      auto x0 = state2vec(current_state);
      if (local_plan.empty()) {
        // no solution found: make plan to just stay put
        OMPL_WARN("Could not reach goal, no solution");
        log_milestone_event("solution:status", "empty");
        local_plan.emplace_back(x0);
                  //state_vecs_equal(total_plan.back(), local_plan.front()),
      } else if (planner_name == "RRTConnect"
              && !state_vecs_equal(local_plan.front(), x0)) {
        // plan from RRTConnect doesn't start from the start state
        OMPL_WARN("Could not reach goal, approximate solution");
        OMPL_WARN("Returned approximate solution does not start from the start state");
        OMPL_WARN("  An RRTConnect bug sometimes returns approximate solutions"
                  " from the goal tree.");
        log_milestone_event("approx-milestone", milestone);
        log_milestone_event("solution:status", "RRTConnect-bug");
        local_plan = {x0}; // replace with an empty plan
      } else {
        OMPL_WARN("Could not reach goal, approximate solution");
        log_milestone_event("approx-milestone", milestone);
        log_milestone_event("solution:status", "approximate");
      }
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
  log_total_stats("time:ik-fix", ik_fix_timer.get_times());
  log_total_stats("time:ik-restarts", ik_restarts_timer.get_times());
  log_total_stats("time:solve", solve_timer.get_times());
  log_total_stats("time:milestone", milestone_watch.get_times());

  return 0;
}
