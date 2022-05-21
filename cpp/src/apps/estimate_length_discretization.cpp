#include <cliparser/CliParser.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <motion-planning/Problem.h>
#include <tendon/TendonRobot.h>

#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <vector>
#include <random>

namespace E = Eigen;

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
        "Estimate length discretization for four different types of criteria.\n"
      "  Multiple values will be output to the terminal to report on found\n"
      "  length discretizations.  The criterias are\n"
      "\n"
      "  - max tip error\n"
      "  - tip error standard deviation\n"
      "\n"
      "  The discretization given in the robot toml file will be the baseline\n"
      "  for comparisons and this program will attempt to decrease from\n"
      "  there."
      );

  parser.add_positional("robot_toml");
  parser.set_required("robot_toml");
  parser.set_description("robot_toml",
                      "A toml file containing a robot configuration.");

  parser.add_positional("samples_csv");
  parser.set_required("samples_csv");
  parser.set_description("samples_csv",
                      "A csv file containing samples.  Must have columns for\n"
      "                the robot controls, 'tau_#' (one for each tendon),\n"
      "                'theta' (if rotation is enabled), and 's_start' (if\n"
      "                retraction is enabled).");

  parser.add_argflag("--tip-error-max");
  parser.set_description("--tip-error-max",
                      "Set the threshold for maximum tip error.  Default is\n"
      "                dL from the robot toml file divided by two.");

  parser.add_argflag("--tip-error-stdev");
  parser.set_description("--tip-error-stdev",
                      "Set the threshold for the maximum standard deviation\n"
      "                error allowed for the tip position.  Default is dL\n"
      "                from the robot toml file divided by six.");

  parser.add_argflag("--step-percentage");
  parser.set_description("--step-percentage",
                      "How much to step each iteration.\n"
      "                (default is 1.0)");
}

std::vector<std::vector<E::Vector3d>> calc_shapes(
    const tendon::TendonRobot &robot, const motion_planning::Problem::PlanType &configs)
{
  std::vector<std::vector<E::Vector3d>> shapes(configs.size());
  #pragma omp parallel for
  for (size_t i = 0; i < configs.size(); ++i) {
    shapes[i] = robot.forward_kinematics(configs[i]);
  }
  return shapes;
}

std::vector<double> calc_tip_errors(
    const std::vector<std::vector<E::Vector3d>> &baseline,
    const std::vector<std::vector<E::Vector3d>> &current)
{
  if (baseline.size() != current.size()) {
    throw std::runtime_error("mismatched vector sizes in calc_errors()");
  }

  std::vector<double> errs(baseline.size());
  for (size_t i = 0; i < baseline.size(); ++i) {
    errs[i] = (current[i].back() - baseline[i].back()).norm();
  }

  return errs;
}

struct MyStats {
  double max;
  double stdev;
};

MyStats calc_my_stats(const std::vector<double> &errors) {
  if (errors.size() == 0) { return {}; }
  double max = errors[0];
  double total = 0.0;
  double sq_total = 0.0;
  for (auto err : errors) {
    max = std::max(max, err);
    total += err;
    sq_total += err*err;
  }
  double variance = (total*total - sq_total) / double(errors.size() - 1);
  double stdev = std::sqrt(variance);
  return {max, stdev};
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);

  try {
    parser.parse(argCount, argList);
  } catch (CliParser::ParseError &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }

  auto robot   = cpptoml::from_file<tendon::TendonRobot>(parser["robot_toml"]);
  auto samples = motion_planning::Problem::load_plan(parser["samples_csv"]);
  auto dL      = robot.specs.dL;

  auto thresh_tip_error_max   = parser.get("--tip-error-max",   dL / 2);
  auto thresh_tip_error_stdev = parser.get("--tip-error-stdev", dL / 6);
  auto step_percentage        = parser.get("--step-percentage", 1.0);

  double min_dL_change = dL * step_percentage / 100.0;

  // estimate length discretization
  //
  // 1. calculate and store shape with baseline dL
  //
  // Do the following for dL value under test for the
  // particular metric under test, simply stepping up each time
  //
  // 2. Store previous iteration's stats
  // 3. For each sample, calculate the error of each point against baseline
  // 4. If one of the threadholds is exceeded, report which one and the
  //    previous iteration's stats
  // 5. If all thresholds have been exceeded, then stop

  // calculate baseline shapes
  std::cout << "calculating baseline (dL = " << robot.specs.dL << ")\n";
  auto baseline = calc_shapes(robot, samples);

  bool past_tip_error_max         = false;
  bool past_tip_error_stdev       = false;

  auto report = [](double dL, double max, double stdev) {
    std::cout << "  dL:               " << dL    << "\n"
              << "  max tip error:    " << max   << "\n"
              << "  tip error stdev:  " << stdev << "\n";
  };

  auto prev_dL = dL;
  MyStats prev_stats {0.0, 0.0};
  while (!past_tip_error_max || !past_tip_error_stdev) {
    robot.specs.dL += min_dL_change;
    std::cout << "calculating shapes for dL = " << robot.specs.dL << "\n";
    auto current = calc_shapes(robot, samples);
    auto errors = calc_tip_errors(baseline, current);
    auto above_count = 0;
    for (auto &err : errors) {
      if (err > thresh_tip_error_max) {
        above_count++;
      }
    }
    auto stats = calc_my_stats(errors);
    if (!past_tip_error_max && stats.max > thresh_tip_error_max) {
      past_tip_error_max = true;
      std::cout << "\n"
                << "Best values for max tip error:\n";
      report(prev_dL, prev_stats.max, prev_stats.stdev);
      std::cout << "Next value had the following:\n";
      report(robot.specs.dL, stats.max, stats.stdev);
      std::cout << above_count << " of " << errors.size()
                << " configurations exceeded the threshold";
      std::cout << "\n";
    }
    if (!past_tip_error_stdev && stats.stdev > thresh_tip_error_stdev) {
      past_tip_error_stdev = true;
      std::cout << "\n"
                << "Best values for tip error stdev:\n";
      report(prev_dL, prev_stats.max, prev_stats.stdev);
      std::cout << "Next value had the following:\n";
      report(robot.specs.dL, stats.max, stats.stdev);
      std::cout << "\n";
    }
    prev_dL = robot.specs.dL;
    prev_stats = stats;
  }

  return 0;
}
