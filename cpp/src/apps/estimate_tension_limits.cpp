#include <cliparser/CliParser.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <tendon/TendonRobot.h>

#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Estimate tension limits given a robot specification with length\n"
      "  limits.  The tension limits in the robot specification will be\n"
      "  ignored and instead the length limits will be used.  An estimate on\n"
      "  the tension limits will be printed to the console and (optionally)\n"
      "  to an output robot description file");

  parser.add_positional("robot_toml");
  parser.set_required("robot_toml");
  parser.set_description("robot_toml",
                      "A toml file containing a robot configuration.  The\n"
      "                robot description needs to be complete.  Even though\n"
      "                the tension limits are not used, they still must be\n"
      "                present.");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                      "Output a robot description complete with the estimated\n"
      "                tension limits to this given file.  By default, no\n"
      "                output file is created.  Instead, the estimated tension\n"
      "                limits are output to the console.");
}

void parse_args(CliParser &parser, int argCount, char* argList[]) {
  parser.parse(argCount, argList);
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);

  try {
    parse_args(parser, argCount, argList);
  } catch (CliParser::ParseError &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }

  auto robot = cpptoml::from_file<tendon::TendonRobot>(parser["robot_toml"]);

  // estimate tension limits
  //
  // 1. increase each tendon limits one at a time with all others zero until a
  //    length limit is exceeded.  Increase by a multiple of two at a time.
  // 2. once we exceed at least one length threshold, do a bisect search to find
  //    a much more precise boundary.

  auto home_shape = robot.home_shape();
  auto is_valid = [&home_shape, &robot](const auto &tau) {
    auto shape = robot.shape(tau, 0.0, 0.0);
    return robot.is_within_length_limits(home_shape.L_i, shape.L_i);
  };

  std::vector<double> tau(robot.tendons.size(), 0.0);
  double dtau_init = 0.001;
  for (size_t i = 0; i < robot.tendons.size(); i++) {
    // step 1: find upper and lower bounds for this one tendon alone
    tau[i] = dtau_init;
    while (is_valid(tau)) {
      tau[i] *= 2;
    }

    auto upper_limit = tau[i];
    auto lower_limit = tau[i] / 2.0;

    tau[i] = lower_limit;
    if (!is_valid(tau)) {
      std::cerr << "Error: Could not determine a lower tension limit\n";
      return 2;
    }

    tau[i] = upper_limit;
    if (is_valid(tau)) {
      std::cerr << "Error: upper-bound is not really an upper bound\n";
      return 3;
    }

    // step 2: bisect between upper and lower limit
    while (upper_limit - lower_limit > dtau_init) {
      auto mid = (upper_limit + lower_limit) / 2.0;
      tau[i] = mid;
      if (is_valid(tau)) {
        lower_limit = mid;
      } else {
        upper_limit = mid;
      }
    }

    // set the robot's tension limit
    auto &tendon = robot.tendons[i];
    tendon.max_tension = upper_limit;
    std::cout
      << "tendon[" << i << "]\n"
      << "  length limits:          [" << tendon.min_length << ", "
                                       << tendon.max_length << "]\n"
      << "  estimated max tension:  "  << upper_limit << std::endl;

    tau[i] = 0; // reset this tendon's tension
  }

  if (parser.has("--output")) {
    std::cout << "\n"
                 "Writing updated robot description to " << parser["--output"]
              << std::endl;
    cpptoml::to_file(robot, parser["--output"]);
  }

  return 0;
}
