#include <cliparser/CliParser.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <tendon/TendonRobot.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <random>

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Estimate length limits given a robot specification with tension\n"
      "  limits.  The length limits in the robot specification will be\n"
      "  ignored and instead the tension limits will be used.  An estimate on\n"
      "  the length limits will be printed to the console and (optionally)\n"
      "  to an output robot description file");

  parser.add_positional("robot_toml");
  parser.set_required("robot_toml");
  parser.set_description("robot_toml",
                      "A toml file containing a robot configuration.  The\n"
      "                robot description needs to be complete.  Even though\n"
      "                the length limits are not used, they still must be\n"
      "                present.");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output",
                      "Output a robot description complete with the estimated\n"
      "                length limits to this given file.  By default, no\n"
      "                output file is created.  Instead, the estimated length\n"
      "                limits are output to the console.");

  parser.add_flag("-r", "--random-sampling");
  parser.set_description("--random-sampling",
                      "Augment the length limit search with random sampling\n");
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

  // estimate length limits
  //
  // During each step, capture the min and max lengths
  //
  // 1. Every combination of min and max tension for each tendon.  This has 2^N
  //    number of combinations for N tendons, which should be pretty small.
  // 2. Randomly sample the tension space until 1000 points are sampled without
  //    updating any of the length limits.

  auto home_shape = robot.home_shape();
  auto N = robot.tendons.size();
  std::vector<double> min_dlength (N, 0.0);
  std::vector<double> max_dlength (N, 0.0);

  // updates min_dlength and max_dlength, return true if an update happened
  auto update_dlength_limits =
    [&robot, &min_dlength, &max_dlength, &home_shape](const auto &tau) {
      auto shape = robot.shape(tau, 0.0, 0.0);
      auto dl = robot.calc_dl(home_shape.L_i, shape.L_i);
      bool update_happened = false;
      for (size_t i = 0; i < dl.size(); i++) {
        if (dl[i] < min_dlength[i]) {
          min_dlength[i] = dl[i];
          update_happened = true;
        } else if (max_dlength[i] < dl[i]) {
          max_dlength[i] = dl[i];
          update_happened = true;
        }
      }
      return update_happened;
    };

  // step 1: do every combination of min and max tension for each tendon
  size_t n_perm = 1 << N;
  for (size_t perm = 0; perm < n_perm; perm++) {
    // generate tension for the given permutation
    std::vector<double> tau(N, 0.0);
    for (size_t i = 0; i < N; i++) {
      if (perm & (1 << i)) {
        tau[i] = robot.tendons[i].max_tension;
      }
    }
    update_dlength_limits(tau);
  }

  // step 2: randomly sample until K points without a length update
  int K = 1000;
  std::minstd_rand generator;
  std::vector<std::uniform_real_distribution<double>> dists;
  for (auto &tendon : robot.tendons) {
    dists.emplace_back(0.0, tendon.max_tension);
  }

  if (parser.has("--random-sampling")) {
    for (int iters_since_update = 0; iters_since_update < K;
         iters_since_update++)
    {
      // randomly sample
      std::vector<double> tau(N, 0.0);
      for (size_t i = 0; i < N; i++) {
        tau[i] = dists[i](generator);
      }

      if (update_dlength_limits(tau)) {
        iters_since_update = 0;
      }
    }
  }

  for (size_t i = 0; i < N; i++) {
    // set the robot's length limits
    auto &tendon = robot.tendons[i];
    tendon.min_length = min_dlength[i];
    tendon.max_length = max_dlength[i];
    std::cout
      << "tendon[" << i << "]\n"
      << "  max tension:               " << tendon.max_tension << "\n"
      << "  estimated length limits:  [" << tendon.min_length << ", "
                                         << tendon.max_length << "]\n";
  }

  if (parser.has("--output")) {
    std::cout << "\n"
                 "Writing updated robot description to " << parser["--output"]
              << std::endl;
    cpptoml::to_file(robot, parser["--output"]);
  }

  return 0;
}
