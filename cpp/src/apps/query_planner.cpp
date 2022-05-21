#include "cliparser/CliParser.h"

#include "motion-planning/ompl_planners.h"

#include <ompl/base/GoalTypes.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <iostream>
#include <iomanip>

#include <cstdio>

namespace ob = ompl::base;
using motion_planning::available_planners;
using motion_planning::make_planner;

namespace {

CliParser parse_args(int argCount, char* argList[]) {
  CliParser parser;

  parser.set_program_description(
      "Query information about a particular planner.");

  parser.add_positional("planner");
  parser.set_required("planner"); // -L makes this not required
  parser.set_description("planner",
                         "One or more planner to print information about.");

  parser.add_flag("-L", "--list-planners");
  parser.set_description("--list-planners", "List available planners and exit");

  try {
    parser.parse_with_exceptions(argCount, argList);
  } catch (CliParser::HelpRequest&) {
    std::cout << parser.usage();
    std::cout.flush();
    std::exit(0);
  } catch (CliParser::MissingRequiredError &ex) {
    if (!parser.has("--list-planners")) {
      std::cerr << "ParseError: " << ex.what() << "\n";
      std::cerr.flush();
      std::exit(1);
    }
  }

  //if (!parser.has("planner") && !parser.has("--list-planners")) {
  //  std::cerr << "ParseError: Missing required positional argument 'planner'\n";
  //  exit(1);
  //}

  return parser;
}

ob::SpaceInformationPtr make_space_information() {
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  return std::make_shared<ob::SpaceInformation>(space);
}

std::string to_string(ob::GoalType gt) {
  std::string retval;
  auto add = [&retval](std::string val) {
    if (retval.empty()) {
      retval = val;
    } else {
      retval += " | " + val;
    }
  };

  if (gt & ob::GOAL_ANY)               { add("ANY"); }
  if (gt & ob::GOAL_REGION)            { add("REGION"); }
  if (gt & ob::GOAL_SAMPLEABLE_REGION) { add("SAMPLEABLE_REGION"); }
  if (gt & ob::GOAL_STATE)             { add("STATE"); }
  if (gt & ob::GOAL_STATES)            { add("STATES"); }
  if (gt & ob::GOAL_LAZY_SAMPLES)      { add("LAZY_SAMPLES"); }

  return retval;
}

} // end of unnamed namespace


int main(int argCount, char* argList[]) {
  auto args = parse_args(argCount, argList);

  if (args.has("--list-planners")) {
    for (auto name : available_planners()) {
      std::cout << name << "\n";
    }
    return 0;
  }

  auto si = make_space_information();
  auto remaining = args.remaining();
  std::vector<std::string> planners;
  planners.emplace_back(args["planner"]);
  planners.insert(planners.end(), remaining.begin(), remaining.end());

  std::cout << "\n-------------------------------\n\n";
  for (auto &name : planners) {
    auto planner = make_planner(si, name);
    auto specs = planner->getSpecs(); // PlannerSpecs
    auto params = planner->params().getParams(); // map<str, GenericParam>

    // print information about the planner
    std::cout << std::boolalpha <<
        "Class:                                        " << name << "\n"
        "Name:                                         "
          << planner->getName() << "\n"
        "Specs:\n"
        "  Goal Type:                                  "
            << to_string(specs.recognizedGoal) << "\n"
        "  Multithreaded:                              "
            << specs.multithreaded << "\n"
        "  Approximate:                                "
            << specs.approximateSolutions << "\n"
        "  Optimizes:                                  "
            << specs.optimizingPaths << "\n"
        "  Directed:                                   "
            << specs.directed << "\n"
        "  Can Prove No Solution:                      "
            << specs.provingSolutionNonExistence << "\n"
        "Parameters:\n";

    for (auto &kv : params) {
      std::printf("  %-43s %s (default: %s)\n",
                  (kv.first + ':').c_str(),
                  kv.second->getRangeSuggestion().c_str(),
                  kv.second->getValue().c_str());
    }

    std::cout << "\nplanner->printProperties():\n";
    planner->printProperties(std::cout);

    std::cout << "\nplanner->printSettings():\n";
    planner->printSettings(std::cout);

    std::cout << "\n-------------------------------\n\n";
  }

  return 0;
}
