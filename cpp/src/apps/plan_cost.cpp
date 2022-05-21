#include "cliparser/CliParser.h"
#include "motion-planning/Problem.h"
#include "cpptoml/toml_conversions.h"

namespace {

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Reads one or more plans from CSV files and calculates the plan cost\n"
      "\n"
      "  Example:\n"
      "\n"
      "    $ plan_cost problem.toml solution.csv\n"
      "\n"
      "  may have output\n"
      "\n"
      "    solution.csv plan cost: 31.567893");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "problem description toml file");

  parser.add_positional("plan-csv");
  parser.set_required("plan-csv");
  parser.set_description("plan-csv",
                      "One or more CSV files containing plans.  Each CSV file\n"
      "                is expected to have the following columns:\n"
      "                  - tau_i   (for i in [1 : # tendons])\n"
      "                  - theta   (if enable_rotation)\n"
      "                  - s_start (if enable_retraction)\n"
      "                Will output the path cost, one line per csv file.");
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  std::vector<std::string> plan_csvs;
  plan_csvs.emplace_back(parser["plan-csv"]);
  plan_csvs.insert(plan_csvs.end(), parser.remaining().begin(),
                                    parser.remaining().end());

  for (auto &csvfile : plan_csvs) {
    auto plan = problem.load_plan(csvfile);
    std::cout << csvfile << " plan cost: " << problem.plan_cost(plan)
              << std::endl;
  }

  return 0;
}

