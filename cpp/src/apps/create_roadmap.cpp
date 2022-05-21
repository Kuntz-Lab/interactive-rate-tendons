#include <cliparser/CliParser.h>
#include <cpptoml/toml_conversions.h>
#include <motion-planning/Problem.h>
#include <motion-planning/VoxelCachedLazyPRM.h>
#include <motion-planning/VoxelBackboneMotionValidatorAndLogger.h>
#include <util/ompl_logging.h>
#include <util/time_function_call.h>

#include <iostream>
#include <string>
#include <set>

using Planner = motion_planning::VoxelCachedLazyPRM;

namespace {

namespace defaults {
  const int N = 30'000;
  const std::string planner_name = "VoxelCachedLazyPRM";
  const std::string output = "roadmap.msgpack";
  const std::string ompl_log_level = "DEBUG";
} // end of namespace defaults

const std::set<std::string> planner_name_choices = {
  "VoxelCachedLazyPRM",
};

void populate_parser(CliParser &parser) {
  parser.set_program_description("Generate samples and build a roadmap");

  parser.add_positional("problem_toml");
  //parser.set_required("problem_toml");
  parser.set_description("problem_toml", "Toml file describing the problem.\n"
      "                Required if not using --list-planners.");

  parser.add_argflag("-N", "--number");
  parser.set_description("--number", "number of samples in roadmap.\n"
      "                (default is " + std::to_string(defaults::N) + ")");

  parser.add_argflag("-P", "--planner-name");
  parser.set_description("--planner-name", "Planner to use.\n"
      "                (default is " + defaults::planner_name + ")");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output", "Output JSON filename for roadmap.\n"
      "                Supports toml, toml.gz, json, bson, cbor, msgpack,\n"
      "                ubjson, dat, and dat.gz.\n"
      "                (default is " + defaults::output + ")");

  parser.add_flag("--check-vertex-collision");
  parser.set_description("--check-vertex-collision",
                      "Make roadmap such that vertices avoid collision with\n"
      "                the voxel environment.  If sampling new vertices in\n"
      "                this call, sampling will be restricted to\n"
      "                collision-free sampling using rejection sampling.  By\n"
      "                default, voxel environment collisions are ignored.");

  parser.add_flag("--check-edge-collision");
  parser.set_description("--check-edge-collision",
                      "Make roadmap such that edges avoid collision with the\n"
      "                voxel environment.  By default, voxel environment\n"
      "                collisions are ignored.");

  parser.add_flag("--check-collision");
  parser.set_description("--check-collision",
                      "Enable --check-vertex-collision and --check-edge-collision");

  parser.add_flag("--remove-disconnected");
  parser.set_description("--remove-disconnected",
                      "At the very end, after generating the roadmap, doing\n"
      "                voxelization (if requested), and doing collision\n"
      "                checking (if requested), ensure there is only one\n"
      "                connected component by keeping only the largest one.");

  parser.add_flag("-L", "--list-planners");
  parser.set_description("--list-planners", "List available planners and exit");

  parser.add_flag("--voxelize-vertices");
  parser.set_description("--voxelize-vertices",
                      "Create voxel cache for roadmap vertices.\n"
      "                If this call is sampling new vertices, this will\n"
      "                cause all new samples to be valid shapes, meaning not\n"
      "                exceeding length limits and not self-colliding.");

  parser.add_flag("--voxelize-edges");
  parser.set_description("--voxelize-edges",
                      "Create voxel cache for roadmap edges.\n");

  parser.add_flag("--voxelize");
  parser.set_description("--voxelize",
                      "Enable --voxelize-vertices and --voxelize-edges");

  parser.add_argflag("--ompl-log-level");
  parser.set_description("--ompl-log-level",
                      "Set the log level used in OMPL.  Choices (in order of\n"
      "                most to least verbose) are 'DEV2', 'DEV1', 'DEBUG',\n"
      "                'INFO', 'WARN', 'ERROR', 'NONE'.\n"
      "                (default is '" + defaults::ompl_log_level + "')");

  parser.add_argflag("--from-roadmap");
  parser.set_description("--from-roadmap",
                      "Start with the given JSON roadmap before creating new\n"
      "                states and precomputing voxelization and/or collision.\n"
      "                Supports toml, toml.gz, json, bson, cbor, msgpack,\n"
      "                and ubjson.\n"
      "\n"
      "                Loading of the roadmap is affected by\n"
      "                --check-*collision flags.  If they are enabled, then\n"
      "                those checks will be performed at the time of\n"
      "                loading, so as to not generate a larger graph only to\n"
      "                prune it later.");

  parser.add_argflag("--num-edges");
  parser.set_description("--num-edges",
                      "Set the number of edges to connect nearest neighbors\n"
      "                in the roadmap.  The default behavior is to\n"
      "                calculate this based on the PRMstar algorithm to\n"
      "                prove connectivity guarantees.  This is calculated\n"
      "                based on the number of requested samples and the\n"
      "                dimensionality of the configuration space.");

  parser.add_flag("--add-missing-edges");
  parser.set_description("--add-missing-edges",
                      "Attempt to reconnect vertices to their nearest\n"
      "                neighbors.  This option only makes sense if using the\n"
      "                --from-roadmap option, and the step will be performed\n"
      "                after loading the given roadmap - trying to connect\n"
      "                vertices based on the value of --num-edges (or the\n"
      "                PRMstar strategy if not specified).\n"
      "\n"
      "                By default, the edges of the roadmap that is loaded\n"
      "                in is not modified at all, except if --number is\n"
      "                larger than the loaded vertices, in which case only\n"
      "                the newly added vertices become connected.  This\n"
      "                option can be useful if you first make a graph with\n"
      "                fewer or zero edges, then you want to increase the\n"
      "                density of edges later.  An example of this is to\n"
      "                create a roadmap with zero edges, so you can voxelize\n"
      "                just the vertices.  Then you can call this app again\n"
      "                to add missing edges without voxelization, so that\n"
      "                your resultant roadmap has a voxel cache on the\n"
      "                vertices but on the edges.");

  parser.add_argflag("--log-dyn-edge-voxelize-min-distance");
  parser.set_description("--log-dyn-edge-voxelize-min-distance",
                      "When voxelizing edges using our dynamic swept volume\n"
      "                discretizer, log the minimum of each subspace to the\n"
      "                given csv file.  This will have one row with the\n"
      "                following columns:\n"
      "                - dtau\n"
      "                - drot\n"
      "                - dret\n"
      "                each one respectively for each subspace.  If that\n"
      "                subspace is not present, its value will be inf.\n"
      "\n"
      "                Note: this will potentially cause each edge to be\n"
      "                voxelized up to four times to get these metrics, so\n"
      "                it can end up being much slower than without it.");
}

void parse_args(CliParser &parser, int arg_count, char *arg_list[]) {
  parser.parse(arg_count, arg_list);

  if (!parser.has("problem_toml") && !parser.has("--list-planners")) {
    throw CliParser::ParseError("Must specify a problem_toml file");
  }

  if (parser.has("--planner-name") &&
      0 == planner_name_choices.count(parser["--planner-name"]))
  {
    throw CliParser::ParseError("Unsupported planner given: "
                                + parser["--planner-name"]
                                + ", use --list-planners to see choices");
  }
}

} // end of unnamed namespace

int main(int arg_count, char* arg_list[]) {
  CliParser parser;
  populate_parser(parser);

  try {
    parse_args(parser, arg_count, arg_list);
  } catch (CliParser::ParseError &ex) {
    std::cerr << ex.what() << std::endl;
    return 1;
  }

  if (parser.has("--list-planners")) {
    for (auto &name : planner_name_choices) {
      std::cout << name << std::endl;
    }
    return 0;
  }

  const auto problem_toml     = parser["problem_toml"];
  const auto N                = parser.get("-N", defaults::N);
  const auto planner_name     = parser.get("--planner-name", defaults::planner_name);
  const auto output_fname     = parser.get("--output", defaults::output);
  const bool use_backbone     = true;
  const bool use_swept_volume = true;
  const bool remove_disconnected = parser.has("--remove-disconnected");
  const bool load_roadmap     = parser.has("--from-roadmap");
  const auto roadmap_file     = parser.get("--from-roadmap", std::string{});
  const auto ompl_log_level   = parser.get("--ompl-log-level", defaults::ompl_log_level);
  const bool prmstar          = !parser.has("--num-edges");
  const auto num_edges        = parser.get("--num-edges", 0); // default: calculate
  const bool add_missing_edges = parser.has("--add-missing-edges");
  const bool log_min_dists    = parser.has("--log-dyn-edge-voxelize-min-distance");
  const auto min_dists_log    = parser.get("--log-dyn-edge-voxelize-min-distance", std::string{});
  const bool voxelize_vertices =
      parser.has("--voxelize-vertices")      || parser.has("--voxelize");
  const bool voxelize_edges   =
      parser.has("--voxelize-edges")         || parser.has("--voxelize");
  const bool check_vertex_collision =
      parser.has("--check-vertex-collision") || parser.has("--check-collision");
  const bool check_edge_collision =
      parser.has("--check-edge-collision")   || parser.has("--check-collision");

  util::set_ompl_log_level(ompl_log_level);

  if (add_missing_edges && !load_roadmap) {
    std::cerr
      << "Warning: --add-missing-edges is only valid with --from-roadmap"
      << std::endl;
  }

  auto problem = cpptoml::from_file<motion_planning::Problem>(problem_toml);
  std::cout << "problem.sample_like_sphere:   " << problem.sample_like_sphere
            << std::endl;
  auto generic_planner = problem.create_planner(planner_name);
  auto planner = std::dynamic_pointer_cast<
      Planner>(generic_planner);
  if (!planner) {
    std::cerr << "Error: planner is not a VoxelCachedLazyPRM" << std::endl;
    return 2;
  }
  problem.update_to_voxel_validators(planner, use_backbone, use_swept_volume);
  if (log_min_dists && use_backbone && use_swept_volume) {
    auto voxels = problem.venv.get_obstacles();
    auto si = planner->getSpaceInformation();
    si->setMotionValidator(
        std::make_shared<
          motion_planning::VoxelBackboneMotionValidatorAndLogger>(
            si, problem.robot, problem.venv, *voxels, min_dists_log));
  }

  if (prmstar) {
    planner->setStarConnectionStrategy();
  } else {
    planner->setup();
    planner->setMaxNearestNeighbors(num_edges);
  }

  float timing;

  if (load_roadmap) {
    util::time_function_call([&]() {
          planner->loadRoadmapFromFile(
              roadmap_file, check_vertex_collision, check_edge_collision);
        }, timing);
    std::cout << "  Roadmap loading:                    "
              << timing << " seconds" << std::endl;

    // Note: we redo collision-checking and voxelization before adding missing edges
    // we will do it again after.  But it's easier to only add the needed edges
    // than to add all edges, then remove the ones that are attached to
    // vertices we would remove anyway

    if (check_vertex_collision) { // collision checking does voxelization
      util::time_function_call(
          [&]() { planner->precomputeVertexValidity(); }, timing);
      std::cout << "  Collision checking (vertices):       "
                << timing << " seconds" << std::endl;
    } else if (voxelize_vertices) {
      util::time_function_call(
          [&]() { planner->precomputeVertexVoxelCache(); }, timing);
      std::cout << "  Voxelization (vertices):             "
                << timing << " seconds" << std::endl;
    }

    if (add_missing_edges) {
      util::time_function_call([&]() { planner->addMissingEdges(); }, timing);
      std::cout << "  Add missing edges:                   "
                << timing << " seconds" << std::endl;
    }
  }

  Planner::CreateRoadmapOption opt = Planner::LazyRoadmap;
  if (voxelize_vertices)      { opt |= Planner::VoxelizeVertices; }
  if (voxelize_edges)         { opt |= Planner::VoxelizeEdges;    }
  if (check_vertex_collision) { opt |= Planner::ValidateVertices; }
  if (check_edge_collision)   { opt |= Planner::ValidateEdges;    }
  util::time_function_call([&]() { planner->createRoadmap(N, opt); }, timing);
  std::cout << "  Roadmap creation:                      "
            << timing << " seconds" << std::endl;

  // check vertices
  if (check_vertex_collision) { // collision checking does voxelization
    util::time_function_call([&]() { planner->precomputeVertexValidity(); }, timing);
    std::cout << "  Collision checking remaining vertices:  "
              << timing << " seconds" << std::endl;
  } else if (voxelize_vertices) {
    util::time_function_call([&]() { planner->precomputeVertexVoxelCache(); }, timing);
    std::cout << "  Voxelization of remaining vertices:     "
              << timing << " seconds" << std::endl;
  }

  // check edges
  if (check_edge_collision) { // collision checking does voxelization
    util::time_function_call([&]() { planner->precomputeEdgeValidity(); }, timing);
    std::cout << "  Collision checking remaining edges:     "
              << timing << " seconds" << std::endl;
  } else if (voxelize_edges) {
    util::time_function_call([&]() { planner->precomputeVoxelCache(); }, timing);
    std::cout << "  Voxelization of remaining edges:        "
              << timing << " seconds" << std::endl;
  }

  if (remove_disconnected) {
    util::time_function_call([&]() { planner->clearDisconnectedVertices(); }, timing);
    std::cout << "  Removing disconnected:                  "
              << timing << " seconds" << std::endl;
  }

  util::time_function_call(
      [&]() { planner->saveRoadmapToFile(output_fname); },
      timing);
  std::cout << "  Writing to file:                        "
            << timing << " seconds" << std::endl;

  return 0;
}
