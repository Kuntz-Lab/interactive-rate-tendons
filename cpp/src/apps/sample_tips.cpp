#include "cliparser/CliParser.h"
#include "collision/VoxelOctree.h"
#include "cpptoml/toml_conversions.h"
#include "csv/Csv.h"
#include "motion-planning/Problem.h"
#include "util/openfile_check.h"
#include "util/vector_ops.h"
#include "util/macros.h"

//#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <itkImage.h>
#include <itkImageFileReader.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <bitset>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator> // for std::back_iterator
#include <random>
#include <string>
#include <vector>

namespace E = Eigen;

namespace {

std::mt19937 rng;

namespace defaults {
  const std::string output = "samples.csv";
  const size_t number = 1000;
}

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Generate many valid samples of the tip position space and store them in\n"
      "  a CSV file.  It will store the tip position.");

  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem",
                      "The problem specification toml file to use\n"
      "                This file is required to have a valid file populated\n"
      "                for voxel_environment.interior_filename.  That file\n"
      "                has interior free space points marked as 'occupied'\n"
      "                voxels.  This is how valid samples are chosen instead\n"
      "                of using the voxel_environment.filename for obstacles\n"
      "                to avoid (since this may be more sparse - like a shell\n"
      "                - for performance reasons).");

  parser.add_argflag("-o", "--output");
  parser.set_description("--output", "CSV file to output\n"
      "                (default is " + defaults::output + ")");

  parser.add_argflag("-N", "--number");
  parser.set_description("--number",
                      "Number of valid samples to take.\n"
      "                (default is " + std::to_string(defaults::number) + ")");

  parser.add_flag("--backbone");
  parser.set_description("--backbone", "Only collision check of backbone\n"
      "                against the voxel environment instead of the full\n"
      "                robot shape.  This option makes collision checking\n"
      "                only check the current point for validity.  Without\n"
      "                this option, a sphere the radius of the robot will\n"
      "                be checked.");

  parser.add_argflag("--random-walk");
  parser.set_description("--random-walk",
                      "Perform a random walk with a minimum and maximum step\n"
      "                size.  The argument expected by this option is two\n"
      "                numbers separated by a comma, which are\n"
      "                \"min_step_size,max_step_size\" (e.g., 0.01,0.05).\n"
      "                If only one number is specified, then the\n"
      "                min_step_size defaults to 0.0 and the specified\n"
      "                number is the max_step_size.");
}

struct ParsedOptions {
  std::string probfile;
  std::string output;
  size_t      number;
  bool        is_backbone;
  bool        is_random_walk;
  double      random_walk_min = 0.0;
  double      random_walk_max = 0.0;
};

ParsedOptions parse_args(CliParser &parser, int arg_count, char *arg_list[]) {
  parser.parse(arg_count, arg_list);

  ParsedOptions opt;
  opt.probfile        = parser["problem"];
  opt.output          = parser.get("--output", defaults::output);
  opt.number          = parser.get("--number", defaults::number);
  opt.is_backbone     = parser.has("--backbone");

  opt.is_random_walk  = parser.has("--random-walk");
  if (opt.is_random_walk) {
    auto walk_range = parser["--random-walk"];
    auto comma_idx  = walk_range.find(',');
    if (comma_idx == std::string::npos) {
      opt.random_walk_min = 0.0;
      opt.random_walk_max = std::stod(walk_range);
    } else {
      opt.random_walk_min = std::stod(walk_range.substr(0, comma_idx));
      opt.random_walk_max = std::stod(walk_range.substr(comma_idx + 1));
    }
  }

  return opt;
}

/** check if a tip position is valid (in the interior)
 *
 * Check if tip position is in the interior voxels.  If you specify a radius,
 * then a sphere centered around that tip position is checked to be entirely
 * within the voxel interior.
 */
bool is_interior(const collision::VoxelOctree &interior_voxels,
                 const E::Vector3d &tip_position,
                 double radius = 0.0)
{
  if (radius == 0.0) {
    return interior_voxels.collides(tip_position);
  } else {
    auto tip_sphere = interior_voxels.empty_copy();
    tip_sphere.add_sphere({tip_position, radius});
    auto intersection = tip_sphere;
    intersection.intersect(interior_voxels);
    return intersection == tip_sphere;
  }
}

/** return the ith occupied voxel */
// TODO: put this into the voxel class
std::tuple<size_t, size_t, size_t> ith_cell(
    const collision::VoxelOctree &voxels, size_t i)
{
  size_t ix, iy, iz;
  class exit_early : std::exception {};
  try {
    voxels.visit_leaves([&i, &ix, &iy, &iz]
      (size_t bx, size_t by, size_t bz, uint64_t val) {
        std::bitset<64> bits(val);
        if (i >= bits.count()) {
          i -= bits.count();
        } else {
          for (size_t j = 0; j < 64; ++j) {
            if (bits[j]) {
              if (i > 0) {
                --i;
              } else {
                iz = bz * 4 + j % 4;
                iy = by * 4 + (j /  4) % 4;
                ix = bx * 4 + (j / 16) % 4;
                throw exit_early();
              }
            }
          }
          throw std::runtime_error("could not find the ith cell");
        }
      });
  } catch (exit_early&) {}

  if (i > 0) {
    throw std::domain_error("requested a voxel index too large");
  }
  return {ix, iy, iz};
}

E::Vector3d random_direction() {
  auto rotation = E::Quaterniond::UnitRandom();
  return rotation * E::Vector3d::UnitX();
}

E::Vector3d interior_step(
    const E::Vector3d &current_point,
    std::function<bool(E::Vector3d)> is_interior_func,
    double step_min,
    double step_max)
{
  std::uniform_real_distribution<double> len_distribution(step_min, step_max);
  auto next_point = current_point;
  do {
    E::Vector3d step = len_distribution(rng) * random_direction();
    next_point = current_point + step;
  } while (!is_interior_func(next_point)); // rejection sampling
  return next_point;
}

/** create N random samples in the interior */
// TODO: put this into the voxel class
std::vector<E::Vector3d> sample_interior(
    const collision::VoxelOctree &interior_voxels, size_t N)
{
  std::uniform_int_distribution<size_t> chooser(0, interior_voxels.ncells());

  const auto dx = interior_voxels.dx();
  const auto dy = interior_voxels.dy();
  const auto dz = interior_voxels.dz();

  std::vector<E::Vector3d> samples(N);
  for (size_t i = 0; i < N; ++i) {
    const auto [ix, iy, iz] = ith_cell(interior_voxels, chooser(rng));

    auto xmin = interior_voxels.xlim().first + ix * dx;
    auto xmax = xmin + dx;
    auto ymin = interior_voxels.ylim().first + iy * dy;
    auto ymax = ymin + dy;
    auto zmin = interior_voxels.zlim().first + iz * dz;
    auto zmax = zmin + dz;

    std::uniform_real_distribution xchooser(xmin, xmax);
    std::uniform_real_distribution ychooser(ymin, ymax);
    std::uniform_real_distribution zchooser(zmin, zmax);

    samples[i] = {xchooser(rng),
                  ychooser(rng),
                  zchooser(rng)};
  }
  return samples;
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  auto opt = parse_args(parser, arg_count, arg_list);

  std::cout << "probfile: " << opt.probfile << std::endl;

  auto problem  = cpptoml::from_file<motion_planning::Problem>(opt.probfile);
  auto voxels   = problem.venv.get_interior();

  auto is_valid = [&opt, &voxels, &problem]
    (const E::Vector3d &tip) -> bool {
      double radius = opt.is_backbone ? 0.0 : problem.robot.r;
      return is_interior(*voxels, tip, radius);
    };

  std::vector<E::Vector3d> valid_samples;
  if (opt.is_random_walk) {
    valid_samples.emplace_back(0.0, 0.0, 0.0); // start at the origin
  }
  while (valid_samples.size() < opt.number) {
    if (!opt.is_random_walk) {
      auto untested_samples =
          sample_interior(*voxels, opt.number - valid_samples.size());
      std::copy_if(untested_samples.begin(), untested_samples.end(),
                   std::back_inserter(valid_samples), is_valid);
    } else { // random walk
      for (size_t i = valid_samples.size(); i < opt.number; ++i) {
        valid_samples.emplace_back(
            interior_step(valid_samples.back(), is_valid,
                          opt.random_walk_min, opt.random_walk_max));
      }
    }
  }

  for (auto &p : valid_samples) {
    p = problem.venv.inv_rotation.transpose() * p;
  }

  std::ofstream out;
  util::openfile_check(out, opt.output);
  csv::CsvWriter writer(out);
  std::cout << "writing to " << opt.output << std::endl;

  writer << "tip_x" << "tip_y" << "tip_z";
  writer.new_row();

  for (auto &sample : valid_samples) {
    writer << sample[0] << sample[1] << sample[2];
    writer.new_row();
  }

  return 0;
}
