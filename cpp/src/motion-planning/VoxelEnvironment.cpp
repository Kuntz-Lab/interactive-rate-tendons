#include "VoxelEnvironment.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include <collision/VoxelOctree.h>
#include <util/angles.h>
#include <util/macros.h>

#include <itkImage.h>

#include <Eigen/Core>
#include <Eigen/Geometry>  // for Quaterniond

namespace E = Eigen;

namespace motion_planning {

std::shared_ptr<cpptoml::table> VoxelEnvironment::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("voxel_environment", tbl);

  VoxelEnvironment defaults;
  if (*this == defaults) {
    return tbl; // return an empty container if not populated
  }

  // allow the voxel environment to be stored in the toml directly
  if (filename.empty() && _obstacle_cache) {
    auto sub_tbl = _obstacle_cache->to_toml();
    for (auto &[key, val] : *sub_tbl) {
      container->insert(key, val);
    }
  } else {
    tbl->insert("filename",      filename);
  }

  tbl->insert("interior_filename", interior_fname);
  tbl->insert("scaling",       scaling);
  tbl->insert("translation",   cpptoml::to_toml(translation));
  E::Quaterniond q(inv_rotation);
  tbl->insert("rotation_quat", cpptoml::to_toml(
        std::vector<double>{q.w(), q.x(), q.y(), q.z()}));
  return container;
}

VoxelEnvironment VoxelEnvironment::from_toml(
    std::shared_ptr<cpptoml::table> tbl)
{
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto top_level = tbl;
  // try to grab the voxel_environment container, otherwise we're inside
  if (tbl->contains("voxel_environment")) {
    auto container = tbl->get("voxel_environment")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'voxel_environment': not a table");
    }
    tbl = container; // pull from the container instead
  }

  VoxelEnvironment env;

  if (tbl->contains("filename")) {
    auto fname  = tbl->get("filename")->as<std::string>();
    if (!fname) {
      throw cpptoml::parse_exception("Wrong type detected for 'filename'");
    }
    env.filename = fname->get();
  } else {
    auto voxels = collision::VoxelOctree::from_toml(top_level);
    env.set_obstacle_cache(std::make_shared<collision::VoxelOctree>(voxels));
  }

  auto scaling     = tbl->get("scaling")->as<double>();
  auto translation = tbl->get("translation")->as_array();
  auto rotation    = tbl->get("rotation_quat")->as_array();

  if (!(scaling && translation && rotation)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }

  auto rotvec     = cpptoml::to_stdvec<double>(rotation);
  E::Quaterniond quat(rotvec[0], rotvec[1], rotvec[2], rotvec[3]);

  env.scaling      = scaling->get();
  env.translation  = cpptoml::to_point(translation);
  env.inv_rotation = quat.toRotationMatrix();

  if (tbl->contains("interior_filename")) {
    auto interior_fname = tbl->get("interior_filename")->as<std::string>();
    if (!interior_fname) {
      throw cpptoml::parse_exception("Wrong type detected for interior_filename");
    }
    env.interior_fname = interior_fname->get();
  }

  return env;
}

void VoxelEnvironment::translate_and_scale_image(
    VoxelEnvironment::ImagePtr image) const
{
  auto origin  = image->GetOrigin();
  auto spacing = image->GetSpacing();

  // translate
  origin[0] += translation[0];
  origin[1] += translation[1];
  origin[2] += translation[2];

  // scale
  origin[0] *= scaling;
  origin[1] *= scaling;
  origin[2] *= scaling;
  spacing[0] *= scaling;
  spacing[1] *= scaling;
  spacing[2] *= scaling;

  // update the image
  image->SetOrigin(origin);
  image->SetSpacing(spacing);
}

E::Vector3d VoxelEnvironment::rotate_point(const E::Vector3d &point) const {
  return inv_rotation * point;
}

void VoxelEnvironment::rotate_points(std::vector<E::Vector3d> &points) const {
  for (auto &p : points) { p = inv_rotation * p; }
}

collision::VoxelOctree VoxelEnvironment::voxelize_backbone_motion(
    const collision::VoxelOctree &reference,
    const tendon::TendonRobot &robot,
    const std::vector<double> &start,
    const std::vector<double> &end,
    double rel_threshold) const
{
  auto fk = [&robot](const std::vector<double> &config) {
    return robot.shape(config);
  };

  auto interp = [&robot](const std::vector<double> &a,
                         const std::vector<double> &b,
                         double t, std::vector<double> &out)
  {
    auto N = robot.tendons.size();
    for (size_t i = 0; i < a.size(); i++) {
      out[i] = a[i] + t * (b[i] - a[i]);
    }

    // fix rotation
    if (robot.enable_rotation) {
      double diff = util::canonical_angle(b[N] - a[N]);
      out[N] = util::canonical_angle(a[N] + t * diff);
    }
  };

  const size_t N = robot.state_size();
  if (start.size() != N || end.size() != N) {
    throw std::invalid_argument("start or end the wrong size");
  }

  // checking assumption 1
  auto &ref = reference;
  if (robot.specs.dL > std::min({ref.dx(), ref.dy(), ref.dz()})) {
    throw std::invalid_argument(
        "robot backbone discretization is too large for this voxel grid.");
  }

  return voxelize_backbone_motion(reference, interp, fk, start, end, rel_threshold);
}

collision::VoxelOctree VoxelEnvironment::voxelize_backbone_motion(
    const collision::VoxelOctree &reference,
    const InterpFunc &interp,
    const FkFunc &fk,
    const std::vector<double> &start,
    const std::vector<double> &end,
    double rel_threshold) const
{
  auto checker = [](const std::vector<double> &config,
                    const tendon::TendonResult &shape)
  {
    UNUSED_VAR(config);
    UNUSED_VAR(shape);
    return true;
  };

  auto partial = this->voxelize_valid_backbone_motion(
      reference, interp, fk, checker, start, end, rel_threshold);

  if (!partial.is_fully_valid) {
    throw std::runtime_error(
        "voxelize_backbone_motion() got an invalid edge with a noop checker");
  }
  if (std::abs(partial.t - 1.0) > 1e-9) {
    std::ostringstream out;
    out << "voxelize_backbone_motion() got t != 1.0 (actual: " << partial.t << ")";
    throw std::runtime_error(out.str());
  }

  return partial.voxels;
}

VoxelEnvironment::PartialVoxelization
VoxelEnvironment::voxelize_valid_backbone_motion(
    const collision::VoxelOctree &reference,
    const InterpFunc &interp,
    const FkFunc &fk,
    const ValidFunc &checker,
    const std::vector<double> &start,
    const std::vector<double> &end,
    double rel_threshold) const
{
  using FkType = decltype(tendon::TendonResult::p);
  auto voxels = reference.empty_copy();
  const size_t N = start.size();
  if (end.size() != N) {
    throw std::invalid_argument("start and end are different sizes");
  }

  // Algorithm:
  // 1. Create a frontier of configuration intervals (implement as a stack)
  //    - This is a stack so that we can do the first half first to more
  //      efficiently determine the valid portion of the motion.
  //    - A stack also limits the amount of memory required by the frontier
  // 2. Create the interval (start, end)
  // 2a. Voxelize start and end shapes
  // 2b. Add this interval to the frontier only if it should be subdividied.
  // 3. While the frontier is not empty
  // 3a. Pop an interval off of the frontier
  // 3b. Find the midpoint and create two new intervals
  // 3c. Voxelize midpoint shape
  // 3d. For both new midpoints, check if each should be subdivided
  //
  // Assumptions:
  // 1. The robot discretization is less than or equal to the smallest voxel
  //    discretization size (so that voxelizing the backbone is simply adding
  //    each point)
  //    > This assumption is checked in the variant with the robot argument
  // 2. The shape given by the robot needs to be rotated (using rotate_points())
  //    before being voxelized.  This function will rotate for you.
  //
  // Details:
  // - Adding to the frontier adds the points to the voxel octree
  // - The frontier holds an object containing the
  //   > beginning shape (after rotation)
  //   > beginning interpolation value
  //   > ending shape (after rotation)
  //   > ending interpolation value
  // - The frontier is a stack to avoid it growing too large in size.  A queue
  //   will be like a depth-first search, whereas a stack is like a depth-first
  //   search.
  // - Subdividing only happens if one point along the backbone is more than
  //   the adjacent voxel

  // tuples of (t, fkshape, is_valid).  Adds to fks, returns the index
  std::vector<std::tuple<double, FkType, bool>> fks;
  double first_invalid_t = 10.0; // nothing is invalid to start out
  auto add_fk = [this, &fk, &fks, &checker, &first_invalid_t]
                (double t, const auto &config)
  {
    auto i = fks.size();
    auto shape = fk(config);
    auto is_valid = checker(config, shape);
    if (!is_valid && t < first_invalid_t) { first_invalid_t = t; }
    this->rotate_points(shape.p); // rotate after passing to checker
    fks.emplace_back(t, std::move(shape.p), is_valid);
    return i;
  };

  // interpolate between start and end (0 <= t <= 1), add to fks, and return index
  auto current = start;
  // TODO: make a simple interpolate() function in util/vector_ops.h
  auto interpolate = [this, &current, &start, &end, &add_fk, &interp] (double t) {
    interp(start, end, t, current);
    return add_fk(t, current);
  };

  //
  // 1. Create a frontier of configuration intervals
  //

  // struct to store in the frontier
  //struct Interval {
  //  size_t a; // index into fks
  //  size_t b; // index into fks
  //};
  using Interval = std::pair<size_t, size_t>;
  std::stack<Interval> frontier;

  //
  // 2. Create the interval (start, end)
  // 2a. Voxelize start and end shapes
  // 2b. Add this interval to the frontier only if it should be subdividied.
  //

  // make the first interval
  Interval motion {add_fk(0.0, start), add_fk(1.0, end)};

  // returns true if one backbone point is more than one voxel distance away
  auto should_subdivide = [&voxels, &fks](size_t a, size_t b) {
    const auto &[t_a, shape_a, is_valid_a] = fks[a]; // initial
    const auto &[t_b, shape_b, is_valid_b] = fks[b]; // final
    if (!is_valid_a) {
      //std::cout << "should_subdivide(): exiting early because !is_valid_a" << std::endl;
      return false;
    }
    UNUSED_VAR(t_a);
    UNUSED_VAR(t_b);
    UNUSED_VAR(is_valid_b);

    // Because of retraction, the backbones can be of different lengths
    // Account for the different lengths, allow up to one link longer
    if (shape_a.size() + 1 < shape_b.size() ||
        shape_a.size()     > shape_b.size() + 1)
    {
      //std::cout << "    length difference: "
      //          << std::abs(long(shape_a.size()) - long(shape_b.size()))
      //          << std::endl;
      return true;
    }
    int P = int(std::min(shape_a.size(), shape_b.size()));
    for (int i = P-1; i >= 0; i--) {
      auto [sx, sy, sz] = voxels.find_cell(shape_a[i]);
      auto [ex, ey, ez] = voxels.find_cell(shape_b[i]);
      long dx = std::abs(long(sx) - long(ex));
      long dy = std::abs(long(sy) - long(ey));
      long dz = std::abs(long(sz) - long(ez));
      //auto distance = dx + dy + dz;
      //if (distance > 1) {
      if (dx > 1 || dy > 1 || dz > 1) {
        //std::cout << "    voxel distance at " << i << " of " << P-1 << ": ("
        //          << dx << ", " << dy << ", " << dz << ")" << std::endl;
        return true;
      }
    }
    return false;
  };

  //
  // 3. While the frontier is not empty
  // 3a. Pop an interval off of the frontier
  // 3b. Find the midpoint and create two new intervals
  // 3c. Voxelize midpoint shape
  // 3d. For both subintervals, check if each should be subdivided
  //

  if (should_subdivide(motion.first, motion.second)) {
    frontier.push(std::move(motion));
    //std::cout << "Added first motion to frontier, size: " << frontier.size() << std::endl;
  }

  //int n_divisions = 0;
  while (!frontier.empty()) {
    auto interval = std::move(frontier.top());
    frontier.pop();
    //std::cout << "Popped motion from frontier, size: " << frontier.size() << ", (" << interval.first << ", " << interval.second << ")" << std::endl;

    const auto &[t_a, shape_a, is_valid_a] = fks[interval.first]; // initial
    const auto &[t_b, shape_b, is_valid_b] = fks[interval.second]; // final
    UNUSED_VAR(shape_a);
    UNUSED_VAR(shape_b);
    UNUSED_VAR(is_valid_a);
    UNUSED_VAR(is_valid_b);

    if ((t_b - t_a) <= rel_threshold) {
      //std::cout << "  Skipped (" << interval.first << ", " << interval.second << ") because rel threshold exceeded: (t_a: " << t_a << ", t_b: " << t_b << ", rel_threshold: " << rel_threshold << ")" << std::endl;
      continue; // already at smallest size, no more bisections
    }
    if (first_invalid_t <= t_a) {
      //std::cout << "  Skipped (" << interval.first << ", " << interval.second << ") because exceeded an invalid t: (t_a: " << t_a << ", t_b: " << t_b << ", first_invalid_t: " << first_invalid_t << ")" << std::endl;
      continue; // skip this bisection, is already in invalid region
    }
    //n_divisions++;
    //std::cout
    //  << std::setprecision(50)
    //  << "  interval " << n_divisions
    //  << " (" << t_a << ", " << t_b << ")" << std::endl;

    auto mid_interp = (t_a + t_b) / 2;

    auto mid_idx = interpolate(mid_interp);

    // add distal end first
    if (should_subdivide(mid_idx, interval.second)) {
      frontier.emplace(mid_idx, interval.second);
      //std::cout << "Added distal motion to frontier, size: " << frontier.size() << std::endl;
    }

    // add proximal end last, so it gets popped off first
    if (should_subdivide(interval.first, mid_idx)) {
      frontier.emplace(interval.first, mid_idx);
      //std::cout << "Added proximal motion to frontier, size: " << frontier.size() << std::endl;
    }
  }
  //std::cout << "Total subdivisions: " << n_divisions << std::endl;

  // voxelize all those that are less than the first invalid t
  // capture the last valid state while voxelizing
  double last_valid_t = 0.0;
  size_t last_valid_idx = 0;
  //std::cout << "fks.size():         " << fks.size() << std::endl;
  for (size_t i = fks.size(); i-->0; ) { // cheaper to count downward
    auto &[t, shape, is_valid] = fks[i];
    //std::cout << "  " << i << ": [t: " << t << ", shape.size(): " << shape.size() << ", is_valid: " << is_valid << "]" << std::endl;
    if (t < first_invalid_t) {
      if (!is_valid) { // assert that they should all be valid
        throw std::runtime_error(
            "voxelize_valid_backbone_motion(): invalid shape in region"
            " marked as valid");
      }
      voxels.add_piecewise_line(shape);
      // find the last valid state
      if (last_valid_t < t) {
        last_valid_t = t;
        last_valid_idx = i;
      }
    }
  }

  auto &[t, shape, is_valid] = fks[last_valid_idx];
  UNUSED_VAR(is_valid);

  if (t != last_valid_t) {
    std::ostringstream msg_builder;
    msg_builder
      << "voxelize_valid_backbone_motion(): last_valid_t != fks[last_valid_idx].t "
         "(" << last_valid_t << " != " << t << ")";
    throw std::runtime_error(msg_builder.str());
  }

  interp(start, end, t, current);

  PartialVoxelization answer{};
  answer.is_fully_valid = (5.0 < first_invalid_t);
  answer.t = t;
  answer.last_valid = std::move(current);
  answer.last_backbone = std::move(shape);
  answer.voxels = std::move(voxels);
  return answer;
}

} // end of namespace motion_planning
