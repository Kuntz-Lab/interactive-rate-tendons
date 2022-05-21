#include "collision/VoxelOctree.h"
#include "cliparser/CliParser.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"
#include "motion-planning/Problem.h"
#include "motion-planning/VoxelEnvironment.h"
#include "util/json_io.h"
#include "util/openfile_check.h"
#include "util/vector_ops.h"

#include <3rdparty/nlohmann/json.hpp>

//#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <Eigen/LU> // for Eigen::Matrix::inverse() definition

#include <itkBinaryDilateImageFilter.h>
#include <itkFlatStructuringElement.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkImageRegionIterator.h>
#include <itkImageRegionIteratorWithIndex.h>
#include <itkNrrdImageIO.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <utility>
#include <vector>
#include <type_traits>

#include <cmath>

using util::operator<<;

using PixelType     = unsigned char;
using ImageType     = itk::Image<PixelType, 3>;
using ImagePtr      = typename ImageType::Pointer;

namespace {

namespace defaults {
  const std::string directory = "env";
} // end of namespace defaults

void populate_parser(CliParser &parser) {
  parser.set_program_description(
      "Prepare a voxel environment to be used in planning.  It will generate\n"
      "  multiple files for each aspect of the preparations, both as .nrrd\n"
      "  and .msgpack file formats\n"
      "\n"
      "  - 01-orig\n"
      "  - 02-orig-moved\n"
      "  - 03-inverted-moved\n"
      "  - 04-inverted-moved-shell\n"
      "  - 05-inverted-moved-dilated\n"
      "  - 06-inverted-moved-dilated-shell\n"
      "  - 07-shrunken-moved-interior\n"
      "  - 08-robot-start\n"
      "  - 09-robot-start-shell\n"
      "  - 10-robot-start-rot\n"
      "  - 11-robot-start-backbone\n"
      "  - 12-robot-start-backbone-rot\n"
      "  - 13-robot-goal\n"
      "  - 14-robot-goal-shell\n"
      "  - 15-robot-goal-rot\n"
      "  - 16-robot-goal-backbone\n"
      "  - 17-robot-goal-backbone-rot\n"
      "  - 18-inverted-moved-shell-with-hole\n"
      "  - 19-inverted-moved-with-hole\n"
      "  - 20-inverted-moved-dilated-shell-with-hole\n"
      "  - 21-inverted-moved-shell-with-hole-clipped\n"
      "  - 22-inverted-moved-dilated-shell-with-hole-clipped\n"
      "  - 23-robot-start-image-coords\n"
      "  - 24-robot-start-backbone-image-coords\n"
      "  - 25-robot-goal-image-coords\n"
      "  - 26-robot-goal-backbone-image-coords\n"
      "  - 27-robot-backbone-motion-image-coords\n"
      );
  parser.add_positional("problem");
  parser.set_required("problem");
  parser.set_description("problem", "The problem specification toml file to use.\n"
      "                Only uses the robot specification and the start position.\n"
      "                Ignores the goal position and the environment.");

  parser.add_positional("environment");
  parser.set_required("environment");
  parser.set_description("environment", "Voxel file containing the Voxel\n"
      "                environment to prepare.\n"
      "                Supports nrrd, toml, toml.gz, json, bson, cbor,\n"
      "                msgpack, and ubjson.");

  parser.add_argflag("-d", "--directory");
  parser.set_description("--directory", "Output directory for all generated\n"
      "                files.  Defaults to '" + defaults::directory + "'.\n");
}

void save_nrrd(const collision::VoxelOctree &v, const std::string &fname) {
  std::cout << "Saving " << fname << std::endl;
  v.to_nrrd(fname);
}

template <typename P>
void save_nrrd(itk::SmartPointer<itk::Image<P, 3>> image, const std::string &fname) {
  using InImg = itk::Image<P, 3>;
  using OutImg = ImageType;
  using CastType = itk::CastImageFilter<InImg, OutImg>;
  typename CastType::Pointer caster;

  // convert from P to PixelType
  ImagePtr ptr;
  if constexpr (std::is_same_v<P, PixelType>) {
    ptr = image;
  } else {
    caster = CastType::New();
    caster->SetInput(image);
    ptr = caster->GetOutput();
  }
  using WriterType = itk::ImageFileWriter<ImageType>;
  auto io = itk::NrrdImageIO::New();
  auto writer = WriterType::New();
  writer->SetInput(ptr);
  writer->SetImageIO(io);
  writer->SetFileName(fname);
  writer->UseCompressionOn();
#if ITK_VERSION_MAJOR >= 5
  writer->SetCompressionLevel(4);
#endif
  std::cout << "Saving " << fname << std::endl;
  writer->Update(); // do the actual writing
}

void save_json(const collision::VoxelOctree &voxels,
               const std::string &fname)
{
  auto data = voxels.to_json();
  std::cout << "Saving " << fname << std::endl;
  util::write_json(fname, data); // supports many file types
}

void save_json(ImagePtr image, const std::string &fname) {
  auto voxels = collision::VoxelOctree::from_itk_image(image);
  save_json(voxels, fname);
}

void invert_image(ImagePtr image) {
  using IteratorType = itk::ImageRegionIterator<ImageType>;
  IteratorType it (image, image->GetLargestPossibleRegion());
  for (; !it.IsAtEnd(); ++it) {
    it.Set(!it.Get());
  }
}

template <unsigned int Dim = 3>
itk::SmartPointer<itk::Image<bool, Dim>> create_elem(
    typename itk::Image<bool, Dim>::SpacingType spacing,
    double radius)
{
  using LocalImageType = itk::Image<bool, Dim>;
  using LocalImagePtr  = typename LocalImageType::Pointer;
  using SizeType       = typename LocalImageType::SizeType;
  using RegionType     = typename LocalImageType::RegionType;
  using PointType      = typename LocalImageType::PointType;
  using DirectionType  = typename LocalImageType::DirectionType;
  using IndexType      = typename LocalImageType::IndexType;

  LocalImagePtr image = LocalImageType::New();
  image->SetSpacing(spacing);

  PointType origin;
  origin.Fill(0.0);
  image->SetOrigin(origin);

  DirectionType orientation;
  orientation.SetIdentity();
  image->SetDirection(orientation);

  IndexType start;
  start.Fill(0);
  IndexType center;
  SizeType size;
  for (unsigned int i = 0; i < center.Dimension; ++i) {
    center[i] = std::ceil(radius / spacing[i]);
    size[i] = center[i] * 2 - 1;
  }
  RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
  image->SetRegions(region);

  image->Allocate();
  image->FillBuffer(1); // rectangular prism region

  return image;
}

void prepare_env(const motion_planning::Problem &problem, ImagePtr env)
{
  // We want to create the following models and save them to file as nrrd only
  // - 01-orig.nrrd
  // - 02-orig-moved.nrrd
  // - 03-inverted-moved.nrrd
  // - 04-inverted-moved-shell.nrrd
  // - 05-inverted-moved-dilated.nrrd
  // - 06-inverted-moved-dilated-shell.nrrd
  // - 07-shrunken-moved-interior.nrrd
  // - 08-robot-start.nrrd
  // - 09-robot-start-shell.nrrd
  // - 10-robot-start-rot.nrrd
  // - 11-robot-start-backbone.nrrd
  // - 12-robot-start-backbone-rot.nrrd
  // - 13-robot-goal.nrrd
  // - 14-robot-goal-shell.nrrd
  // - 15-robot-goal-rot.nrrd
  // - 16-robot-goal-backbone.nrrd
  // - 17-robot-goal-backbone-rot.nrrd
  // - 18-inverted-moved-shell-with-hole.nrrd
  // - 19-inverted-moved-with-hole.nrrd
  // - 20-inverted-moved-dilated-shell-with-hole.nrrd
  // - 21-inverted-moved-shell-with-hole-clipped.nrrd
  // - 22-inverted-moved-dilated-shell-with-hole-clipped.nrrd
  // - 23-robot-start-image-coords.nrrd
  // - 24-robot-start-backbone-image-coords.nrrd
  // - 25-robot-goal-image-coords.nrrd
  // - 26-robot-goal-backbone-image-coords.nrrd
  // - 27-robot-backbone-motion-image-coords.nrrd

  auto &venv = problem.venv;

  // convert to voxel object's size for the image
  auto voxels = collision::VoxelOctree::from_itk_image(env);
  env = voxels.to_itk_image();

  collision::VoxelOctree workspace(voxels.Nx());
  const auto workspace_radius = problem.robot.specs.L + problem.robot.r;
  workspace.set_xlim(- workspace_radius, workspace_radius);
  workspace.set_ylim(- workspace_radius, workspace_radius);
  workspace.set_zlim(- workspace_radius, workspace_radius);

  // 1. original image
  save_nrrd(env, "01-orig.nrrd");
  save_json(env, "01-orig.msgpack");

  // 2. moved image
  venv.translate_and_scale_image(env);
  auto interior_voxels = collision::VoxelOctree::from_itk_image(env);
  save_nrrd(env, "02-orig-moved.nrrd");
  save_json(env, "02-orig-moved.msgpack");
  {
    auto env_in_robot = interior_voxels;
    auto mesh = env_in_robot.to_mesh();
    auto rotation = venv.inv_rotation.inverse();
    for (auto &vertex : mesh.vertices) {
      Eigen::Vector3d point{vertex[0], vertex[1], vertex[2]};
      point = rotation * point;
      vertex[0] = float(point[0]);
      vertex[1] = float(point[1]);
      vertex[2] = float(point[2]);
    }
    std::cout << "Saving 02-orig-moved-robot-coords.stl" << std::endl;
    mesh.to_stl("02-orig-moved-robot-coords.stl");
  }

  // 3. inverted moved image
  invert_image(env);
  save_nrrd(env, "03-inverted-moved.nrrd");
  save_json(env, "03-inverted-moved.msgpack");

  // 4. inverted moved shell
  voxels = collision::VoxelOctree::from_itk_image(env);
  voxels.remove_interior();
  save_nrrd(voxels, "04-inverted-moved-shell.nrrd");
  save_json(voxels, "04-inverted-moved-shell.msgpack");

  // 5. inverted moved and dilated
  //using StructuringElementType = itk::FlatStructuringElement<3>;
  //using DilateFilterType = itk::BinaryDilateImageFilter<
  //    ImageType, ImageType, StructuringElementType>;
  //auto region = env->GetLargestPossibleRegion();
  //auto spacing = env->GetSpacing();
  //auto prism = create_elem(spacing, problem.robot.r);
  //auto elem = StructuringElementType::FromImage(prism);
  ImagePtr dilate_im;
  {
    voxels = collision::VoxelOctree::from_itk_image(env);
    voxels.dilate_sphere(problem.robot.r);
    dilate_im = voxels.to_itk_image();
    //auto dilate_filter = DilateFilterType::New();
    //dilate_filter->SetKernel(elem);
    //dilate_filter->SetInput(env);
    //dilate_filter->SetDilateValue(1);
    //dilate_filter->Update();  // apply the dilation
    //dilate_im = dilate_filter->GetOutput();
    save_nrrd(dilate_im, "05-inverted-moved-dilated.nrrd");
    save_json(dilate_im, "05-inverted-moved-dilated.msgpack");
  }
  //save_nrrd(prism, "dilate-shape.nrrd");

  // 6. inverted moved and dilated shell
  {
    voxels = collision::VoxelOctree::from_itk_image(dilate_im);
    voxels.remove_interior();
    save_nrrd(voxels, "06-inverted-moved-dilated-shell.nrrd");
    save_json(voxels, "06-inverted-moved-dilated-shell.msgpack");
  }

  // 7. inverted, moved, dilated, then reinverted - shrunken interior moved
  {
    voxels = collision::VoxelOctree::from_itk_image(dilate_im);
    auto dilate_im_inverted = voxels.to_itk_image();
    invert_image(dilate_im_inverted);
    save_nrrd(dilate_im_inverted, "07-shrunken-moved-interior.nrrd");
    save_json(dilate_im_inverted, "07-shrunken-moved-interior.msgpack");
  }

  {
    // 8. robot start
    auto robot = workspace.empty_copy();
    auto shape = problem.start_shape();
    for (auto &p : shape.p) {
      robot.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot, "08-robot-start.nrrd");
    save_json(robot, "08-robot-start.msgpack");

    // 9. robot start shell
    robot.remove_interior();
    save_nrrd(robot, "09-robot-start-shell.nrrd");
    save_json(robot, "09-robot-start-shell.msgpack");
  }

  // 10. robot start rotated
  {
    auto robot_rot = workspace.empty_copy();
    auto pvec_rot = problem.start_shape().p;
    venv.rotate_points(pvec_rot);
    for (auto &p : pvec_rot) {
      robot_rot.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot_rot, "10-robot-start-rot.nrrd");
    save_json(robot_rot, "10-robot-start-rot.msgpack");
  }

  // 11. robot start backbone
  {
    auto robot = workspace.empty_copy();
    auto shape = problem.start_shape();
    robot.add_piecewise_line(shape.p);
    save_nrrd(robot, "11-robot-start-backbone.nrrd");
    save_json(robot, "11-robot-start-backbone.msgpack");
  }

  // 12. robot start backbone rotated
  {
    auto robot_rot = workspace.empty_copy();
    auto pvec_rot = problem.start_shape().p;
    venv.rotate_points(pvec_rot);
    robot_rot.add_piecewise_line(pvec_rot);
    save_nrrd(robot_rot, "12-robot-start-backbone-rot.nrrd");
    save_json(robot_rot, "12-robot-start-backbone-rot.msgpack");
  }

  {
    // 13. robot goal
    auto goal_shape = problem.goal_shape();
    auto robot = workspace.empty_copy();
    for (auto &p : goal_shape.p) {
      robot.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot, "13-robot-goal.nrrd");
    save_json(robot, "13-robot-goal.msgpack");

    // 14. robot goal shell
    robot.remove_interior();
    save_nrrd(robot, "14-robot-goal-shell.nrrd");
    save_json(robot, "14-robot-goal-shell.msgpack");
  }

  // 15. robot goal rotated
  {
    auto robot_rot = workspace.empty_copy();
    auto pvec_rot = problem.goal_shape().p;
    venv.rotate_points(pvec_rot);
    for (auto &p : pvec_rot) {
      robot_rot.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot_rot, "15-robot-goal-rot.nrrd");
    save_json(robot_rot, "15-robot-goal-rot.msgpack");
  }

  // 16. robot goal backbone
  {
    auto robot = workspace.empty_copy();
    auto shape = problem.goal_shape();
    robot.add_piecewise_line(shape.p);
    save_nrrd(robot, "16-robot-goal-backbone.nrrd");
    save_json(robot, "16-robot-goal-backbone.msgpack");
  }

  // 17. robot goal backbone rot
  {
    auto robot_rot = workspace.empty_copy();
    auto pvec_rot = problem.goal_shape().p;
    venv.rotate_points(pvec_rot);
    robot_rot.add_piecewise_line(pvec_rot);
    save_nrrd(robot_rot, "17-robot-goal-backbone-rot.nrrd");
    save_json(robot_rot, "17-robot-goal-backbone-rot.msgpack");
  }

  // 18. inverted moved shell with hole
  {
    auto env_voxels = collision::VoxelOctree::from_itk_image(env);
    env_voxels.remove_interior();
    auto robot_base = env_voxels.empty_copy();
    auto pvec_rot = problem.start_shape().p;
    venv.rotate_points(pvec_rot);
    for (size_t i = 0; i < std::min(size_t(3), pvec_rot.size()); i++) { // first 3
      robot_base.add_sphere({pvec_rot[i], problem.robot.r*1.5}); // enlarged
    }
    env_voxels.remove(robot_base);
    save_nrrd(env_voxels, "18-inverted-moved-shell-with-hole.nrrd");
    save_json(env_voxels, "18-inverted-moved-shell-with-hole.msgpack");
  }

  {
    // 19. inverted moved with a long hole
    auto env_hole = collision::VoxelOctree::from_itk_image(env);
    std::vector<Eigen::Vector3d> pvec_rot {};
    for (double s = -0.5; s <= 2.0; s += 0.01) {
      pvec_rot.emplace_back(0, 0, s * problem.robot.r);
    }
    venv.rotate_points(pvec_rot);
    auto voxel_sphere = env_hole.empty_copy();
    for (auto &p : pvec_rot) {
      voxel_sphere.add_sphere({p, 1.5 * problem.robot.r});
    }
    env_hole.remove(voxel_sphere); // enlarged
    save_nrrd(env_hole, "19-inverted-moved-with-hole.nrrd");
    save_json(env_hole, "19-inverted-moved-with-hole.msgpack");

    // 20. inverted moved dilated shell with hole
    env_hole.dilate_sphere(problem.robot.r);
    env_hole.remove_interior();
    save_nrrd(env_hole, "20-inverted-moved-dilated-shell-with-hole.nrrd");
    save_json(env_hole, "20-inverted-moved-dilated-shell-with-hole.msgpack");

    // 21. inverted moved shell clipped with hole
    auto voxel_env = voxels;
    auto robot_workspace = voxels.empty_copy();
    robot_workspace.add_sphere({{0, 0, 0}, workspace_radius});
    voxel_env.intersect(robot_workspace);
    save_nrrd(voxel_env,
              "21-inverted-moved-shell-clipped-with-hole.nrrd");
    save_json(voxel_env,
              "21-inverted-moved-shell-clipped-with-hole.msgpack");

    // 22. inverted moved dilated shell clipped with hole
    auto dilate_voxels = env_hole;
    dilate_voxels.intersect(robot_workspace);
    save_nrrd(dilate_voxels,
              "22-inverted-moved-dilated-shell-clipped-with-hole.nrrd");
    save_json(dilate_voxels,
              "22-inverted-moved-dilated-shell-clipped-with-hole.msgpack");
  }

  {
    // 23. robot start in image coordinates
    auto robot_im = voxels.empty_copy();
    auto pvec_rot = problem.start_shape().p;
    venv.rotate_points(pvec_rot);
    for (auto &p : pvec_rot) {
      robot_im.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot_im, "23-robot-start-image-coords.nrrd");
    save_json(robot_im, "23-robot-start-image-coords.msgpack");

    // 24. robot start backbone in image coordinates
    auto backbone_im = voxels.empty_copy();
    backbone_im.add_piecewise_line(pvec_rot);
    save_nrrd(backbone_im, "24-robot-start-backbone-image-coords.nrrd");
    save_json(backbone_im, "24-robot-start-backbone-image-coords.msgpack");
  }

  {
    // 25. robot goal in image coordinates
    auto robot_im = voxels.empty_copy();
    auto pvec_rot = problem.goal_shape().p;
    venv.rotate_points(pvec_rot);
    for (auto &p : pvec_rot) {
      robot_im.add_sphere(collision::Sphere{p, problem.robot.r});
    }
    save_nrrd(robot_im, "25-robot-goal-image-corrds.nrrd");
    save_json(robot_im, "25-robot-goal-image-corrds.msgpack");

    // 26. robot goal backbone in image coordinates
    auto backbone_im = voxels.empty_copy();
    backbone_im.add_piecewise_line(pvec_rot);
    save_nrrd(backbone_im, "26-robot-goal-backbone-image-coords.nrrd");
    save_json(backbone_im, "26-robot-goal-backbone-image-coords.msgpack");
  }

  // 27. robot backbone motion in image coordinates
  {
    auto motion_im = venv.voxelize_backbone_motion(voxels, problem.robot,
                                                   problem.start_state(),
                                                   problem.goal_state());
    save_nrrd(motion_im, "27-robot-backbone-motion-image-coords.nrrd");
    save_json(motion_im, "27-robot-backbone-motion-image-coords.msgpack");
  }

  // 28. robot backbone motion not in image coordinates
  {
    auto finer_robot = problem.robot;
    finer_robot.specs.dL = workspace.dx();
    auto motion_im = venv.voxelize_backbone_motion(workspace, finer_robot,
                                                   problem.start_state(),
                                                   problem.goal_state());
    save_nrrd(motion_im, "28-robot-backbone-motion.nrrd");
    save_json(motion_im, "28-robot-backbone-motion.msgpack");
  }
}

} // end of unnamed namespace

int main(int argCount, char* argList[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(argCount, argList);

  std::cout << "Loading " << parser["problem"] << std::endl;
  auto problem = cpptoml::from_file<motion_planning::Problem>(parser["problem"]);
  std::cout << "Loading " << parser["environment"] << std::endl;
  auto image = collision::VoxelOctree::from_file(parser["environment"]).to_itk_image();
  auto directory = parser.get("--directory", defaults::directory);

  boost::filesystem::create_directories(directory);
  boost::filesystem::current_path(directory);

  prepare_env(problem, image);

  return 0;
}
