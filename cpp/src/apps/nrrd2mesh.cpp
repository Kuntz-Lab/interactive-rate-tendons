#include "cliparser/CliParser.h"
#include "collision/Mesh.h"
#include "collision/VoxelOctree.h"

#include <itkBinaryMask3DMeshSource.h>
#include <itkBinaryThresholdImageFilter.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkMesh.h>

#include <limits>

namespace {

using PixelType = unsigned char;
using ImageType = itk::Image<PixelType, 3>;
using ImagePtr  = typename ImageType::Pointer;

void populate_parser(CliParser &parser) {
  parser.set_program_description("Convert a nrrd voxel image to a mesh");

  parser.add_positional("nrrd-file");
  parser.set_required("nrrd-file");
  parser.set_description("nrrd-file", "3D binary voxel grid file to read");

  parser.add_flag("-a", "--ascii");
  parser.set_description("--ascii", "save in ascii format instead of binary.");

  parser.add_flag("--marching-cubes");
  parser.set_description("--marching-cubes", "use marching cubes instead of the\n"
      "                default of outputting a boxy mesh that exactly matches the\n"
      "                voxel object.");
}

ImagePtr load_image(const std::string &fname, PixelType threshold = 1) {
  using ReaderType = itk::ImageFileReader<ImageType>;
  auto reader = ReaderType::New();
  reader->SetFileName(fname);
  reader->Update();

  using ThresholdFilterType =
      itk::BinaryThresholdImageFilter<ImageType, ImageType>;
  auto threshold_filter = ThresholdFilterType::New();
  threshold_filter->SetInput(reader->GetOutput());
  // everything not equal to zero is set to 1
  threshold_filter->SetOutsideValue(0);
  threshold_filter->SetInsideValue(1);
  threshold_filter->SetLowerThreshold(threshold);
  threshold_filter->SetUpperThreshold(std::numeric_limits<PixelType>::max());

  threshold_filter->Update();

  return threshold_filter->GetOutput();
}

collision::Mesh to_box_mesh(ImagePtr image) {
  auto voxels = collision::VoxelOctree::from_itk_image(image);
  // TODO: simplify mesh on flat surfaces?  Can save a lot for voxel conversions
  return voxels.to_mesh();
}

collision::Mesh to_marching_cubes_mesh(ImagePtr image) {
  collision::Mesh mesh;

  using MeshType = itk::Mesh<double, 3>;
  using MeshSourceType = itk::BinaryMask3DMeshSource<ImageType, MeshType>;
  auto mesh_source = MeshSourceType::New();
  mesh_source->SetObjectValue(1);
  mesh_source->SetInput(image);
  mesh_source->Update();
  auto itk_mesh = mesh_source->GetOutput();

  using CellType = MeshType::CellType;
  using TriangleType = itk::TriangleCell<CellType>;

  std::cout << "# nodes  = " << mesh_source->GetNumberOfNodes() << "\n"
            << "# cells  = " << mesh_source->GetNumberOfCells()  << "\n"
            << std::endl;

  std::cout << "# points = " << itk_mesh->GetNumberOfPoints() << "\n"
            << "# cells  = " << itk_mesh->GetNumberOfCells()  << "\n"
            << std::endl;

  mesh.vertices.resize(itk_mesh->GetNumberOfPoints());
  mesh.triangles.reserve(itk_mesh->GetNumberOfCells());

  const auto point_container = itk_mesh->GetPoints();
  if (point_container) {
    auto pts_iter = point_container->Begin();
    auto pts_end  = point_container->End();
    for (; pts_iter != pts_end; ++pts_iter) {
      auto point = pts_iter.Value();
      auto idx = pts_iter.Index();
      mesh.vertices[idx] = {point[0], point[1], point[2]};
    }
  }

  const auto cell_container = itk_mesh->GetCells();
  if (cell_container) {
    auto cell_iter = cell_container->Begin();
    auto cell_end  = cell_container->End();
    for (; cell_iter != cell_end; ++cell_iter) {
      auto *cell = cell_iter.Value();
      if (cell->GetNumberOfPoints() != 3 ||
#if ITK_VERSION_MAJOR < 5
          cell->GetType() != CellType::CellGeometry::TRIANGLE_CELL
#else
          cell->GetType() != itk::CellGeometryEnum::TRIANGLE_CELL
#endif
         )
      {
        throw std::runtime_error("mesh is not only triangles");
      }
      auto *tri = dynamic_cast<TriangleType*>(cell);

      auto tri_iter = tri->PointIdsBegin();
      auto tri_end  = tri->PointIdsEnd();
      collision::Mesh::Triangle triangle;
      for (size_t i = 0; tri_iter != tri_end; ++tri_iter, ++i) {
        triangle[i] = *tri_iter;
      }
      mesh.triangles.emplace_back(std::move(triangle));
    }
  }

  return mesh;
}

void save_mesh(const collision::Mesh &mesh, const std::string &fname,
               bool binary = true)
{
  std::cout << "Saving " << fname << std::endl;
  mesh.to_stl(fname, binary);
}

} // end of unnamed namespace

int main(int arg_count, char *arg_list[]) {
  CliParser parser;
  populate_parser(parser);
  parser.parse(arg_count, arg_list);

  std::vector<std::string> nrrd_files;
  nrrd_files.emplace_back(parser["nrrd-file"]);
  nrrd_files.insert(nrrd_files.end(),
                    parser.remaining().begin(), parser.remaining().end());
  bool binary = !parser.has("--ascii");

  int err = 0;
  for (auto &nrrd_file : nrrd_files) {
    auto out_file = nrrd_file + ".stl";

    std::cout << "Loading " << nrrd_file << std::endl;
    auto image = load_image(nrrd_file);

    collision::Mesh mesh;
    if (parser.has("--marching-cubes")) {
      mesh = to_marching_cubes_mesh(image);
    } else {
      mesh = to_box_mesh(image);
    }

    save_mesh(mesh, out_file, binary);
  }

  return err;
}
