#include "Mesh.h"

#include "collision/stl_io.h"

#include "cpptoml/toml_conversions.h"

#include <string>
#include <vector>

namespace collision {

// TODO: add optional argument of relative directory
// TODO: convert filename to be relative to the given relative directory
// TODO: have the relative directory default to the current directory
// TODO: use boost::filesystem to do the path resolving
std::shared_ptr<cpptoml::table> Mesh::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("mesh", tbl);
  if (!this->filename.empty()) {
    // output just the filename
    tbl->insert("filename", this->filename);
  } else {
    // output the triangles

    auto vec3_to_table = [](std::string name, auto vec3) {
      auto tbl = cpptoml::make_table();
      auto arr = cpptoml::make_array();
      arr->push_back(vec3[0]);
      arr->push_back(vec3[1]);
      arr->push_back(vec3[2]);
      tbl->insert(name, arr);
      return tbl;
    };

    // vertices
    if (!this->vertices.empty()) {
      auto tbl_vertices = cpptoml::make_table_array();
      for (auto &v : this->vertices) {
        tbl_vertices->push_back(vec3_to_table("vertex", v));
      }
      tbl->insert("vertices", tbl_vertices);
    }

    // triangles
    if (!this->triangles.empty()) {
      auto tbl_triangles = cpptoml::make_table_array();
      for (auto &tri : this->triangles) {
        tbl_triangles->push_back(vec3_to_table("indices", tri));
      }
      tbl->insert("triangles", tbl_triangles);
    }
  }
  return container;
}

// TODO: add argument specifying the relative directory
// TODO: make filename resolution be relative to the given directory
// TODO: have the default for the relative directory be the current directory
// TODO: use boost::filesystem to handle paths
// TODO: allow ros resource manager string types? (like package:// and file://)
Mesh Mesh::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // try to grab the "mesh" container, otherwise assume we're already inside
  if (tbl->contains("mesh")) {
    auto container = tbl->get("mesh")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'mesh': not a table");
    }
    tbl = container;  // pull from the container instead
  }

  if (tbl->contains("filename")) {
    // load from file
    auto filename = tbl->get("filename")->as<std::string>();
    if (!filename) {
      throw cpptoml::parse_exception("Wrong type detected for filename");
    }
    return Mesh::from_stl(filename->get());
  } else {
    // load from triangles and vertices in the table
    Mesh mesh;

    // vertices
    if (tbl->contains("vertices")) {
      auto vertices = tbl->get("vertices")->as_table_array();
      if (!vertices) {
        throw cpptoml::parse_exception("Wrong type detected for 'vertices'");
      }
      for (auto &sub_tbl : vertices->get()) {
        auto vertex_array = sub_tbl->get("vertex")->as_array();
        if (!vertex_array) {
          throw cpptoml::parse_exception("Wrong type detected in vertex");
        }
        auto vertex = cpptoml::to_point(vertex_array);
        mesh.vertices.emplace_back(vertex[0], vertex[1], vertex[2]);
      }
    }

    // triangles
    if (tbl->contains("triangles")) {
      auto triangles = tbl->get("triangles")->as_table_array();
      if (!triangles) {
        throw cpptoml::parse_exception("Wrong type detected for 'triangles'");
      }
      for (auto &sub_tbl : triangles->get()) {
        auto indices_array = sub_tbl->get("indices")->as_array();
        if (!indices_array) {
          throw cpptoml::parse_exception("Wrong type detected in triangle "
                                         "indices");
        }
        auto indices = cpptoml::to_stdvec<int64_t>(indices_array);
        if (indices.size() < 3) {
          throw std::out_of_range("indices array too short (shorter than 3)");
        }
        if (indices.size() > 3) {
          throw cpptoml::parse_exception("indices array too long (longer than "
                                         "3)");
        }
        mesh.triangles.emplace_back(indices[0], indices[1], indices[2]);
      }
    }

    return mesh;
  }
}

void Mesh::to_stl(const std::string &fname, bool binary) const {
  if (binary) {
    write_stl_file_binary(fname, vertices, triangles);
  } else {
    write_stl_file_ascii(fname, vertices, triangles);
  }

  if (this->filename.empty()) {
    this->filename = fname;
  }
}

Mesh Mesh::from_stl(const std::string &fname) {
  using vec = std::vector<float>;
  using ivec = std::vector<size_t>;
  vec points, normals;
  ivec triangles, solid_ranges;

  read_stl_file(fname.c_str(), points, normals, triangles, solid_ranges);

  Mesh mesh;
  mesh.filename = fname;
  mesh.triangles.reserve(triangles.size() / 3);
  mesh.vertices.reserve(points.size() / 3);

  for (size_t tri_idx = 0; tri_idx < triangles.size(); tri_idx += 3) {
    mesh.triangles.emplace_back(
        triangles[tri_idx],
        triangles[tri_idx + 1],
        triangles[tri_idx + 2]
        );
  }

  for (size_t pnt_idx = 0; pnt_idx < points.size(); pnt_idx += 3) {
    mesh.vertices.emplace_back(
        points[pnt_idx],
        points[pnt_idx + 1],
        points[pnt_idx + 2]
        );
  }

  return mesh;
}

} // end of namespace collision
