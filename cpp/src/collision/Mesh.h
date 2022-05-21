#ifndef MESH_H
#define MESH_H

#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>

#include <memory>
#include <string>
#include <vector>

namespace cpptoml {
class table;
}

namespace collision {

/// represents a triangle mesh
// TODO: hide vertices and triangles behind getters and setters?
// TODO: cache the from_fcl and to_fcl shared pointers.  Should only do once.
// TODO: if vertices and/or triangles change, then null out the FCL pointer.
// TODO-   or do we want to modify the FCL object in-place?  This would cause
// TODO-   pulled instances of it to be updated rather than outdated.
// TODO: can I just use the FCL type as the storage instead of converting?
struct Mesh {
  using Triangle = fcl::Triangle;
  using Vertex = fcl::Vec3f;

  std::vector<Vertex> vertices;
  std::vector<Triangle> triangles;
  mutable std::string filename;

  bool empty() const { return vertices.empty() || triangles.empty(); }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static Mesh from_toml(std::shared_ptr<cpptoml::table> table);

  void to_stl(const std::string &fname, bool binary=true) const; // may set filename
  static Mesh from_stl(const std::string &fname);

  template <typename BV>
  static Mesh from_fcl(std::shared_ptr<fcl::BVHModel<BV>> model) {
    Mesh mesh;
    mesh.vertices =
        std::vector<Vertex>(model->vertices,
                            model->vertices + model->num_vertices);
    mesh.triangles =
        std::vector<Triangle>(model->tri_indices,
                              model->tri_indices + model->num_tris);
    return mesh;
  }

  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::BVHModel<BV>> to_fcl_model() const {
    auto model = std::make_shared<fcl::BVHModel<BV>>();
    if (!this->empty()) {
      model->beginModel(triangles.size(), vertices.size());
      model->addSubModel(vertices, triangles);
      model->endModel();
      model->computeLocalAABB();
    // TODO: how to get point clouds to work?
    // point cloud mode seg faults... does not work...
    // } else if (triangles.empty()) {
    //   // point cloud
    //   model->beginModel(0, vertices.size());
    //   model->endModel();
    //   model->addSubModel(vertices);
    //   model->computeLocalAABB();
    }
    return model;
  }

  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::CollisionObject> to_fcl_object() const {
    return std::make_shared<fcl::CollisionObject>(this->to_fcl_model<BV>());
  }

  /// Doesn ot compare with filename, just vertices and triangles
  bool equal(const Mesh &other) const {
    // TODO: check for reordering of vertices and triangles
    return std::equal(vertices.begin(), vertices.end(),
                      other.vertices.begin(), other.vertices.end(),
                      [](const Vertex &v1, const Vertex &v2) {
                        return v1.equal(v2);
                      })
        && std::equal(triangles.begin(), triangles.end(),
                      other.triangles.begin(), other.triangles.end(),
                      [](const Triangle &t1, const Triangle &t2) {
                        return t1[0] == t2[0]
                            && t1[1] == t2[1]
                            && t1[2] == t2[2];
                      });
        ;
  }

  /// Requires both equal triangles and equal filename
  bool operator==(const Mesh &other) const {
    return this->equal(other)
        && filename == other.filename
        ;
  }
};

inline std::ostream& operator<<(std::ostream &out, const Mesh::Triangle &tri) {
  out << "(" << tri[0] << ", " << tri[1] << ", " << tri[2] << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream &out, const Mesh &mesh) {
  auto print_vec = [&out](const auto &vec) {
    bool first = true;
    for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
      if (!first) { out << ", "; }
      out << vec[i];
      first = false;
    }
  };

  out << "Mesh{vertices=[";
  print_vec(mesh.vertices);
  out << "], triangles=[";
  print_vec(mesh.triangles);
  out << "], filename='" << mesh.filename << "'}";
  return out;
}

} // end of namespace collision

#endif // MESH_H
