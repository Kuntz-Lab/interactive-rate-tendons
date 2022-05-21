#ifndef SPHERE_H
#define SPHERE_H

#include <collision/Point.h>

#include <fcl/BV/OBBRSS.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>

#include <exception>
#include <iostream>
#include <memory>

namespace cpptoml {
class table;
}

namespace fcl {
class Sphere;
class Transform3f;
}

namespace collision {

struct Sphere {
  Point c;
  double r;

  bool operator==(const Sphere &other) const {
    return c == other.c && r == other.r;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static Sphere from_toml(std::shared_ptr<cpptoml::table> table);

  std::pair<std::shared_ptr<fcl::Sphere>, fcl::Transform3f>
  to_fcl_model() const;

  std::shared_ptr<fcl::CollisionObject> to_fcl_object() const;

  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::BVHModel<BV>> to_fcl_mesh_model() const
  {
    auto [sphere, transform] = to_fcl_model();
    auto mesh = std::make_shared<fcl::BVHModel<BV>>();
    fcl::generateBVHModel(*mesh, *sphere, transform, 64);
    return mesh;
  }

  template <typename BV=fcl::OBBRSS>
  std::shared_ptr<fcl::CollisionObject> to_fcl_mesh_object() const
  {
    auto [sphere, transform] = to_fcl_model();
    auto mesh = std::make_shared<fcl::BVHModel<BV>>();
    fcl::generateBVHModel(*mesh, *sphere, fcl::Transform3f{}, 64);
    auto obj = std::make_shared<fcl::CollisionObject>(mesh, transform);
    return obj;
  }
};

inline std::ostream& operator<<(std::ostream &out, const Sphere &s) {
  return out << "Sphere(" << s.c << ", r=" << s.r << ")";
}

} // end of namespace collision

#endif // SPHERE_H
