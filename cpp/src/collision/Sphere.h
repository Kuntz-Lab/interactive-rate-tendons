#ifndef SPHERE_H
#define SPHERE_H

#include <collision/Point.h>
#include <collision/fcl_types.h>

#include <exception>
#include <iostream>
#include <memory>

namespace cpptoml {
class table;
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

  std::pair<std::shared_ptr<::collision::fcl::Sphere>, ::collision::fcl::Transform>
  to_fcl_model() const;

  std::shared_ptr<::collision::fcl::CollisionObject> to_fcl_object() const;

  template <typename BV=::collision::fcl::BV>
  std::shared_ptr<::collision::fcl::BVHModel<BV>> to_fcl_mesh_model() const
  {
    auto [sphere, transform] = to_fcl_model();
    auto mesh = std::make_shared<::collision::fcl::BVHModel<BV>>();
    ::fcl::generateBVHModel(*mesh, *sphere, transform, 64);
    return mesh;
  }

  template <typename BV=::collision::fcl::BV>
  std::shared_ptr<::collision::fcl::CollisionObject> to_fcl_mesh_object() const
  {
    auto [sphere, transform] = to_fcl_model();
    auto mesh = std::make_shared<::collision::fcl::BVHModel<BV>>();
    ::fcl::generateBVHModel(*mesh, *sphere, ::collision::fcl::Transform{}, 64);
    auto obj = std::make_shared<::collision::fcl::CollisionObject>(mesh, transform);
    return obj;
  }
};

inline std::ostream& operator<<(std::ostream &out, const Sphere &s) {
  return out << "Sphere(" << s.c << ", r=" << s.r << ")";
}

} // end of namespace collision

#endif // SPHERE_H
