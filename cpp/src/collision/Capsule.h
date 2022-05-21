#ifndef CAPSULE_H
#define CAPSULE_H

#include <collision/Point.h>
#include <collision/collision_primitives.h> // for interpolate() and closest_t()

#include <fcl/math/transform.h>

#include <iostream>
#include <memory>
#include <utility> // for std::pair

namespace cpptoml {
class table;
}

namespace fcl {
class Capsule;
class Transform3f;
class CollisionObject;
}

namespace collision {

struct Capsule {
  Point a;
  Point b;
  double r;

  bool operator==(const Capsule &other) const {
    return a == other.a && b == other.b && r == other.r;
  }

  Point interpolate(double t) const { return collision::interpolate(a, b, t); }
  double closest_t(const Point &p) const { return collision::closest_t(a, b, p); }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static Capsule from_toml(std::shared_ptr<cpptoml::table> table);

  std::pair<std::shared_ptr<fcl::Capsule>, fcl::Transform3f>
  to_fcl_model() const;

  std::shared_ptr<fcl::CollisionObject> to_fcl_object() const;
};

inline std::ostream& operator<<(std::ostream &out, const Capsule &c) {
  return out << "Capsule(" << c.a << ", " << c.b << ", " << "r=" << c.r << ")";
}

} // end of namespace collision

#endif // CAPSULE_H
