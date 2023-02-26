#include "collision/Capsule.h"
#include <collision/fcl_types.h>
#include <cpptoml/toml_conversions.h>
#include <util/eigen_ops.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace E = Eigen;

namespace collision {

std::shared_ptr<cpptoml::table> Capsule::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("capsule", tbl);
  tbl->insert("radius", this->r);
  tbl->insert("a", cpptoml::to_toml(this->a));
  tbl->insert("b", cpptoml::to_toml(this->b));
  return container;
}

Capsule Capsule::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // try to grab the "capsule" container, otherwise assume we're already inside
  if (tbl->contains("capsule")) {
    auto container = tbl->get("capsule")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'capsule': not a table");
    }
    tbl = container; // pull from the container instead
  }

  auto r = tbl->get("radius")->as<double>();
  auto a = tbl->get("a")->as_array();
  auto b = tbl->get("b")->as_array();
  if (!(r && a && b)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }
  return {cpptoml::to_point(a), cpptoml::to_point(b), r->get()};
}

std::pair<std::shared_ptr<::collision::fcl::Capsule>, ::collision::fcl::Transform>
Capsule::to_fcl_model() const
{
  auto direction = this->b - this->a;
  auto center = this->a + (direction / 2.0);
  auto length = direction.norm();
  auto capsule = std::make_shared<::collision::fcl::Capsule>(this->r, length);
  auto e_quat = util::quat_from_zaxis(direction);
  auto transform = ::collision::fcl::mktransform(e_quat, center);
  return {capsule, transform};
}

std::shared_ptr<::collision::fcl::CollisionObject> Capsule::to_fcl_object() const {
  auto [fcl_capsule, transform] = this->to_fcl_model();
  return std::make_shared<::collision::fcl::CollisionObject>(fcl_capsule, transform);
}

} // end of namespace collision
