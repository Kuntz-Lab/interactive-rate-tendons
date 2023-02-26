#include "Sphere.h"

#include "collision/fcl_types.h"
#include "cpptoml/toml_conversions.h"

namespace collision {

std::shared_ptr<cpptoml::table> Sphere::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("sphere", tbl);
  tbl->insert("radius", this->r);
  tbl->insert("center", cpptoml::to_toml(this->c));
  return container;
}

Sphere Sphere::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // try to grab the "sphere" container, otherwise assume we're already inside
  if (tbl->contains("sphere")) {
    auto container = tbl->get("sphere")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'sphere': not a table");
    }
    tbl = container;
  }

  auto r = tbl->get("radius")->as<double>();
  auto center = tbl->get("center")->as_array();
  if (!(r && center)) {
    throw cpptoml::parse_exception("wrong type detected");
  }
  return Sphere{cpptoml::to_point(center), r->get()};
}

std::pair<std::shared_ptr<::collision::fcl::Sphere>, ::collision::fcl::Transform>
Sphere::to_fcl_model() const
{
  auto fcl_sphere = std::make_shared<::collision::fcl::Sphere>(r);
  auto trans = ::collision::fcl::mktransform(c);
  return {fcl_sphere, trans};
}

std::shared_ptr<::collision::fcl::CollisionObject> Sphere::to_fcl_object() const {
  auto [fcl_sphere, transform] = this->to_fcl_model();
  return std::make_shared<::collision::fcl::CollisionObject>(fcl_sphere, transform);
}
} // end of namespace collision
