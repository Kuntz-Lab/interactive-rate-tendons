#include "Sphere.h"

#include "cpptoml/toml_conversions.h"

#include <fcl/shape/geometric_shapes.h> // for fcl::Sphere
#include <fcl/math/transform.h> // for Transform3f

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

std::pair<std::shared_ptr<fcl::Sphere>, fcl::Transform3f>
Sphere::to_fcl_model() const
{
  auto fcl_sphere = std::make_shared<fcl::Sphere>(r);
  fcl::Transform3f trans(fcl::Vec3f(c[0], c[1], c[2]));
  return {fcl_sphere, trans};
}

std::shared_ptr<fcl::CollisionObject> Sphere::to_fcl_object() const {
  auto [fcl_sphere, transform] = this->to_fcl_model();
  return std::make_shared<fcl::CollisionObject>(fcl_sphere, transform);
}
} // end of namespace collision
