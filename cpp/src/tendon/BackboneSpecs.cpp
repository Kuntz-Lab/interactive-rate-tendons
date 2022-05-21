#include "BackboneSpecs.h"

#include <cpptoml/toml_conversions.h>

namespace tendon {

std::shared_ptr<cpptoml::table> BackboneSpecs::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("backbone_specs", tbl);
  tbl->insert("length" , L );
  tbl->insert("length_discretization", dL);
  tbl->insert("ro", ro);
  tbl->insert("ri", ri);
  tbl->insert("E" , E );
  tbl->insert("nu", nu);
  return container;
}

BackboneSpecs BackboneSpecs::from_toml(std::shared_ptr<cpptoml::table> table) {
  if (!table) { throw std::invalid_argument("null table given"); }

  auto container = table->get("backbone_specs")->as_table();
  if (!container) {
    throw cpptoml::parse_exception(
        "Wrong type for 'backbone_specs': not a table");
  }

  auto L  = container->get("length")->as<double>();
  auto dL = container->get("length_discretization")->as<double>();
  auto ro = container->get("ro")->as<double>();
  auto ri = container->get("ri")->as<double>();
  auto E  = container->get("E") ->as<double>();
  auto nu = container->get("nu")->as<double>();
  if (!(L && dL && ro && ri && E && nu)) {
    throw cpptoml::parse_exception("Wrong type for one of the BackboneSpecs");
  }
  return BackboneSpecs{L->get(), dL->get(), ro->get(), ri->get(), E->get(),
                       nu->get()};
}

} // end of namespace tendon
