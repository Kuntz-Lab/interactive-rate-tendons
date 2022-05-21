#include "TendonResult.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace E = Eigen;

namespace tendon {

void TendonResult::rotate_z(double theta) {
  E::Matrix3d rot = E::AngleAxisd(theta, E::Vector3d::UnitZ())
                    .toRotationMatrix();
  for (auto &pval : this->p) { pval = rot * pval; }
  for (auto &Rval : this->R) { Rval = rot * Rval; }
}

std::shared_ptr<cpptoml::table> TendonResult::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("tendon_result", tbl);

  if (!t.empty()) {
    tbl->insert("t", cpptoml::to_toml(t));
  }
  if (!p.empty()) {
    auto tbl_arr = cpptoml::make_table_array();
    for (auto &point : p) {
      auto sub_tbl = cpptoml::make_table();
      sub_tbl->insert("point", cpptoml::to_toml(point));
      tbl_arr->push_back(sub_tbl);
    }
    tbl->insert("p", tbl_arr);
  }
  if (!R.empty()) {
    tbl->insert("R", cpptoml::to_toml(R));
  }
  tbl->insert("L", L);
  if (!L_i.empty()) {
    tbl->insert("L_i", cpptoml::to_toml(L_i));
  }
  tbl->insert("u_i", cpptoml::to_toml(u_i));
  tbl->insert("v_i", cpptoml::to_toml(v_i));
  tbl->insert("u_f", cpptoml::to_toml(u_f));
  tbl->insert("v_f", cpptoml::to_toml(v_f));
  tbl->insert("converged", converged);

  return container;
}

TendonResult TendonResult::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  auto container = tbl->get("tendon_result")->as_table();
  if (!container) {
    throw cpptoml::parse_exception(
        "Wrong type detected for 'tendon_result': not a table");
  }

  TendonResult result;

  // t
  if (container->contains("t")) {
    auto t = container->get("t")->as_array();
    if (!t) {
      throw cpptoml::parse_exception("Wrong type detected for 't'");
    }
    result.t = cpptoml::to_stdvec<double>(t);
  }

  // p
  if (container->contains("p")) {
    auto p = container->get("p")->as_table_array();
    if (!p) {
      throw cpptoml::parse_exception("Wrong type detected for 'p'");
    }
    for (auto sub_tbl : p->get()) {
      auto point = sub_tbl->get("point")->as_array();
      if (!point) {
        throw cpptoml::parse_exception("Wrong point type detected");
      }
      result.p.emplace_back(cpptoml::to_point(point));
    }
  }

  // R
  if (container->contains("R")) {
    auto R = container->get("R")->as_table_array();
    if (!R) {
      throw cpptoml::parse_exception("Wrong type detected for 'R'");
    }
    for (auto sub_tbl : R->get()) {
      result.R.emplace_back(cpptoml::to_matrix(sub_tbl));
    }
  }

  // L
  auto L = container->get("L")->as<double>();
  if (!L) {
    throw cpptoml::parse_exception("Wrong type detected for 'L'");
  }
  result.L = L->get();

  // L_i
  if (container->contains("L_i")) {
    auto L_i = container->get("L_i")->as_array();
    if (!L_i) {
      throw cpptoml::parse_exception("Wrong type detected for 'L_i'");
    }
    result.L_i = cpptoml::to_stdvec<double>(L_i);
  }

  // u_i
  if (container->contains("u_i")) {
    auto u_i = container->get("u_i")->as_array();
    if (!u_i) {
      throw cpptoml::parse_exception("Wrong type detected for 'u_i'");
    }
    result.u_i = cpptoml::to_point(u_i);
  }

  // v_i
  if (container->contains("v_i")) {
    auto v_i = container->get("v_i")->as_array();
    if (!v_i) {
      throw cpptoml::parse_exception("Wrong type detected for 'v_i'");
    }
    result.v_i = cpptoml::to_point(v_i);
  }

  // u_f
  if (container->contains("u_f")) {
    auto u_f = container->get("u_f")->as_array();
    if (!u_f) {
      throw cpptoml::parse_exception("Wrong type detected for 'u_f'");
    }
    result.u_f = cpptoml::to_point(u_f);
  }

  // v_f
  if (container->contains("v_f")) {
    auto v_f = container->get("v_f")->as_array();
    if (!v_f) {
      throw cpptoml::parse_exception("Wrong type detected for 'v_f'");
    }
    result.v_f = cpptoml::to_point(v_f);
  }

  // converged
  if (container->contains("converged")) {
    auto converged = container->get("converged")->as<bool>();
    if (!converged) {
      throw cpptoml::parse_exception("Wrong type detected for 'converged'");
    }
    result.converged = converged->get();
  }

  return result;
}

} // end of namespace tendon
