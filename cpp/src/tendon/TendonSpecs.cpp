#include "TendonSpecs.h"

#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

namespace tendon {

namespace {

/** Returns degree of polynomial from coefficient array
 *
 * @param coef: polynomial coefficients: p(t) = coef[0] + coef[1]*t + ...
 * @param eps: if (abs(coef[i]) <= eps) then it is considered to be zero
 *
 * @return degree of the polynomial (0 = constant, 1 = linear, etc.)
 */
size_t poly_degree(const Eigen::VectorXd &coef, double eps = 0.0) {
  if (coef.size() == 0) { return 0; }
  for (size_t i = coef.size() - 1; i > 0; i--) {
    if (std::abs(coef[i]) > eps) {
      return i;
    }
  }
  return 0;
}

} // end of unnamed namespace

size_t TendonSpecs::r_degree(double eps) const { return poly_degree(D, eps); }
size_t TendonSpecs::theta_degree(double eps) const { return poly_degree(C, eps); }

std::shared_ptr<cpptoml::table> TendonSpecs::to_toml() const {
  auto tbl = cpptoml::make_table();
  tbl->insert("C", cpptoml::to_toml(C));
  tbl->insert("D", cpptoml::to_toml(D));
  tbl->insert("max_tension", max_tension);
  tbl->insert("min_length", min_length);
  tbl->insert("max_length", max_length);
  return tbl;
}

TendonSpecs TendonSpecs::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }
  auto C = tbl->get("C")->as_array();
  auto D = tbl->get("D")->as_array();
  auto max_tension = tbl->get("max_tension")->as<double>();
  auto min_length = tbl->get("min_length")->as<double>();
  auto max_length = tbl->get("max_length")->as<double>();
  if (!(C && D && max_tension && min_length && max_length)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }
  return {cpptoml::to_vector(C), cpptoml::to_vector(D), max_tension->get(),
          min_length->get(), max_length->get()};
}

std::ostream& operator<<(std::ostream &out, const TendonSpecs &specs) {
  auto print_vec = [&out](const Eigen::VectorXd &vec) {
    bool first = true;
    for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
      if (!first) { out << ", "; }
      out << vec[i];
      first = false;
    }
  };

  out << "TendonSpecs{C=[";
  print_vec(specs.C);
  out << "], D=[";
  print_vec(specs.D);
  out << "], max_tension=" << specs.max_tension
      << ", min_length=" << specs.min_length
      << ", max_length=" << specs.max_length
      << "}";
  return out;
}

} // end of namespace tendon
