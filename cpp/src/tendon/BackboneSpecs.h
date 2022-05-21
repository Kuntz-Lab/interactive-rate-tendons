#ifndef BACKBONE_SPECS_H
#define BACKBONE_SPECS_H

#include <memory>
#include <ostream>

namespace cpptoml {
class table;
}

namespace tendon {

/// Backbone specifications
struct BackboneSpecs {
  double L  = 0.2;   // length (meters)
  double dL = 0.005; // length discretization (meters)
  double ro = 0.01;  // outer backbone radius (meters)
  double ri = 0.0;   // inner backbone radius of the cavity (meters)
  double E  = 2.1e6; // Young's modulus (pascals = N/m^2)
  double nu = 0.3;   // Poisson's ratio: response in directions orthogonal to
                     // uniaxial stress.  Relates young's modulus to shear
                     // modulus with isotropic materials.

  bool operator==(const BackboneSpecs &other) const {
    return
      L  == other.L  &&
      dL == other.dL &&
      ro == other.ro &&
      ri == other.ri &&
      E  == other.E  &&
      nu == other.nu;
  }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static BackboneSpecs from_toml(std::shared_ptr<cpptoml::table> table);
};

inline std::ostream& operator<<(std::ostream& out, const BackboneSpecs &specs) {
  return out
    << "specs{"
       "L: "  << specs.L  << ", "
       "dL: " << specs.dL << ", "
       "ro: " << specs.ro << ", "
       "ri: " << specs.ri << ", "
       "E: "  << specs.E  << ", "
       "nu: " << specs.nu << "}";
}

} // end of namespace tendon

#endif // BACKBONE_SPECS_H
