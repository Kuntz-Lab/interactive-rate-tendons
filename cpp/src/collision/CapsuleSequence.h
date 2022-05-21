#ifndef CAPSULE_SEQUENCE_H
#define CAPSULE_SEQUENCE_H

#include <collision/Capsule.h>
#include <collision/Mesh.h>
#include <collision/Point.h>
#include <collision/collision_primitives.h> // for interpolate() and closest_t()

#include <fcl/BV/OBBRSS.h>

#include <iostream>
#include <memory>
#include <vector>

namespace cpptoml {
class table;
}

namespace collision {

/// A sequence of capsules with constant radius
struct CapsuleSequence {
  std::vector<Point> points;
  double r;

  /// Get the i'th capsule (no bounds checking)
  Capsule operator[] (size_t i) const {
    return Capsule{points[i], points[i+1], r};
  }

  /// Get the i'th capsule (with bounds checking)
  Capsule at(size_t i) const {
    return Capsule{points.at(i), points.at(i+1), r};
  }

  /// Returns the number of capsules represented
  size_t size() const { return std::max<size_t>(1, points.size()) - 1; }

  /** interpolation is 0.0 <= t <= (points.size() - 1.0)
   *
   * With three points, you have two capsules, and 0.0 <= t <= 2.0
   *
   * Note, you can extrapolate using this function by giving numbers outside of
   * the range.  If you give a negative number, it will extrapolate from the
   * first capsule.  If you give a number larger than (points.size() - 1), then
   * the last capsule is used.
   */
  Point interpolate(double t) const {
    size_t i = std::floor(std::max(0.0, t));
    // between 0 and points.size() - 2 with underflow protection
    i = std::max<size_t>(2, std::min<size_t>(i+2, points.size())) - 2;
    return collision::interpolate(points[i], points[i-1], t);
  }

  /** Gives the closest t value to the point
   *
   * Note: this is not efficient as it will check each capsule for distance and
   * return the smallest one.  If you just want to know if it collides, use
   * collides() function instead.
   *
   * Note: does not extend to extrapolation range, will keep within the
   * interpolation range.
   */
  double closest_t(const Point &p) const {
    double t         = 0.0;
    double dist2     = 0.0;  // distance squared

    // set t and dist2 from the k'th capsule
    auto set_dist = [&t, &dist2, &p, this](size_t k) {
      t = k + closest_t_segment(this->points[k], this->points[k+1], p);
      auto closest = collision::interpolate(this->points[k], this->points[k+1], t);
      dist2 = closest.dot(closest);
    };

    // assume the first one is the shortest
    set_dist(0);
    double t_min     = t;
    double dist2_min = dist2;

    // try to find a shorter one
    for (size_t i = 1; i < points.size() - 1; i++) {
      set_dist(i);
      if (dist2 < dist2_min) {
        dist2_min = dist2;
        t_min = t;
      }
    }

    return t_min;
  }

  bool operator==(const CapsuleSequence &other) const;
  std::shared_ptr<cpptoml::table> to_toml() const;
  static CapsuleSequence from_toml(std::shared_ptr<cpptoml::table> table);

  Mesh to_mesh(unsigned int n_circle_pts = 16, bool add_spherical_caps = true) const;

  template <typename BV=fcl::OBBRSS, typename... Args>
  auto to_fcl_mesh_model(Args&&... args) const {
    Mesh mesh = to_mesh(std::forward<Args>(args)...);
    return mesh.to_fcl_model<BV>();
  }

  template <typename BV=fcl::OBBRSS, typename... Args>
  auto to_fcl_mesh_object(Args&&... args) const {
    Mesh mesh = to_mesh(std::forward<Args>(args)...);
    return mesh.to_fcl_object<BV>();
  }
};

inline std::ostream& operator<<(std::ostream &out, const CapsuleSequence &seq) {
  out << "CapsuleSequence(";
  for (auto &p : seq.points) {
    out << p << ", ";
  }
  out << "r=" << seq.r << ")";
  return out;
}

} // end of namespace collision

#endif // CAPSULE_SEQUENCE_H
