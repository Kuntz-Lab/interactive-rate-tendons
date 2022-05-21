#ifndef COLLISION_PRIMITIVES_H
#define COLLISION_PRIMITIVES_H

#include <collision/Point.h>

#include <algorithm> // for std::min() and std::max()
#include <limits>    // for std::numeric_limits<double>::epsilon()
#include <tuple>     // for std::tuple<>

namespace collision {

/** Interpolate between a and b.  0.0 returns a, 1.0 returns b.
 *
 * If you pass a negative t or a t bigger than 1.0, then it will return the
 * extrapolated value.
 */
inline Point interpolate(const Point &a, const Point &b, double t) {
  return a + (b - a) * t;
}

/** Finds the t parameter to pass into interpolate for the closest point to p
 *
 * Creates an infinite line from a to b extending in both directions.  Then,
 * return the t parameter to pass to interpolate that gives the closest point
 * from p to this line.
 *
 * If you want it to be the closest t value in the line segment from a to b,
 * just truncate the value to be between 0.0 and 1.0.  For example:
 *
 *   double t = std::max(0.0, std::min(1.0, closest_t(a, b, p)));
 *
 * closest_t_segment() is provided as such a convenience function.
 */
inline double closest_t(const Point &a, const Point &b, const Point &p) {
  const double eps = std::numeric_limits<double>::epsilon();

  const Point diff = b - a;
  const double diff_squared = diff.dot(diff);
  if (diff_squared <= eps*eps) { // a and b are basically the same point
    return 0.0;
  }

  return diff.dot(p - a) / diff_squared;
}

/// Like closest_t() but for a line segment rather than an infinite line
inline double closest_t_segment(const Point &a, const Point &b, const Point &p) {
  return std::max(0.0, std::min(1.0, closest_t(a, b, p)));
}

std::tuple<double, double>
closest_st_segment(const Point &A, const Point &B,
                   const Point &C, const Point &D);

/** Returns true if segment AB intersects the axis-aligned box
 *
 * The box is with corner endpoints C and D
 *
 * Implementation adapted from "Intersection of a Line and a Box" from
 *   https://www.geometrictools.com/Documentation/IntersectionLineBox.pdf
 */
inline bool segment_aabox_intersect(const Point &A, const Point &B,
                                    const Point &C, const Point &D)
{
  const Point  AB   = B - A;
  const double len  = AB.norm() / 2;             // line length from center
  const Point  U    = AB / (2 * len);            // line unit vector direction
  const Point  Uabs = U.cwiseAbs();              // unit step dim lenths
  const Point  P    = (A + B) / 2 - (D + C) / 2; // line center rel to box
  const Point  ext  = (D - C).cwiseAbs() / 2;    // box extents (dims / 2)
  const Point  UxP  = U.cross(P).cwiseAbs();
  const Point  Pabs = P.cwiseAbs();

  bool intersects =
      // segment-specific tests
         Pabs[0] > ext[0] + len*Uabs[0]
      || Pabs[1] > ext[1] + len*Uabs[1]
      || Pabs[2] > ext[2] + len*Uabs[2]
      // line-specific tests
      || UxP[0] > ext[1]*Uabs[2] + ext[2]*Uabs[1]
      || UxP[1] > ext[2]*Uabs[0] + ext[0]*Uabs[2]
      || UxP[2] > ext[0]*Uabs[1] + ext[1]*Uabs[0];

  return !intersects;
}

} // end of namespace collision

#endif // COLLISION_PRIMITIVES_H
