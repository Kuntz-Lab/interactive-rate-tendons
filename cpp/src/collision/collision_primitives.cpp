#include "collision/collision_primitives.h"
#include "collision/Point.h"

#include <algorithm> // for std::min() and std::max()
#include <limits>    // for std::numeric_limits<double>::epsilon()
#include <tuple>     // for std::tuple<>

namespace collision {

std::tuple<double, double>
closest_st_segment(const Point &A, const Point &B,
                   const Point &C, const Point &D)
{
  const double eps = std::numeric_limits<double>::epsilon();
  const double eps_squared = eps*eps;

  double s = 0.0;
  double t = 0.0;

  const Point AB = B - A;
  const Point CD = D - C;
  const double a = AB.dot(AB);     // non-negative
  const double c = CD.dot(CD);     // non-negative

  // define some helper functions
  // (faster than function calls because uses already computed values)
  auto closest_AB_s = [&A, &AB, a, eps_squared](const Point &P) {
    if (a <= eps_squared) {
      return 0.0;
    }
    return AB.dot(P - A) / a;
  };
  auto closest_CD_t = [&C, &CD, c, eps_squared](const Point &P) {
    if (c <= eps_squared) {
      return 0.0;
    }
    return CD.dot(P - C) / c;
  };
  auto bound = [](double t) { return std::max(0.0, std::min(1.0, t)); };

  if (a <= eps_squared) {          // A and B are basically the same point
    return std::tuple{0.0, bound(closest_CD_t(A))};
  }

  if (c <= eps_squared) {          // C and D are basically the same point
    return std::tuple{bound(closest_AB_s(C)), 0.0};
  }

  const Point AC = C - A;
  const double b = AB.dot(CD);
  const double d = AC.dot(AB);
  const double e = AC.dot(CD);
  const double denom = std::max(0.0, a*c - b*b);  // non-negative

  if (denom <= eps_squared) {      // AB and CD are basically parallel
    // cycle through each point and return the best one (three is enough)
    t = closest_CD_t(A);
    if (0.0 <= t && t <= 1.0) {
      return std::tuple{0.0, t};
    }
    t = closest_CD_t(B);
    if (0.0 <= t && t <= 1.0) {
      return std::tuple{1.0, t};
    }
    s = closest_AB_s(C);
    if (0.0 <= s && s <= 1.0) {
      return std::tuple{s, 0.0};
    }
    // non-overlapping parallel lines
    // return best endpoint distance
    const Point AD = D - A;
    const Point BC = C - B;
    const Point BD = D - B;
    const double ac2 = AC.dot(AC);
    const double ad2 = AD.dot(AD);
    const double bc2 = BC.dot(BC);
    const double bd2 = BD.dot(BD);
    if (ac2 <= ad2 && ac2 <= bc2 && ac2 <= bd2) {
      return std::tuple{0.0, 0.0};
    }
    if (ad2 <= bc2 && ad2 <= bd2) {
      return std::tuple{0.0, 1.0};
    }
    if (bc2 <= bd2) {
      return std::tuple{1.0, 0.0};
    }
    return std::tuple{1.0, 1.0};
  }

  // now, segments are long and not parallel
  s = (c*d - b*e) / denom;
  t = (b*d - a*e) / denom;

  if (0.0 <= t && t <= 1.0) {
    return std::tuple{bound(s), t};
  }
  if (t < 0.0) {
    return std::tuple{bound(-c / a), 0.0};
  }
  // else (t > 1.0)
  return std::tuple{bound((b - c) / a), 1.0};
}

} // end of namespace collision
