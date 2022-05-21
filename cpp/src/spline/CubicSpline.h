#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H

#include "Cubic.h"

namespace spline {

/**
 * Represents a cubic spline constructed using boundary conditions (i.e., the y
 * values and the dydx values at each end).
 */
template <typename XType, typename YType>
class CubicSpline {
public:
  using x_type = XType;
  using y_type = YType;

  /// Construct a cubic spline from two tripples (x, y, dydx)
  CubicSpline(XType x1, YType y1, YType y1p, XType x2, YType y2, YType y2p)
    : _x1(x1)
  {
    _c.c0 = y1;
    _c.c1 = y1p;
    XType dx = x2 - x1;
    _c.c3 = (2*y1 - 2*y2 + dx*(y1p + y2p)) / (dx*dx*dx);
    _c.c2 = (y2p - y1p - 3 * _c.c3 * dx * dx) / (2 * dx);
  }

  auto operator() (XType x) const {
    XType dx = x - _x1;
    return _c(dx);
  }

  /// derivative
  CubicSpline<XType, YType> deriv() const {
    CubicSpline<XType, YType> copy = *this;
    copy._c = copy._c.deriv();
    return copy;
  }

private:
  XType _x1;
  Cubic<YType> _c;
};

} // end of namespace spline

#endif // CUBIC_SPLINE_H
