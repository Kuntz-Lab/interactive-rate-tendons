#ifndef CUBIC_SPLINE_SEQUENCE_H
#define CUBIC_SPLINE_SEQUENCE_H

#include "CubicSpline.h"

#include <iterator>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace spline {

template <typename XType, typename YType>
class CubicSplineSequence {
public:
  using x_type = XType;
  using y_type = YType;

  CubicSplineSequence(const std::vector<XType> &x,
                      const std::vector<YType> &y,
                      const std::vector<YType> &yp)
    : _x(x)
  {
    if (x.size() != y.size() || x.size() != yp.size()) {
      std::ostringstream message_builder;
      message_builder
        << "Given vectors must be the same size (x.size = " << x.size();
      if (x.size() != y.size()) {
        message_builder << ", y.size = " << y.size() << ")";
      } else {
        message_builder << ", yp.size = " << yp.size() << ")";
      }
      throw std::out_of_range(message_builder.str());
    }
    if (x.size() < 2) {
      throw std::out_of_range("Must have at least two points to create cubic splines");
    }

    for (size_t i = 1; i < x.size(); i++) {
      if (x[i-1] >= x[i]) {
        throw std::invalid_argument("x vector must be monotonically increasing");
      }
      _splines.emplace_back(x[i-1], y[i-1], yp[i-1], x[i], y[i], yp[i]);
    }
  }

  YType operator() (XType x) const {
    // let the first and last splines extend
    if (x > _x.back()) {
      return _splines.back()(x);
    }

    // find the correct spline such that x is in [spine.x1, spline.x2)
    // std::lower_bound() returns the first element A such that x <= A
    auto it = std::lower_bound(_x.begin(), _x.end(), x);
    if (it == _x.begin()) {
      return _splines[0](x);
    } else {
      return _splines[std::distance(_x.begin(), it) - 1](x);
    }
  }

  /// derivative
  CubicSplineSequence<XType, YType> deriv() const {
    CubicSplineSequence<XType, YType> copy = *this;
    for (auto &spline : copy._splines) {
      spline = spline.deriv();
    }
    return copy;
  }

  XType x0() const { return _x.front(); }
  XType xn() const { return _x.back(); }

private:
  std::vector<XType> _x;
  std::vector<CubicSpline<XType, YType>> _splines;
};

} // end of namespace spline

#endif // CUBIC_SPLINE_SEQUENCE_H
