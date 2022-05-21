#ifndef POINT_H
#define POINT_H

#include <Eigen/Core>

#include <iostream>

namespace collision {

using Point = Eigen::Vector3d;

inline std::ostream& operator<<(std::ostream &out, const Point &p) {
  return out << "Point{" << p[0] << ", " << p[1] << ", " << p[2] << "}";
}

} // end of namespace collision

#endif // POINT_H
