#include "collision/CapsuleSequence.h"
#include <cpptoml/toml_conversions.h>
#include <util/eigen_ops.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace E = Eigen;
using MatX = E::MatrixXd;
using Vec = E::Vector3d;

namespace collision {

namespace {

E::Matrix3d rotation_matrix_from_z_axis(const Vec &vec) {
  auto q = util::quat_from_zaxis(vec);
  return q.toRotationMatrix();
}

/// unit circle in the x-y plane centered at the origin
MatX unit_circle(unsigned int n_circle_pts) {
  MatX circle{3, n_circle_pts};
  for (unsigned int i = 0; i < n_circle_pts; i++) {
    double angle = double(2 * M_PI * i) / double(n_circle_pts);
    circle(0, i) = std::cos(angle);
    circle(1, i) =  std::sin(angle);
    circle(2, i) = 0.0;
  }
  return circle;
}

// Append a circle of points to the mesh as vertices
void add_circle_to_mesh(Mesh &mesh, const MatX &circle) {
  for (int col = 0; col < circle.cols(); col++) {
    mesh.vertices.emplace_back(
        circle(0, col), circle(1, col), circle(2, col));
  }
}

// Function to add the faces between two circles to a mesh's triangles
// - first_circle_idx: first point index for the first circle
// - second_circle_idx: first point index for the second circle
void add_cylinder_faces_to_mesh(Mesh &mesh, unsigned int n_circle_pts,
                                int first_circle_idx, int second_circle_idx)
{
  for (unsigned int j = 0; j < n_circle_pts; j++) {
    int a = first_circle_idx + j;
    int b = first_circle_idx + ((j + 1) % n_circle_pts);
    int c = second_circle_idx + j;
    int d = second_circle_idx + ((j + 1) % n_circle_pts);
    mesh.triangles.emplace_back(a, b, c);
    mesh.triangles.emplace_back(b, d, c);
  }
}

// Function to add the triangles connecting a circle to a point
// - circle_idx: first point index for the circle
// - tip_idx: point index for connective point
// - above: true means cone tip is above the circle (using right-hand rule)
void add_cone_faces_to_mesh(Mesh &mesh, unsigned int n_circle_pts,
                            int circle_idx, int tip_idx, bool above)
{
  for (unsigned int i = 0; i < n_circle_pts; i++) {
    int a = circle_idx + i;
    int b = circle_idx + ((i + 1) % n_circle_pts);
    if (!above) { std::swap(a, b); }
    mesh.triangles.emplace_back(a, b, tip_idx);
  }
}

} // end of unnamed namespace

bool CapsuleSequence::operator==(const CapsuleSequence &other) const {
  return points == other.points && r == other.r;
}

std::shared_ptr<cpptoml::table> CapsuleSequence::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("capsule_sequence", tbl);
  tbl->insert("radius", this->r);
  if (!this->points.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &p : this->points) {
      auto sub_tbl = cpptoml::make_table();
      sub_tbl->insert("point", cpptoml::to_toml(p));
      tbl_array->push_back(sub_tbl);
    }
    tbl->insert("points", tbl_array);
  }
  return container;
}

CapsuleSequence CapsuleSequence::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // check if the container is inside of tbl, otherwise assume we are already in
  if (tbl->contains("capsule_sequence")) {
    auto container = tbl->get("capsule_sequence")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'capsule_sequence': not a table");
    }
    tbl = container;  // pull from the container
  }

  CapsuleSequence seq;

  // radius
  auto r = tbl->get("radius")->as<double>();
  if (!r) {
    throw cpptoml::parse_exception("Wrong type detected for 'r'");
  }
  seq.r = r->get();

  // points
  if (tbl->contains("points")) {
    auto points = tbl->get("points")->as_table_array();
    if (!points) {
      throw cpptoml::parse_exception("Wrong type detected for 'points'");
    }
    for (auto &sub_tbl : points->get()) {
      auto point = sub_tbl->get("point")->as_array();
      if (!point) {
        throw cpptoml::parse_exception("Wrong type detected in point");
      }
      seq.points.emplace_back(cpptoml::to_point(point));
    }
  }

  return seq;
}

Mesh CapsuleSequence::to_mesh(unsigned int n_circle_pts, bool add_spherical_caps) const {
  if (n_circle_pts < 3) {
    throw std::invalid_argument("Cannot make a circle with fewer than 3 points");
  }

  if (points.size() <= 1 ||
      (points.size() == 2 &&
       (points[1] - points[0]).norm()
          <= std::numeric_limits<double>::epsilon()))
  {
    return Mesh{};
  }

  Mesh mesh;
  MatX r_circle = r * unit_circle(n_circle_pts);

  // Add the circles for the cylinders
  for (size_t i = 1; i < points.size(); i++) {
    auto R = rotation_matrix_from_z_axis(points[i] - points[i-1]);
    MatX rotated_circle = R * r_circle;
    if (i == 1) { // add the base only once
      add_circle_to_mesh(mesh, rotated_circle.colwise() + points[0]);
    }
    add_circle_to_mesh(mesh, rotated_circle.colwise() + points[i]);
  }

  // Add the triangles for the cylinders
  for (size_t i = 1; i < points.size(); i++) {
    int prev_circle_idx = (i - 1) * n_circle_pts;
    int cur_circle_idx = i * n_circle_pts;
    add_cylinder_faces_to_mesh(mesh, n_circle_pts,
                               prev_circle_idx, cur_circle_idx);
  }

  if (add_spherical_caps) { // sphere ends
    const double dphi = M_PI / double(n_circle_pts);
    double phi = M_PI / 2.0;

    // first hemisphere
    auto R = rotation_matrix_from_z_axis(points[1] - points[0]);
    int prev_circle_idx = 0;
    for (unsigned int i = 1; i < std::ceil(n_circle_pts / 2.0) - 1; i++) {
      MatX new_r_circle = (std::sin(phi - dphi*i) * r_circle).colwise()
                        - Vec{0, 0, r*std::cos(phi - dphi*i)};
      int cur_circle_idx = mesh.vertices.size();
      add_circle_to_mesh(mesh, (R * new_r_circle).colwise() + points[0]);
      add_cylinder_faces_to_mesh(mesh, n_circle_pts,
                                 cur_circle_idx, prev_circle_idx);
      prev_circle_idx = cur_circle_idx;
    }
    Vec first_htop = points[0] - points[1];
    first_htop = points[0] + first_htop * r / first_htop.norm();
    mesh.vertices.emplace_back(first_htop[0], first_htop[1], first_htop[2]);
    add_cone_faces_to_mesh(mesh, n_circle_pts, prev_circle_idx,
                           mesh.vertices.size() - 1, false);

    // second hemisphere
    R = rotation_matrix_from_z_axis(points.back() - points[points.size() - 2]);
    prev_circle_idx = (points.size() - 1) * n_circle_pts;
    for (unsigned int i = 1; i < std::ceil(n_circle_pts / 2.0) - 1; i++) {
      MatX new_r_circle = (std::sin(phi - dphi*i) * r_circle).colwise()
                        + Vec{0, 0, r*std::cos(phi - dphi*i)};
      int cur_circle_idx = mesh.vertices.size();
      add_circle_to_mesh(mesh, (R * new_r_circle).colwise() + points.back());
      add_cylinder_faces_to_mesh(mesh, n_circle_pts,
                                 prev_circle_idx, cur_circle_idx);
      prev_circle_idx = cur_circle_idx;
    }
    Vec last_htop = points.back() - points[points.size() - 2];
    last_htop = points.back() + last_htop * r / last_htop.norm();
    mesh.vertices.emplace_back(last_htop[0], last_htop[1], last_htop[2]);
    add_cone_faces_to_mesh(mesh, n_circle_pts, prev_circle_idx,
                           mesh.vertices.size() - 1, true);

  } else { // flat ends

    // first flat end
    mesh.vertices.emplace_back(points[0][0], points[0][1], points[0][2]);
    add_cone_faces_to_mesh(mesh, n_circle_pts, 0, mesh.vertices.size() - 1,
                           false);

    // last flat end
    mesh.vertices.emplace_back(points.back()[0], points.back()[1], points.back()[2]);
    int last_circle_idx = (points.size() - 1) * n_circle_pts;
    add_cone_faces_to_mesh(mesh, n_circle_pts, last_circle_idx,
                           mesh.vertices.size() - 1, true);
  }

  return mesh;
}

} // end of namespace collision
