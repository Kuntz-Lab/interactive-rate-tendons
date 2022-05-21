#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <collision/Capsule.h>
#include <collision/Mesh.h>
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/VoxelOctree.h>
#include <collision/collision.h>

#include <memory>
#include <ostream>

namespace cpptoml {
class table;
}

namespace fcl {
  class CollisionObject;
  class DynamicAABBTreeCollisionManager;
  class BroadPhaseCollisionManager;
}

namespace motion_planning {

class Environment {
public:
  using Point            = collision::Point;
  using Sphere           = collision::Sphere;
  using Capsule          = collision::Capsule;
  using Mesh             = collision::Mesh;
  using CollisionManager = fcl::DynamicAABBTreeCollisionManager;

  const std::vector<Point>&   points()   const { return _points;   }
  const std::vector<Sphere>&  spheres()  const { return _spheres;  }
  const std::vector<Capsule>& capsules() const { return _capsules; }
  const std::vector<Mesh>&    meshes()   const { return _meshes;   }

  Environment();
  Environment(const Environment& other);
  Environment(Environment&& other);
  Environment& operator= (const Environment& env);
  Environment& operator= (Environment&& env);

  void clear() {
    clear<Point>();
    clear<Sphere>();
    clear<Capsule>();
    clear<Mesh>();
  }

  template <typename Shape>
  void clear() {
    get_vec<Shape>().clear();
    _dirty = true;
  }

  template <typename Shape, typename... Args>
  void emplace_back(Args&&... args) {
    get_vec<Shape>().emplace_back(std::forward<Args>(args)...);
    _dirty = true;
  }

  template <typename Shape>
  void push_back(const Shape& shape) {
    get_vec<Shape>().push_back(shape);
    _dirty = true;
  }

  template <typename Shape>
  void push_back(Shape&& shape) {
    get_vec<Shape>().push_back(std::forward<Shape>(shape));
    _dirty = true;
  }

  template <typename Shape>
  void set_vec(std::vector<Shape> &&vec) {
    get_vec<Shape>() = std::forward<std::vector<Shape>>(vec);
    _dirty = true;
  }

  template <typename Shape>
  void set_vec(const std::vector<Shape> &vec) {
    get_vec<Shape>() = vec;
    _dirty = true;
  }

  // to make collision checks thread-safe, you can call this first
  // to continue to be thread-safe, you must call this after any changes.
  void setup() const {
    check_update();
  }

  bool collides(fcl::CollisionObject *obj) const;
  bool collides(fcl::BroadPhaseCollisionManager* manager) const;
  bool collides(const collision::CapsuleSequence &caps) const;
  bool collides(const Environment &other) const;

  bool operator==(const Environment &other) const {
    return _points   == other._points
        && _spheres  == other._spheres
        && _capsules == other._capsules
        && _meshes   == other._meshes
           ;
  }

  /** Voxelize using the reference voxel object size and dimensions
   *
   * Does not yet support meshes, so will throw std::logic_error if there are
   * any meshes in this class.
   *
   * @param reference: a reference VoxelOctree from which to copy the
   *     dimensions and workspace limits.
   * @param dilate: how much to dilate the objects in the environment.  For
   *     example, you could pass in the radius of the tendon robot so that you
   *     can motion plan only against the robot's backbone centerline (thus
   *     improving the motion planning's computational efficiency).
   *     Default value is 0.0 -- meaning no dilation.  A negative value does
   *     not make sense.
   */
  std::shared_ptr<collision::VoxelOctree> voxelize(
      const collision::VoxelOctree &reference) const;
  std::shared_ptr<collision::VoxelOctree> voxelize(
      const collision::VoxelOctree &reference, double dilate) const;

  bool collides(const Point &object) const {
    // TODO: implement
    (void)object;
    throw std::logic_error("collision between Point and fcl broadphase not implemented");
  }

  bool collides(const Sphere  &s) const { return shape_collides(s); }
  bool collides(const Capsule &c) const { return shape_collides(c); }
  bool collides(const Mesh    &m) const { return shape_collides(m); }

  std::shared_ptr<cpptoml::table> to_toml() const;
  static Environment from_toml(std::shared_ptr<cpptoml::table> tbl);

private:
  template <typename Shape> std::vector<Shape>& get_vec();
  template <typename Shape> const std::vector<Shape>& get_vec() const;

  template <typename Shape>
  bool shape_collides(const Shape &shape) const {
    auto fcl_obj = shape.to_fcl_object();
    return collides(fcl_obj.get());
  }

  void check_update() const;
  void update_broadphase() const;

private:
  std::vector<collision::Point>   _points;
  std::vector<collision::Sphere>  _spheres;
  std::vector<collision::Capsule> _capsules;
  std::vector<collision::Mesh>    _meshes;

  // Cached collision detection stuff
  mutable bool _dirty = false;  // flag signaling we need to update cache
  mutable std::vector<std::shared_ptr<fcl::CollisionObject>> _objects;
  mutable std::shared_ptr<CollisionManager> _broadphase;
};

template <> inline std::vector<Environment::Point  >&
Environment::get_vec<Environment::Point  >() {
  return _points;
}

template <> inline std::vector<Environment::Sphere >&
Environment::get_vec<Environment::Sphere >() {
  return _spheres;
}

template <> inline std::vector<Environment::Capsule>&
Environment::get_vec<Environment::Capsule>() {
  return _capsules;
}

template <> inline std::vector<Environment::Mesh   >&
Environment::get_vec<Environment::Mesh   >() {
  return _meshes;
}

template <> inline const std::vector<Environment::Point  >&
Environment::get_vec<Environment::Point  >() const {
  return _points;
}

template <> inline const std::vector<Environment::Sphere >&
Environment::get_vec<Environment::Sphere >() const {
  return _spheres;
}

template <> inline const std::vector<Environment::Capsule>&
Environment::get_vec<Environment::Capsule>() const {
  return _capsules;
}

template <> inline const std::vector<Environment::Mesh   >&
Environment::get_vec<Environment::Mesh   >() const {
  return _meshes;
}

inline std::ostream& operator<<(std::ostream &out, const Environment &env) {
  auto print_vec = [&out](const auto &vec) {
    bool first = true;
    for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
      if (!first) { out << ", "; }
      out << vec[i];
      first = false;
    }
  };

  out << "Environment{points=[";
  print_vec(env.points());
  out << "], spheres=[";
  print_vec(env.spheres());
  out << "], capsules=[";
  print_vec(env.capsules());
  out << "], meshes=[";
  print_vec(env.meshes());
  out << "]}";
  return out;
}

} // end of namespace motion_planning

#endif // ENVIRONMENT_H
