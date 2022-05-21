#ifndef COLLISION_HXX
#define COLLISION_HXX

#include <collision/collision_primitives.h>
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/Capsule.h>
#include <collision/CapsuleSequence.h>
#include <collision/Mesh.h>
#include <collision/VoxelOctree.h>

namespace collision {


// ==================
//   SELF COLLISION  
// ==================

//
// CapsuleSequence
//

// in collision.cpp
// bool collides_self(const CapsuleSequence &seq);

// ==================
//   PAIR COLLISION  
// ==================

//
// FCL
//

// defiend in collision.cpp
// bool collides(fcl::CollisionObject* a, fcl::CollisionObject* b);
// bool collides(std::shared_ptr<fcl::CollisionObject> a,
//               std::shared_ptr<fcl::CollisionObject> b);
// bool collides(fcl::BroadPhaseCollisionManager* manager,
//               fcl::CollisionObject* obj);
// bool collides(std::shared_ptr<fcl::BroadPhaseCollisionManager> manager,
//               std::shared_ptr<fcl::CollisionObject> obj);
// bool collides(fcl::CollisionObject* obj,
//               fcl::BroadPhaseCollisionManager* manager);
// bool collides(std::shared_ptr<fcl::CollisionObject> obj,
//               std::shared_ptr<fcl::BroadPhaseCollisionManager> manager);
// bool collides(fcl::BroadPhaseCollisionManager* a,
//               fcl::BroadPhaseCollisionManager* b);
// bool collides(std::shared_ptr<fcl::BroadPhaseCollisionManager> a,
//               std::shared_ptr<fcl::BroadPhaseCollisionManager> b);


//
// Point
//

inline bool collides(const Point &a, const Point &b) {
  return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}


//
// Sphere
//

inline bool collides(const Sphere &s, const Point &p) {
  Point diff = s.c - p;
  return diff.dot(diff) <= (s.r * s.r);
}

inline bool collides(const Point &p, const Sphere &s) {
  return collides(s, p);
}

inline bool collides(const Sphere &a, const Sphere &b) {
  return collides(Sphere{a.c, a.r + b.r}, b.c);
}


//
// Capsule
//

inline bool collides(const Capsule &c, const Point &p) {
  auto t = closest_t_segment(c.a, c.b, p);
  auto closest = interpolate(c.a, c.b, t);
  return collides(Sphere{closest, c.r}, p);
}

inline bool collides(const Point &p, const Capsule &c) {
  return collides(c, p);
}

inline bool collides(const Capsule &c, const Sphere &s) {
  // dilated capsule against point
  return collides(Capsule{c.a, c.b, c.r + s.r}, s.c);
}

inline bool collides(const Sphere &s, const Capsule &c) {
  return collides(c, s);
}

inline bool collides(const Capsule &c1, const Capsule &c2) {
  auto [s, t] = closest_st_segment(c1.a, c1.b, c2.a, c2.b);
  auto closest_1 = interpolate(c1.a, c1.b, s);
  auto closest_2 = interpolate(c2.a, c2.b, t);
  const bool val = collides(Sphere{closest_1, c1.r + c2.r}, closest_2);
  return val;
}


//
// CapsuleSequence
//

// piggy back on the collides(Capsule, *) functions
template <typename T>
inline bool collides(const CapsuleSequence &seq, const T &val) {
  for (size_t i = 0; i < seq.size(); i++) {
    if (collides(seq[i], val)) {
      return true;
    }
  }
  return false;
}

inline bool collides(const Point &p, const CapsuleSequence &seq) {
  return collides(seq, p);
}

inline bool collides(const Sphere &s, const CapsuleSequence &seq) {
  return collides(seq, s);
}

inline bool collides(const Capsule &c, const CapsuleSequence &seq) {
  return collides(seq, c);
}


//
// Mesh
//

// not implemented
//inline bool collides(const Mesh &m, const Point &p) {
//
//}

// not implemented
//inline bool collides(const Point &p, const Mesh &m) {
//
//}

inline bool collides(const Mesh &m, const Sphere &s) {
  if (m.empty()) {
    return false;
  }
  return collides(m.to_fcl_object(), s.to_fcl_object());
}

inline bool collides(const Sphere &s, const Mesh &m) {
  return collides(m, s);
}

inline bool collides(const Mesh &m, const Capsule &c) {
  if (m.empty()) {
    return false;
  }
  return collides(m.to_fcl_object(), c.to_fcl_object());
}

inline bool collides(const Capsule &c, const Mesh &m) {
  return collides(m, c);
}

// not implemented
//inline bool collides(const Mesh &m, const CapsuleSequence &seq) {
//
//}

// not implemented
//inline bool collides(const CapsuleSequence &seq, const Mesh &m) {
//  return collides(m, seq);
//}

inline bool collides(const Mesh &a, const Mesh &b) {
  if (a.empty() || b.empty()) {
    return false;
  }
  return collides(a.to_fcl_object(), b.to_fcl_object());
}


//
// VoxelOctree
//

inline bool collides(const VoxelOctree &a, const VoxelOctree &b) {
  return a.collides(b);
}

} // end of namespace collision

#endif // COLLISION_HXX
