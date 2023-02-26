#ifndef COLLISION_H
#define COLLISION_H

#include <collision/collision_primitives.h>
#include <collision/Point.h>
#include <collision/Sphere.h>
#include <collision/Capsule.h>
#include <collision/CapsuleSequence.h>
#include <collision/Mesh.h>
#include <collision/VoxelOctree.h>
#include <collision/fcl_types.h>

#include <memory>


namespace collision {

// ==================
//   SELF COLLISION  
// ==================

// CapsuleSequence
bool collides_self(const CapsuleSequence &seq);

// ==================
//   PAIR COLLISION  
// ==================

// FCL
bool collides(::collision::fcl::CollisionObject* a,
              ::collision::fcl::CollisionObject* b);
bool collides(std::shared_ptr<::collision::fcl::CollisionObject> a,
              std::shared_ptr<::collision::fcl::CollisionObject> b);
bool collides(::collision::fcl::BroadPhaseCollisionManager* manager,
              ::collision::fcl::CollisionObject* obj);
bool collides(std::shared_ptr<::collision::fcl::BroadPhaseCollisionManager> manager,
              std::shared_ptr<::collision::fcl::CollisionObject> obj);
bool collides(::collision::fcl::CollisionObject* obj,
              ::collision::fcl::BroadPhaseCollisionManager* manager);
bool collides(std::shared_ptr<::collision::fcl::CollisionObject> obj,
              std::shared_ptr<::collision::fcl::BroadPhaseCollisionManager> manager);
bool collides(::collision::fcl::BroadPhaseCollisionManager* a,
              ::collision::fcl::BroadPhaseCollisionManager* b);
bool collides(std::shared_ptr<::collision::fcl::BroadPhaseCollisionManager> a,
              std::shared_ptr<::collision::fcl::BroadPhaseCollisionManager> b);

// Point
inline bool collides(const Point &a, const Point &b);

// Sphere
inline bool collides(const Sphere &s, const Point &p);
inline bool collides(const Point &p, const Sphere &s);
inline bool collides(const Sphere &a, const Sphere &b);

// Capsule
inline bool collides(const Capsule &c, const Point &p);
inline bool collides(const Point &p, const Capsule &c);
inline bool collides(const Capsule &c, const Sphere &s);
inline bool collides(const Sphere &s, const Capsule &c);
inline bool collides(const Capsule &c1, const Capsule &c2);

// CapsuleSequence
template <typename T> // piggy back on the collides(Capsule, *) functions
inline bool collides(const CapsuleSequence &seq, const T &val);
inline bool collides(const Point &p, const CapsuleSequence &seq);
inline bool collides(const Sphere &s, const CapsuleSequence &seq);
inline bool collides(const Capsule &c, const CapsuleSequence &seq);

// Mesh
// not implemented
//inline bool collides(const Mesh &m, const Point &p);
//inline bool collides(const Point &p, const Mesh &m);
//inline bool collides(const Mesh &m, const CapsuleSequence &seq);
//inline bool collides(const CapsuleSequence &seq, const Mesh &m);
inline bool collides(const Mesh &m, const Sphere &s);
inline bool collides(const Sphere &s, const Mesh &m);
inline bool collides(const Mesh &m, const Capsule &c);
inline bool collides(const Capsule &c, const Mesh &m);
inline bool collides(const Mesh &a, const Mesh &b);

// VoxelOctree
inline bool collides(const VoxelOctree &a, const VoxelOctree &b);



} // end of namespace collision

#include "collision.hxx"

#endif // COLLISION_H
