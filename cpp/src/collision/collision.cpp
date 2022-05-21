#include "collision/collision.h"

#include <fcl/collision.h>      // for fcl::collide()
#include <fcl/collision_data.h> // for fcl::CollisionRe{quest,sult}
#include <fcl/broadphase/broadphase.h>

namespace collision {

bool collides_self(const CapsuleSequence &seq) {
  // short-hand aliases
  auto &pts = seq.points;
  auto &r = seq.r;

  double dist_to_consider = 3.0 * r; // compare capsules this far away
  auto N = pts.size();

  if (N <= 2) { return false; }

  // Note: this could be made more accurate by taking the direction vector into
  // account.  For example, if the current capsule is pointing into the
  // previous capsule, then they collide.

  // accumulated L2 distances from beginning
  std::vector<double> acc_dists;
  acc_dists.reserve(N);
  double dist = 0.0;
  auto p_prev = pts[0];
  for (auto &p : pts) {
    dist += (p - p_prev).norm();
    acc_dists.emplace_back(dist);
    p_prev = p;
  }

  // for each capsule, compare against all distal capsules that have a backbone
  // distance more than a heuristic multiple of the capsule diameter.
  // That distance is dist_to_consider above
  for (size_t a = 0; a < N-3; ++a) {
    for (size_t b = a+2; b < N-1; ++b) {
      if (acc_dists[b] - acc_dists[a+1] < dist_to_consider) {
        continue; // skip
      }
      if (collides(seq[a], seq[b])) { // use capsule collision
        return true;
      }
    }
  }

  return false;
} // end of function collides_self(CapsuleSequence)

struct CollisionData {
  fcl::CollisionRequest request;
  fcl::CollisionResult result;
};


// used in collides() with BroadPhaseCollisionManager
bool fclCollisionCallback(fcl::CollisionObject* a,
                                 fcl::CollisionObject* b,
                                 void* data)
{
  CollisionData* cdata = static_cast<CollisionData*>(data);
  const fcl::CollisionRequest& request = cdata->request;
  fcl::CollisionResult& result = cdata->result;

  if(result.isCollision()) {
    return true;
  }

  fcl::collide(a, b, request, result);

  return result.isCollision();
}

bool collides(fcl::CollisionObject* a, fcl::CollisionObject* b) {
  fcl::CollisionRequest request; // default request
  fcl::CollisionResult result;
  fcl::collide(a, b, request, result);
  return result.isCollision();
}

bool collides(std::shared_ptr<fcl::CollisionObject> a,
                     std::shared_ptr<fcl::CollisionObject> b)
{
  return collides(a.get(), b.get());
}

bool collides(fcl::BroadPhaseCollisionManager* manager,
                     fcl::CollisionObject* obj)
{
  // fcl::BroadPhaseDynamicAABBTreeCollisionManager: good collision manager
  CollisionData data;
  manager->collide(obj, &data, fclCollisionCallback);
  return data.result.isCollision();
}

bool collides(std::shared_ptr<fcl::BroadPhaseCollisionManager> manager,
                     std::shared_ptr<fcl::CollisionObject> obj)
{
  return collides(manager.get(), obj.get());
}

bool collides(fcl::CollisionObject* obj,
                     fcl::BroadPhaseCollisionManager* manager)
{
  return collides(manager, obj);
}

bool collides(std::shared_ptr<fcl::CollisionObject> obj,
                     std::shared_ptr<fcl::BroadPhaseCollisionManager> manager)
{
  return collides(manager, obj);
}

bool collides(fcl::BroadPhaseCollisionManager* a,
                     fcl::BroadPhaseCollisionManager* b)
{
  CollisionData data;
  a->collide(b, &data, fclCollisionCallback);
  return data.result.isCollision();
}

bool collides(std::shared_ptr<fcl::BroadPhaseCollisionManager> a,
              std::shared_ptr<fcl::BroadPhaseCollisionManager> b)
{
  return collides(a.get(), b.get());
}

} // end of namespace collision
