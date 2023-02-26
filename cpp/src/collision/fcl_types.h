#ifndef FCL_TYPES_H
#define FCL_TYPES_H

// This file handles the challenges with includes and types that change over
// different FCL versions.

#include <fcl/config.h>
#if FCL_MINOR_VERSION == 5
#  include <fcl/BV/OBBRSS.h>
#  include <fcl/BVH/BVH_model.h>
#  include <fcl/broadphase/broadphase.h>
#  include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#  include <fcl/collision.h>      // for fcl::collide()
#  include <fcl/collision_data.h> // for fcl::CollisionRe{quest,sult}
#  include <fcl/data_types.h>
#  include <fcl/math/transform.h>
#  include <fcl/math/vec_3f.h>
#  include <fcl/octree.h>
#  include <fcl/shape/geometric_shape_to_BVH_model.h>
#  include <fcl/shape/geometric_shapes.h>
#elif FCL_MINOR_VERSION == 6
#  include <fcl/broadphase/broadphase_collision_manager.h>
#  include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#  include <fcl/common/types.h>  // for fcl::Vector3<S>
#  include <fcl/geometry/bvh/BVH_model.h>
#  include <fcl/geometry/geometric_shape_to_BVH_model.h>
#  include <fcl/geometry/octree/octree.h>
#  include <fcl/geometry/shape/capsule.h>
#  include <fcl/geometry/shape/sphere.h>
#  include <fcl/math/bv/OBBRSS.h>
#  include <fcl/math/triangle.h>
#  include <fcl/narrowphase/collision.h> // for fcl::collide()
#  include <fcl/narrowphase/collision_object.h>
#else
#  error "Unsupported FCL version"
#endif


#include <Eigen/Core>
#include <Eigen/Geometry>


namespace collision {
  namespace fcl {
    template <typename BV> using BVHModel = ::fcl::BVHModel<BV>;
    using Triangle = ::fcl::Triangle;

#if FCL_MINOR_VERSION == 5

    using FloatType = float;

    using BV = ::fcl::OBBRSS;
    using BroadPhaseCollisionManager = ::fcl::BroadPhaseCollisionManager;
    using Capsule = ::fcl::Capsule;
    using CollisionObject = ::fcl::CollisionObject;
    using CollisionRequest = ::fcl::CollisionRequest;
    using CollisionResult = ::fcl::CollisionResult;
    using DynamicAABBTreeCollisionManager = ::fcl::DynamicAABBTreeCollisionManager;
    using OcTree = ::fcl::OcTree;
    using Quaternion = ::fcl::Quaternion3f;
    using Sphere = ::fcl::Sphere;
    using Transform = ::fcl::Transform3f;
    using Vertex = ::fcl::Vec3f;

    inline Transform mktransform(const Eigen::Quaterniond &q, const Eigen::Vector3d &v) {
      return Transform{
        Quaternion(q.w(), q.x(), q.y(), q.z()),
        Vertex(v[0], v[1], v[2])
      };
    }

    inline Transform mktransform(const Eigen::Vector3d &v) {
      return Transform{Vertex{v[0], v[1], v[2]}};
    }

#elif FCL_MINOR_VERSION == 6

    using FloatType = double;

    using BV = ::fcl::OBBRSS<FloatType>;
    using BroadPhaseCollisionManager = ::fcl::BroadPhaseCollisionManager<FloatType>;
    using Capsule = ::fcl::Capsule<FloatType>;
    using CollisionObject = ::fcl::CollisionObject<FloatType>;
    using CollisionRequest = ::fcl::CollisionRequest<FloatType>;
    using CollisionResult = ::fcl::CollisionResult<FloatType>;
    using DynamicAABBTreeCollisionManager = ::fcl::DynamicAABBTreeCollisionManager<FloatType>;
    using OcTree = ::fcl::OcTree<FloatType>;
    using Quaternion = ::fcl::Quaternion<FloatType>;
    using Sphere = ::fcl::Sphere<FloatType>;
    using Transform = ::fcl::Transform3<FloatType>;
    using Vertex = ::fcl::Vector3<FloatType>;

    inline Transform mktransform(const Eigen::Quaterniond &q, const Eigen::Vector3d &v) {
      Transform t;
      t.rotate(q);
      t.translate(v);
      return t;
    }

    inline Transform mktransform(const Eigen::Vector3d &v) {
      Transform t;
      t.translate(v);
      return t;
    }

#endif

  } // end of namespace collision::fcl
} // end of namespace collision


#endif // FCL_TYPES_H
