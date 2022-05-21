#ifndef OCTOMAP_WRAP_H
#define OCTOMAP_WRAP_H

#include <fcl/octree.h>
#include <fcl/collision.h>      // for fcl::collide()
#include <fcl/collision_data.h> // for fcl::CollisionRe{quest,sult}

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <memory>  // for std::shared_ptr

namespace collision {

class OctomapWrap {
public:
  size_t Nx() const { return _Nx; }
  size_t Ny() const { return _Ny; }
  size_t Nz() const { return _Nz; }

  explicit OctomapWrap(double resolution)
    : _Nx(1/resolution), _Ny(1/resolution), _Nz(1/resolution)
    , _data(new octomap::OcTree(resolution))
    , _collision_object(new fcl::OcTree(_data))
  {
    _data->useBBXLimit(true);
    octomap::point3d min {0.0, 0.0, 0.0};
    octomap::point3d max {1.0, 1.0, 1.0};
    _data->setBBXMin(min);
    _data->setBBXMax(max);
  }

  explicit OctomapWrap(const VoxelOctree &vtree)
    : _Nx(vtree.Nx()), _Ny(vtree.Ny()), _Nz(vtree.Nz())
    , _data(new octomap::OcTree(
          std::min(vtree.dx(),
                   std::min(vtree.dy(),
                            vtree.dz()))))
    , _collision_object(new fcl::OcTree(_data))
  {
    _data->useBBXLimit(true);
    auto ll = vtree.lower_left();
    auto ur = vtree.upper_right();
    octomap::point3d min {float(ll[0]), float(ll[1]), float(ll[2])};
    octomap::point3d max {float(ur[0]), float(ur[1]), float(ur[2])};
    _data->setBBXMin(min);
    _data->setBBXMax(max);

    // set each voxel as occupied or not
    vtree.visit_voxels(
      [this, &vtree](size_t ix, size_t iy, size_t iz, bool is_occupied) {
        const bool lazy = true;
        auto center = vtree.voxel_center(ix, iy, iz);
        this->add_point(center[0], center[1], center[2], is_occupied, lazy);
      });

    // update cache of tree hierarchy
    this->update();
  }

  OctomapWrap(const OctomapWrap &other)
    : _Nx(other._Nx), _Ny(other._Ny), _Nz(other._Nz)
    , _data(new octomap::OcTree(*other._data))
    , _collision_object(new fcl::OcTree(_data))
  {
    _collision_object->setUserData(other._collision_object->getUserData());
    //_collision_object->setTransform(other._collision_object->getTransform());
    //_collision_object->setCostDensity(other._collision_object->getCostDensity());
  }

  OctomapWrap(OctomapWrap &&other) // move
    : _Nx(other._Nx), _Ny(other._Ny), _Nz(other._Nz)
    , _data(std::move(other._data))
    , _collision_object(std::move(other._collision_object))
  {}

  size_t memoryUsage() const { return _data->memoryUsage(); }
  size_t nblocks() const { return _data->size(); }

  void add_point(float x, float y, float z, bool occupied = true, bool lazy = false) {
    _data->updateNode(octomap::point3d{x, y, z}, occupied, lazy);
  }

  void update() {
    _data->updateInnerOccupancy();
  }

  void add_sphere(double x, double y, double z, double r) {
    // Algorithm:
    // 1. add the sphere center to the voxelization
    // 2. grow the sphere center, asking each voxel center to see if it is inside
    //    the sphere

    // 1. add center
    add_point(x, y, z);

    // 2. grow the sphere center, asking each voxel center to see if it is inside
    //    the sphere
    auto is_in_sphere = [x, y, z, r](double _x, double _y, double _z) {
      double dx = x - _x;
      double dy = y - _y;
      double dz = z - _z;
      bool is_in = dx*dx + dy*dy + dz*dz <= r*r;
      return is_in;
    };

    auto idx_to_voxel_center = [this] (size_t _ix, size_t _iy, size_t _iz) {
      float x = this->dx() * (_ix + 0.5);
      float y = this->dy() * (_iy + 0.5);
      float z = this->dz() * (_iz + 0.5);
      return octomap::point3d{x, y, z};
    };

    // just check all of them
    for (size_t ix = 0; ix < _Nx; ix++) {
      for (size_t iy = 0; iy < _Ny; iy++) {
        for (size_t iz = 0; iz < _Nz; iz++) {
          auto p = idx_to_voxel_center(ix, iy, iz);
          _data->updateNode(p, is_in_sphere(p.x(), p.y(), p.z()));
        }
      }
    }
  }

  bool collides(const OctomapWrap &other) const {
    fcl::CollisionRequest request; // default request
    fcl::CollisionResult result;
    fcl::CollisionObject a(_collision_object);
    fcl::CollisionObject b(other._collision_object);
    fcl::collide(&a, &b, request, result);
    return result.isCollision();
  }

  void set_xlim(double xmin, double xmax) {
    if (xmin >= xmax) {
      throw std::length_error("xlimits must be positive in size");
    }
    auto min = _data->getBBXMin();
    auto max = _data->getBBXMax();
    min.x() = xmin;
    max.x() = xmax;
    _data->setBBXMin(min);
    _data->setBBXMax(max);
    _Nx = (xmax - xmin) / dx();
  }

  void set_ylim(double ymin, double ymax) {
    if (ymin >= ymax) {
      throw std::length_error("ylimits must be positive in size");
    }
    auto min = _data->getBBXMin();
    auto max = _data->getBBXMax();
    min.y() = ymin;
    max.y() = ymax;
    _data->setBBXMin(min);
    _data->setBBXMax(max);
    _Ny = (ymax - ymin) / dy();
  }

  void set_zlim(double zmin, double zmax) {
    if (zmin >= zmax) {
      throw std::length_error("zlimits must be positive in size");
    }
    auto min = _data->getBBXMin();
    auto max = _data->getBBXMax();
    min.z() = zmin;
    max.z() = zmax;
    _data->setBBXMin(min);
    _data->setBBXMax(max);
    _Nz = (zmax - zmin) / dz();
  }

  std::pair<double, double> xlim() const {
    return { _data->getBBXMin().x(), _data->getBBXMax().x() };
  }
  std::pair<double, double> ylim() const {
    return { _data->getBBXMin().y(), _data->getBBXMax().y() };
  }
  std::pair<double, double> zlim() const {
    return { _data->getBBXMin().z(), _data->getBBXMax().z() };
  }

  // size of cells
  double dx() const { return _data->getResolution(); }
  double dy() const { return _data->getResolution(); }
  double dz() const { return _data->getResolution(); }

private:
  size_t _Nx = 0;
  size_t _Ny = 0;
  size_t _Nz = 0;
  std::shared_ptr<octomap::OcTree> _data;
  std::shared_ptr<fcl::OcTree> _collision_object;
}; // end of class OctomapWrap

} // end of namespace collision

#endif // OCTOMAP_WRAP_H
