#include "Environment.h"
#include "cpptoml/cpptoml.h"
#include "cpptoml/toml_conversions.h"

#include <fcl/broadphase/broadphase.h>


namespace motion_planning {

Environment::Environment()
  : _points()
  , _spheres()
  , _capsules()
  , _meshes()
  , _dirty(true)
  , _objects()
  , _broadphase(std::make_shared<Environment::CollisionManager>())
{ }

// copy constructor
Environment::Environment(const Environment& other)
  : _points(other._points)
  , _spheres(other._spheres)
  , _capsules(other._capsules)
  , _meshes(other._meshes)
  , _dirty(true)
  , _objects()
  , _broadphase(std::make_shared<Environment::CollisionManager>())
{ }

// move constructor
Environment::Environment(Environment&& other)
  : _points(std::move(other._points))
  , _spheres(std::move(other._spheres))
  , _capsules(std::move(other._capsules))
  , _meshes(std::move(other._meshes))
  , _dirty(other._dirty)
  , _objects(std::move(other._objects))
  , _broadphase(std::move(other._broadphase))
{
  other._dirty = true;
}

// copy assignment
Environment& Environment::operator= (const Environment& env) {
  return *this = Environment(env);
}

// move assignment
Environment& Environment::operator= (Environment&& env) {
  _points     = std::move(env._points);
  _spheres    = std::move(env._spheres);
  _capsules   = std::move(env._capsules);
  _meshes     = std::move(env._meshes);
  _dirty      = env._dirty;
  _objects    = std::move(env._objects);
  _broadphase = std::move(env._broadphase);
  env._dirty  = true;
  return *this;
}

std::shared_ptr<collision::VoxelOctree> Environment::voxelize(
      const collision::VoxelOctree &reference) const
{
  if (!_meshes.empty()) {
    throw std::logic_error("Meshes are not supported yet for voxelization");
  }

  auto voxels = std::make_shared<collision::VoxelOctree>(reference.empty_copy());
  for (auto &p : _points  ) { voxels->add(p); }
  for (auto &s : _spheres ) { voxels->add(s); }
  for (auto &c : _capsules) { voxels->add(c); }
  return voxels;
}

std::shared_ptr<collision::VoxelOctree> Environment::voxelize(
      const collision::VoxelOctree &reference, double dilate) const
{
  if (!_meshes.empty()) {
    throw std::logic_error("Meshes are not supported yet for voxelization");
  }
  if (dilate < 0.0) {
    throw std::invalid_argument("Negative dilation value given");
  }

  // handle dilation
  Environment dummy;
  if (dilate > 0.0) {
    for (auto &p : _points) {
      dummy._spheres.emplace_back(collision::Sphere{p, dilate});
    }
    for (auto &s : _spheres) {
      dummy._spheres.emplace_back(collision::Sphere{s.c, s.r + dilate});
    }
    for (auto &c : _capsules) {
      dummy._capsules.emplace_back(collision::Capsule{c.a, c.b, c.r + dilate});
    }
  }

  return dummy.voxelize(reference);
}

std::shared_ptr<cpptoml::table> Environment::to_toml() const {
  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("environment", tbl);

  // points
  if (!this->_points.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &p : this->_points) {
      auto sub_tbl = cpptoml::make_table();
      sub_tbl->insert("point", cpptoml::to_toml(p));
      tbl_array->push_back(sub_tbl);
    }
    tbl->insert("points", tbl_array);
  }

  // spheres
  if (!this->_spheres.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &s : this->_spheres) {
      tbl_array->push_back(s.to_toml());
    }
    tbl->insert("spheres", tbl_array);
  }

  // capsules
  if (!this->_capsules.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &c : this->_capsules) {
      tbl_array->push_back(c.to_toml());
    }
    tbl->insert("capsules", tbl_array);
  }

  // meshes
  if (!this->_meshes.empty()) {
    auto tbl_array = cpptoml::make_table_array();
    for (auto &m : this->_meshes) {
      tbl_array->push_back(m.to_toml());
    }
    tbl->insert("meshes", tbl_array);
  }

  return container;
}

Environment Environment::from_toml(std::shared_ptr<cpptoml::table> tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }

  // try to grab the "environment" container, otherwise assume we're already inside
  if (tbl->contains("environment")) {
    auto container = tbl->get("environment")->as_table();
    if (!container) {
      throw cpptoml::parse_exception(
          "Wrong type detected for 'environment': not a table");
    }
    tbl = container;
  }

  Environment env;

  // points
  if (tbl->contains("points")) {
    auto points   = tbl->get("points")  ->as_table_array();
    if (!points) {
      throw cpptoml::parse_exception("Wrong type detected for 'points'");
    }
    for (auto &sub_tbl : points->get()) {
      auto point = sub_tbl->get("point")->as_array();
      if (!point) {
        throw cpptoml::parse_exception("Wrong type detected in point");
      }
      env.push_back(cpptoml::to_point(point));
    }
  }

  // spheres
  if (tbl->contains("spheres")) {
    auto spheres  = tbl->get("spheres") ->as_table_array();
    if (!spheres) {
      throw cpptoml::parse_exception("Wrong type detected for 'spheres'");
    }
    for (auto &sub_tbl : spheres->get()) {
      env.push_back(collision::Sphere::from_toml(sub_tbl));
    }
  }

  // capsules
  if (tbl->contains("capsules")) {
    auto capsules = tbl->get("capsules")->as_table_array();
    if (!capsules) {
      throw cpptoml::parse_exception("Wrong type detected for 'capsules'");
    }
    for (auto &sub_tbl : capsules->get()) {
      env.push_back(collision::Capsule::from_toml(sub_tbl));
    }
  }

  // meshes
  if (tbl->contains("meshes")) {
    auto meshes = tbl->get("meshes")->as_table_array();
    if (!meshes) {
      throw cpptoml::parse_exception("Wrong type detected for 'meshes'");
    }
    for (auto &sub_tbl : meshes->get()) {
      env.push_back(collision::Mesh::from_toml(sub_tbl));
    }
  }

  return env;
}

void Environment::check_update() const {
  if (_dirty) {
    update_broadphase();
    _dirty = false;
  }
}

void Environment::update_broadphase() const {
  _broadphase->clear();
  _objects.clear();

  // TODO: implement point cloud for the points
  if (!_points.empty()) {
    std::cerr << "Warning: collision with points is not implemented\n";
  }

  auto add_vec = [this](const auto &vec) {
    for (const auto &shape : vec) {
      auto fcl_object = shape.to_fcl_object();
      _objects.push_back(std::move(fcl_object));
      _broadphase->registerObject(_objects.back().get());
    }
  };

  add_vec(_spheres);
  add_vec(_capsules);
  add_vec(_meshes);

  _broadphase->setup();
}

bool Environment::collides(fcl::CollisionObject *obj) const {
  check_update();
  return collision::collides(obj, _broadphase.get());
}

bool Environment::collides(fcl::BroadPhaseCollisionManager* manager) const {
  check_update();
  return collision::collides(manager, _broadphase.get());
}

bool Environment::collides(const collision::CapsuleSequence &caps) const {
  std::vector<std::shared_ptr<fcl::CollisionObject>> objs;
  CollisionManager manager;
  for (size_t i = 0; i < caps.size(); i++) {
    objs.emplace_back(caps[i].to_fcl_object());
    manager.registerObject(objs.back().get());
  }
  manager.setup();
  return collides(&manager);
}

bool Environment::collides(const Environment &other) const {
  other.check_update();
  return collides(other._broadphase.get());
}

} // end of namespace motion_planning
