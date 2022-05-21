#include "submodule_motion_planning.h"

#include <motion-planning/AbstractValidityChecker.h>
#include <motion-planning/AbstractVoxelMotionValidator.h>
#include <motion-planning/AbstractVoxelValidityChecker.h>
#include <motion-planning/Environment.h>
#include <motion-planning/OctomapValidityChecker.h>
#include <motion-planning/Problem.h>
#include <motion-planning/ValidityChecker.h>
#include <motion-planning/VoxelBackboneDiscreteMotionValidator.h>
#include <motion-planning/VoxelBackboneMotionValidator.h>
#include <motion-planning/VoxelEnvironment.h>
#include <motion-planning/VoxelValidityChecker.h>
#include <motion-planning/ompl_planners.h>

#include <collision/VoxelOctree.h>
#include <collision/OctomapWrap.h>
#include <cpptoml/toml_conversions.h>
#include <tendon/BackboneSpecs.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonSpecs.h>

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types

#include <Eigen/Core>

#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace E = Eigen;
namespace py = pybind11;
namespace ob = ompl::base;

namespace {

void def_functions(py::module &m) {
  m.def("available_planners", &motion_planning::available_planners,
        "returns the list of available planners");
  m.def("is_valid_state", &motion_planning::is_valid_state,
        py::arg("state"), py::arg("robot"), py::arg("env"),
        py::arg("home_shape"), py::arg("fk_shape"),
        "returns True if not in collision and within limits");
}

void def_class_Environment(py::module &m) {
  using Env = motion_planning::Environment;

  py::class_<Env>(m, "Environment",
      "Environment of obstacles")
    .def(py::init<>())

    // attributes
    .def_property("points", &Env::points,
                  [](Env &e, std::vector<Env::Point> &&vec) { e.set_vec(vec); })
    .def_property("spheres", &Env::spheres,
                  [](Env &e, std::vector<Env::Sphere> &&vec) { e.set_vec(vec); })
    .def_property("capsules", &Env::capsules,
                  [](Env &e, std::vector<Env::Capsule> &&vec) { e.set_vec(vec); })
    .def_property("meshes", &Env::meshes,
                  [](Env &e, std::vector<Env::Mesh> &&vec) { e.set_vec(vec); })

    // methods
    .def("clear",       [](Env &e) { e.clear(); }, "clear all obstacles")
    .def("add_point",   [](Env &e, const Env::Point   &p) { e.push_back(p); })
    .def("add_sphere",  [](Env &e, const Env::Sphere  &s) { e.push_back(s); })
    .def("add_capsule", [](Env &e, const Env::Capsule &c) { e.push_back(c); })
    .def("add_mesh",    [](Env &e, const Env::Mesh    &m) { e.push_back(m); })
    .def("setup", &Env::setup,
        "To make collision checks thread-safe, you can call this first\n"
        "To continue to be thread-safe, you must call this after any changes")
    .def("collides",
        py::overload_cast<const Env::Sphere&>(&Env::collides, py::const_))
    .def("collides",
        py::overload_cast<const Env::Capsule&>(&Env::collides, py::const_))
    .def("collides",
        py::overload_cast<const Env::Mesh&>(&Env::collides, py::const_))
    .def("collides",
        py::overload_cast<const collision::CapsuleSequence&>(
          &Env::collides, py::const_))
    .def("collides", py::overload_cast<const Env&>(&Env::collides, py::const_))
    .def("to_toml", [](const Env &e, const std::string &fname) {
          cpptoml::to_file(e, fname);
        }, py::arg("filepath"), "save this object to a toml file")
    .def("voxelize",
        [](const Env &e, const collision::VoxelOctree &v, double dilate = 0.0) {
          return collision::VoxelOctree(*e.voxelize(v, dilate));
        },
        py::arg("reference"),
        py::arg("dilate") = 0.0,
        "Voxelize the environment to the same voxel dimensions and workspace "
        "limits as the given reference voxel object.  Optionally, you can "
        "dilate the environment, for example, by giving the robot radius, so "
        "that you can do motion planning and collision check only against the "
        "voxelized backbone centerline (for computational efficiency).")

    // static methods
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<Env>(fname);
        }, py::arg("filepath"),
        "load an Environment object from the given toml file")

    // python-specific added methods
    .def("__eq__", [](const Env &a, const Env &b) { return a == b; })
    .def("__repr__", [](const Env &e) {
          using motion_planning::operator<<;
          std::ostringstream builder;
          builder << e;
          return builder.str();
        })
    .def("__str__", [](const Env &e) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, e.to_toml());
          return builder.str();
        })

    ;
}

void def_class_VoxelEnvironment(py::module &m) {
  using VEnv = motion_planning::VoxelEnvironment;
  using PartVox = VEnv::PartialVoxelization;

  // TODO: finish adding everything

  py::class_<PartVox>(m, "PartialVoxelization",
      "Represents a full or partial voxelized edge")
    .def(py::init<>())

    // attributes
    .def_readwrite("is_fully_valid", &PartVox::is_fully_valid,
        "True means the whole edge was valid")
    .def_readwrite("t", &PartVox::t, "percent between a and b of last_valid")
    .def_readwrite("last_valid", &PartVox::last_valid,
        "last valid state where valid edge ends")
    .def_readwrite("last_backbone", &PartVox::last_backbone,
        "backbone line of last valid state")
    .def_readwrite("voxels", &PartVox::voxels, "voxelized (partial) edge")
    ;

  py::class_<VEnv>(m, "VoxelEnvironment", "Voxelized collision environment")
    .def(py::init<>())

    // attributes
    .def_readwrite("filename", &VEnv::filename,
        "voxel file containing obstacles")
    .def_readwrite("scaling", &VEnv::scaling,
        "scale the voxel file to meters")
    .def_readwrite("translation", &VEnv::translation,
        "move voxel file to robot frame")
    .def_readwrite("inv_rotation", &VEnv::inv_rotation,
        "rotate robot from to voxels")
    .def_readwrite("interior_fname", &VEnv::interior_fname,
        "voxel file containing free space")

    // public methods
    .def("set_obstacle_cache", &VEnv::set_obstacle_cache,
        py::arg("obstacles"),
        "set the voxel obstacles directly instead of from file")
    .def("get_obstacles",
        [](const VEnv &env) { return *env.get_obstacles(); }, // return copy
        "load voxels from filename the first time, then caches")
    .def("get_interior", &VEnv::get_interior,
        "load interior voxels from interior_fname the first time, then cache")
    .def("rotate_point", &VEnv::rotate_point,
        "rotate point in robot frame to image frame")
    .def("rotate_points",
        [](const VEnv& env, std::vector<Eigen::Vector3d> pts) {
          env.rotate_points(pts);
          return pts;
        },
        "return rotated points in robot frame to image frame")
    .def("to_toml", [](const VEnv &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<VEnv>(fname);
        }, py::arg("filepath"),
        "load a VoxelEnvironment object from the given toml file")

    // python-specific added methods
    .def("__eq__", [](const VEnv &a, const VEnv &b) { return a == b; })
    .def("__str__", [](const VEnv &venv) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, venv.to_toml());
          return builder.str();
        })

    ; // end
}

void def_class_Problem(py::module &m) {
  using Prob = motion_planning::Problem;

  // TODO: add some of the methods of these classes
  py::class_<ob::SpaceInformation,
             ob::SpaceInformationPtr>(m, "OmplSpaceInformation");
  py::class_<ob::Planner,
             ob::PlannerPtr>(m, "OmplPlanner")
    .def("getSpaceInformation", &ob::Planner::getSpaceInformation,
        "get OmplSpaceInformation object");

  py::class_<Prob>(m, "Problem", "Motion planning problem")
    .def(py::init<>())

    // attributes
    .def_readwrite("robot", &Prob::robot, "Tendon robot description")
    .def_readwrite("env", &Prob::env, "Collision environment")
    .def_readwrite("venv", &Prob::venv, "Voxel collision environment")
    .def_readwrite("start", &Prob::start, "Start tensions")
    .def_readwrite("goal", &Prob::goal, "Goal tensions")
    .def_readwrite("min_tension_change", &Prob::min_tension_change,
        "min tension change between collision checks")
    .def_readwrite("min_rotation_change", &Prob::min_rotation_change,
        "min drot between collision checks")
    .def_readwrite("min_retraction_change", &Prob::min_retraction_change,
        "min dret between collision checks")
    .def_readwrite("start_rotation", &Prob::start_rotation,
        "start state rotation (if enabled)")
    .def_readwrite("start_retraction", &Prob::start_retraction,
        "start state retraction (if enabled)")
    .def_readwrite("goal_rotation", &Prob::goal_rotation,
        "goal state rotation (if enabled)")
    .def_readwrite("goal_retraction", &Prob::goal_retraction,
        "goal state retraction (if enabled)")
    .def_readwrite("sample_like_sphere", &Prob::sample_like_sphere,
        "planner to sample like r of sphere")

    // public methods
    .def("start_state", &Prob::start_state, "full start state")
    .def("goal_state", &Prob::goal_state, "full goal state")
    .def("start_shape", &Prob::start_shape, "start robot shape")
    .def("goal_shape", &Prob::goal_shape, "goal robot shape")
    .def("start_home_shape", &Prob::start_home_shape, "start robot home shape")
    .def("goal_home_shape", &Prob::goal_home_shape, "goal robot home shape")
    .def("is_valid",
        [](const Prob &p, const std::vector<double> &state,
           std::optional<tendon::TendonResult> home_shape,
           std::optional<tendon::TendonResult> shape)
        {
          if (!home_shape) { home_shape = p.robot.home_shape(state); }
          if (!shape)      { shape = p.robot.shape(state);           }
          return p.is_valid(state, *home_shape, *shape);
        },
        py::arg("state"),
        py::arg("home_shape") = py::none(),
        py::arg("shape") = py::none(),
        "returns True if not in collision and within limits\n"
        "you may provide a precomputed cached home_shape or shape for reuse\n"
        "of computations.")
    .def("create_space_information", &Prob::create_space_information,
        "create OMPL space information object")
    .def("create_planner", &Prob::create_planner,
        py::arg("planner_name") = "RRTConnect",
        py::arg("planner_options") = motion_planning::Options{},
        py::arg("quiet") = false,
        "create OMPL planner object")
    .def("update_to_voxel_validators",
        &Prob::update_to_voxel_validators,
        py::arg("planner"),
        py::arg("backbone"),
        py::arg("swept_volume"),
        "replace state and motion validators with voxel ones")
    .def("update_to_wspace_goal", &Prob::update_to_wspace_goal,
        py::arg("planner"),
        py::arg("tip_desired"),
        py::arg("threshold") = 5e-4,
        "replace goal type from config to workspace with the given tip")
    .def("plan_cost", &Prob::plan_cost,
        py::arg("plan"),
        "calculate cost of plan")
    .def("extend_plan", &Prob::extend_plan,
        py::arg("full_plan"),
        py::arg("extension"),
        "correctly extend the plan, handling rotation space correctly")
    .def("make_plan_continuous", &Prob::make_plan_continuous,
        py::arg("plan"),
        "fix angle part of the states to be continuous, and remove duplicates")
    .def("save_plan", &Prob::save_plan,
        py::arg("csv_filename"),
        py::arg("plan"),
        "save plan to a csv file")
    .def("to_toml", [](const Prob &p, const std::string &fname) {
          cpptoml::to_file(p, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("load_plan", &Prob::load_plan,
        py::arg("csv_filename"),
        "load plan from CSV file")
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<Prob>(fname);
        }, py::arg("filepath"),
        "load a Problem object from the given toml file")

    // python-specific added methods
    .def("__eq__", [](const Prob &a, const Prob &b) { return a == b; })
    .def("__str__", [](const Prob &p) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, p.to_toml());
          return builder.str();
        })

    ; // end
}

void def_classes_StateValidators(py::module &m) {
  using AbsVC = motion_planning::AbstractValidityChecker; // mesh
  using AbsVVC = motion_planning::AbstractVoxelValidityChecker;
  using VC = motion_planning::ValidityChecker; // mesh
  using VVC = motion_planning::VoxelValidityChecker;
  using OctVC = motion_planning::OctomapValidityChecker; // OctomapWrap

  py::class_<AbsVC, std::shared_ptr<AbsVC>>(m, "AbstractValidityChecker")
    .def("clear_timing", &AbsVC::clear_timing, "clear all timers")
    .def("timing", &AbsVC::timing, py::arg("key"), "return timer times")
    .def("timings",
        [](const AbsVC &mv) {
          std::unordered_map<std::string, std::vector<float>> timings;
          for (auto &[name, timer] : mv.timers()) {
            timings.emplace(name, timer.get_times());
          }
          return timings;
        },
        "return all times from all timers")
    .def("calls", &AbsVC::calls, py::arg("key"), "return timer calls")
    .def("robot", &AbsVC::robot, "robot description")
    .def("fk", &AbsVC::fk, py::arg("state"),
        "FK, returning (fk_shape, home_shape)")
    .def("is_valid_shape", &AbsVC::is_valid_shape,
        py::arg("fk_shape"), py::arg("home_shape"),
        "Within tension and length limits, and does not self-collide.\n"
        "Does not check for collisions against the environment.\n"
        "Pass in values from fk().")
    .def("collides", &AbsVC::collides, py::arg("fk_shape"),
        "Return True if the fk_shape collides with the environment.")
    ;

  py::class_<AbsVVC, std::shared_ptr<AbsVVC>, AbsVC>(
          m, "AbstractVoxelValidityChecker")
    .def("voxelize", &AbsVVC::voxelize, py::arg("fk_shape"),
        "Return the fk shape as a voxel object")
    .def("collides",
        py::overload_cast<const collision::VoxelOctree&>(
            &AbsVVC::collides, py::const_),
        py::arg("voxelized"),
        "Collision check against an already voxelized shape")
    ;

  py::class_<VC, std::shared_ptr<VC>, AbsVC>(m, "ValidityChecker")
    .def(py::init<const ob::SpaceInformationPtr&,
                  const tendon::TendonRobot&,
                  const motion_planning::Environment&>())
    ;

  py::class_<VVC, std::shared_ptr<VVC>, AbsVVC>(m, "VoxelValidityChecker")
    .def(py::init<const ob::SpaceInformationPtr&,
                  const tendon::TendonRobot&,
                  const motion_planning::VoxelEnvironment&,
                  collision::VoxelOctree>())
    ;

  py::class_<OctVC, std::shared_ptr<OctVC>, AbsVC>(m, "OctomapValidityChecker")
    .def(py::init<const ob::SpaceInformationPtr&,
                  const tendon::TendonRobot&,
                  const motion_planning::VoxelEnvironment&,
                  const collision::VoxelOctree&>())
    ;
}

void def_classes_MotionValidators(py::module &m) {
  using AbsMV = motion_planning::AbstractVoxelMotionValidator;
  using MV = motion_planning::VoxelBackboneMotionValidator;
  using DMV = motion_planning::VoxelBackboneDiscreteMotionValidator;

  py::class_<AbsMV, std::shared_ptr<AbsMV>>(m, "AbstractVoxelMotionValidator")
    .def("clear_timing", &AbsMV::clear_timing, "clear all timers")
    .def("timing", &AbsMV::timing, py::arg("key"), "return timer times")
    .def("timings",
        [](const AbsMV &mv) {
          std::unordered_map<std::string, std::vector<float>> timings;
          for (auto &[name, timer] : mv.timers()) {
            timings.emplace(name, timer.get_times());
          }
          return timings;
        },
        "return all times from all timers")
    .def("calls", &AbsMV::calls, py::arg("key"), "return timer calls")
    .def("num_voxelize_errors", &AbsMV::num_voxelize_errors)
    .def("validity_checker", &AbsMV::validity_checker,
        "return the validity checker from the space information")
    .def("voxelize", &AbsMV::voxelize, py::arg("a"), py::arg("b"),
        "voxelize swept volume between a and b, without collision checks,\n"
        "but may end if an otherwise 'invalid' state is encountered")
    .def("voxelize_until_invalid", &AbsMV::voxelize_until_invalid,
        py::arg("a"), py::arg("b"),
        "voxelize swept volume between a and b, stopping at the first\n"
        "collision")
    .def("collides", &AbsMV::collides, py::arg("swept_volume"),
        "check for collision against an already voxelized edge")
    ;

  py::class_<MV, std::shared_ptr<MV>, AbsMV>(m, "VoxelBackboneMotionValidator")
    .def(py::init<const ob::SpaceInformationPtr&,
                  const tendon::TendonRobot&,
                  const motion_planning::VoxelEnvironment&,
                  collision::VoxelOctree>())
    ;

  py::class_<DMV, std::shared_ptr<DMV>, MV>(
          m, "VoxelBackboneDiscreteMotionValidator")
    .def(py::init<const ob::SpaceInformationPtr&,
                  const tendon::TendonRobot&,
                  const motion_planning::VoxelEnvironment&,
                  collision::VoxelOctree>())
    ;
}

void def_class_NearestNeighbor(py::module &m) {
  struct NNEntry {
    E::VectorXd key;
    py::object  val;

    bool operator==(const NNEntry &other) const { return val.equal(other.val); }
    bool operator!=(const NNEntry &other) const { return val.not_equal(other.val); }

    py::tuple to_tuple() const { return py::make_tuple(key, val); }
  };

  auto entry_vec_conv = [](const std::vector<NNEntry> &vec) {
    std::vector<py::tuple> converted(vec.size());
    std::transform(vec.begin(), vec.end(), converted.begin(),
                   [](const auto &val) { return val.to_tuple(); });
    return converted;
  };

  using NN = ompl::NearestNeighborsGNATNoThreadSafety<NNEntry>;
  py::class_<NN>(m, "NearestNeighborL2",
      "Nearest neighbor data structure with keys being numpy array-like and\n"
      "values being any object.  This structure provides efficient lookup\n"
      "of values that have close keys.\n"
      "\n"
      "This class is a python interface to OMPL's\n"
      "NearestNeighborGNATNoThreadSafety class.\n"
      "\n"
      "Keys are restricted to be array-like of the same size. Nearness is\n"
      "simply the L2 norm of the keys.\n"
      "\n"
      "Equivalence is equivalence of the values (not including the keys).\n"
      "That means we remove elements based on the value, not the key.")

    // constructor (also hard-codes the distance function)
    .def("__init__",
        [](NN &nn,
           unsigned int degree,
           unsigned int maxDegree,
           unsigned int maxNumPtsPerLeaf,
           unsigned int removedCacheSize,
           bool rebalancing)
        {
          new (&nn) NN(degree, maxDegree, maxNumPtsPerLeaf, removedCacheSize,
                       rebalancing);
          nn.setDistanceFunction([](const NNEntry &a, const NNEntry &b) {
            if (a.key.size() != b.key.size()) {
              throw std::length_error("one key is a different length");
            }
            return (a.key - b.key).norm();
          });
        },
        py::arg("degree") = 8,
        py::arg("maxDegree") = 12,
        py::arg("maxNumPtsPerLeaf") = 50,
        py::arg("removedCacheSize") = 500,
        py::arg("rebalancing") = false,
        "I'm not entirely sure what these parameters are, look at OMPL docs\n"
        "or source code for more information.")

    // public methods
    .def("clear", &NN::clear)
    .def("add",
        [](NN &nn, E::VectorXd &&key, py::object val) {
          NNEntry entry { std::move(key), val };
          nn.add(std::move(entry));
        },
        py::arg("key"),
        py::arg("val"))
    .def("extend",
        [](NN &nn, const py::iterable &keys_and_vals) {
          for (const py::handle kvobj : keys_and_vals) {
            const py::tuple kv = py::cast<const py::tuple>(kvobj);
            if (kv.size() != 2) {
              throw std::length_error("found key-value pair of length != 2");
            }
            NNEntry entry {kv[0].cast<E::VectorXd>(), kv[1]};
            nn.add(std::move(entry));
          }
        },
        py::arg("keys_and_vals"),
        "Extend multiple entries.  Pass in an iterable of 2-tuples of\n"
        "(key, value) pairs.")
    .def("rebuild", &NN::rebuildDataStructure,
        "Rebuild the internal data structure")
    .def("remove",
        [](NN &nn, const E::VectorXd &key, const py::object &val) {
          return nn.remove(NNEntry{key, val});
        },
        py::arg("key"),
        py::arg("val"),
        "Remove data from the tree by key and value.  Key should be close\n"
        "by, but value needs to be equivalent.\n"
        "\n"
        "Returns True if the element was found and 'removed'\n"
        "\n"
        "The element won't actually be removed immediately, but just marked\n"
        "for removal in the removed_ cache.  When the cache is full, the\n"
        "tree will be rebuilt and the elements marked for removal will\n"
        "actually be removed.")
    .def("nearest",
        [](NN &nn, const E::VectorXd &key) {
          auto nearest = nn.nearest(NNEntry{key, py::none()});
          return py::make_tuple(nearest.key, nearest.val);
        },
        py::arg("key"),
        "Return the nearest neighbor as a tuple of (key, val)")
    .def("nearestK",
        [=](NN &nn, const E::VectorXd &key, size_t k) {
          std::vector<NNEntry> nearest;
          nearest.reserve(k);
          nn.nearestK(NNEntry{key, py::none()}, k, nearest);
          return entry_vec_conv(nearest);
        },
        py::arg("key"),
        py::arg("k"),
        "Returns (up to) the k nearest neighbors in a list of (key, val) pairs")
    .def("nearestR",
        [=](NN &nn, const E::VectorXd &key, double r) {
          std::vector<NNEntry> nearest;
          nn.nearestR(NNEntry{key, py::none()}, r, nearest);
          return entry_vec_conv(nearest);
        },
        py::arg("key"),
        py::arg("r"),
        "Returns neighbors within a distance of r as a list of (key, val)\n"
        "pairs")
    .def("items",
        [=](NN &nn) {
          std::vector<NNEntry> items;
          nn.list(items);
          return entry_vec_conv(items);
        })

    // python-specific functions
    .def("__len__", &NN::size)
    //.def("__repr__", [](NN &nn) {
    //      using ompl::operator<<;
    //      std::ostringstream ss;
    //      ss << nn;
    //      return ss.str();
    //    })

    ;
}

} // end of unnamed namespace

py::module def_submodule_motion_planning(py::module &m) {
  auto submodule = m.def_submodule("motion_planning",
                                   "Motion planning components for tendon robots");
  def_functions(submodule);
  def_class_Environment(submodule);
  def_class_VoxelEnvironment(submodule);
  def_class_Problem(submodule);
  def_classes_StateValidators(submodule);
  def_classes_MotionValidators(submodule);
  def_class_NearestNeighbor(submodule);
  return submodule;
}
