#include "submodule_tendon.h"

#include <cpptoml/toml_conversions.h>
#include <tendon/BackboneSpecs.h>
#include <tendon/TendonResult.h>
#include <tendon/TendonRobot.h>
#include <tendon/TendonSpecs.h>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types

#include <Eigen/Core>

#include <sstream>
#include <string>
#include <vector>

namespace E = Eigen;
namespace py = pybind11;

namespace {

using tendon::BackboneSpecs;
using tendon::TendonSpecs;
using tendon::TendonRobot;
using tendon::TendonResult;
using tendon::PointForces;
using tendon::DistFLFunc;


void def_class_BackboneSpecs(py::module &m) {
  py::class_<BackboneSpecs>(m, "BackboneSpecs",
      "Specs for the backbone on a tendon-actuated robot")
    .def(py::init<double, double, double, double, double, double>(),
        py::arg("L") = 0.2,
        py::arg("dL") = 0.005,
        py::arg("ro") = 0.01,
        py::arg("ri") = 0.0,
        py::arg("E") = 2.1e6,
        py::arg("nu") = 0.3)

    // attributes
    .def_readwrite("L", &BackboneSpecs::L,
                   "robot backbone length (meters)")
    .def_readwrite("dL", &BackboneSpecs::dL,
                   "backbone discretization length (meters)")
    .def_readwrite("ro", &BackboneSpecs::ro,
                   "outer backbone radius (meters)")
    .def_readwrite("ri", &BackboneSpecs::ri,
                   "inner backbone radius (meters) - for hollow backbones")
    .def_readwrite("E", &BackboneSpecs::E,
                   "Young's modulus (pascals = N/m^2)")
    .def_readwrite("nu", &BackboneSpecs::nu,
        "Poisson's ratio: response in directions orthogonal to\n"
        "uniaxial stress.  Relates young's modulus to shear\n"
        "modulus with isotropic materials.")

    // methods
    .def("to_toml", [](const BackboneSpecs &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml",
        [](const std::string &fname) {
          return cpptoml::from_file<BackboneSpecs>(fname);
        }, py::arg("filepath"),
        "load a BackboneSpecs object from the given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const BackboneSpecs &a, const BackboneSpecs &b) {
          return a == b;
        })
    .def("__repr__", [](const BackboneSpecs &obj) {
          std::ostringstream builder;
          builder << "BackboneSpecs("
                     "L: " << obj.L << ", "
                     "dL: " << obj.dL << ", "
                     "ro: " << obj.ro << ", "
                     "ri: " << obj.ri << ", "
                     "E: " << obj.E << ", "
                     "nu: " << obj.nu << ")";
          return builder.str();
        })
    .def("__str__", [](const BackboneSpecs &obj) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, obj.to_toml());
          return builder.str();
        })
    .def("__copy__", [](const BackboneSpecs &obj) {
          return BackboneSpecs(obj);
        })
    .def("__deepcopy__", [](const BackboneSpecs &obj) {
          return BackboneSpecs(obj);
        })
    .def(py::pickle(
        [](const BackboneSpecs &s) { // __getstate__
          return py::make_tuple(s.L, s.dL, s.ro, s.ri, s.E, s.nu);
        },
        [](py::tuple t) { // __setstate__
          return BackboneSpecs{
              t[0].cast<double>(),
              t[1].cast<double>(),
              t[2].cast<double>(),
              t[3].cast<double>(),
              t[4].cast<double>(),
              t[5].cast<double>()};
        }))
    ;
}

void def_class_TendonResult(py::module &m) {
  py::class_<TendonResult>(m, "TendonResult",
      "Results from calculating tendon shape")
    .def(py::init<std::vector<double>,
                  std::vector<E::Vector3d>,
                  std::vector<E::Matrix3d>,
                  double,
                  std::vector<double>,
                  E::Vector3d,
                  E::Vector3d,
                  E::Vector3d,
                  E::Vector3d,
                  bool>(),
        py::arg("t") = std::vector<double>{},
        py::arg("p") = std::vector<E::Vector3d>{},
        py::arg("R") = std::vector<E::Matrix3d>{},
        py::arg("L") = 0.020,
        py::arg("L_i") = std::vector<double>{},
        py::arg("u_i") = E::Vector3d::Zero(),
        py::arg("v_i") = E::Vector3d{0, 0, 1},
        py::arg("u_f") = E::Vector3d::Zero(),
        py::arg("v_f") = E::Vector3d{0, 0, 1},
        py::arg("converged") = true)

    // attributes
    .def_readwrite("t", &TendonResult::t,
                   "parameterization value")
    .def_readwrite("p", &TendonResult::p,
                   "p[j] = backbone position at t[j]")
    .def_readwrite("R", &TendonResult::R,
                   "R[j] = rotation matrix from base frame at t[j]")
    .def_readwrite("L", &TendonResult::L,
                   "Current length of the backbone (meters)")
    .def_readwrite("L_i", &TendonResult::L_i,
                   "L_i[i] = current length of tendon i from the robot base")
    .def_readwrite("u_i", &TendonResult::u_i, "initial u vector at t = 0")
    .def_readwrite("v_i", &TendonResult::v_i, "initial v vector at t = 0")
    .def_readwrite("u_f", &TendonResult::u_f, "final u vector at t = L")
    .def_readwrite("v_f", &TendonResult::v_f, "final v vector at t = L")
    .def_readwrite("converged", &TendonResult::converged,
                   "true means u_f and v_f converged")

    // methods
    .def("to_toml",
         [](const TendonResult &result, const std::string &fname) {
           cpptoml::to_file(result, fname);
         }, py::arg("filepath"), "save this object to a toml file")
    .def("rotate_z", &TendonResult::rotate_z, py::arg("theta"),
         "rotate this object about the base-frame's z-axis.\n"
         "recalculates the positions and rotations only.")

    // python-specific added methods
    .def("__str__", [](const TendonResult &result) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, result.to_toml());
          return builder.str();
        })
    .def("__copy__", [](const TendonResult &obj) {
          return TendonResult(obj);
        })
    .def("__deepcopy__", [](const TendonResult &obj) {
          return TendonResult(obj);
        })
    .def(py::pickle(
        [](const TendonResult &r) { // __getstate__
          return py::make_tuple(r.t, r.p, r.R, r.L, r.L_i,
                                r.u_i, r.v_i, r.u_f, r.v_f,
                                r.converged);
        },
        [](py::tuple t) { // __setstate__
          return TendonResult{
              t[0].cast<std::vector<double>>(),       // t
              t[1].cast<std::vector<E::Vector3d>>(),  // p
              t[2].cast<std::vector<E::Matrix3d>>(),  // R
              t[3].cast<double>(),                    // L
              t[4].cast<std::vector<double>>(),       // L_i
              t[5].cast<E::Vector3d>(),               // u_i
              t[6].cast<E::Vector3d>(),               // v_i
              t[7].cast<E::Vector3d>(),               // u_f
              t[8].cast<E::Vector3d>(),               // v_f
              t[9].cast<bool>()                       // converged
          };
        }))
    ;
}

void def_class_TendonRobot(py::module &m) {
  py::class_<PointForces>(m, "PointForces",
      "Point forces on the tendon robot\n"
      "calculated from a shape computation and tensions")
    .def(py::init<E::Vector3d,
                  E::Vector3d,
                  E::Vector3d,
                  E::Vector3d,
                  E::Vector3d,
                  E::Vector3d>(),
        py::arg("F_e") = E::Vector3d{0, 0, 0},
        py::arg("L_e") = E::Vector3d{0, 0, 0},
        py::arg("F_t") = E::Vector3d{0, 0, 0},
        py::arg("L_t") = E::Vector3d{0, 0, 0},
        py::arg("n")   = E::Vector3d{0, 0, 0},
        py::arg("m")   = E::Vector3d{0, 0, 0})

    // attributes
    .def_readwrite("F_e", &PointForces::F_e,
                   "External force exterted on the robot")
    .def_readwrite("L_e", &PointForces::L_e,
                   "External torques exterted on the robot")
    .def_readwrite("F_t", &PointForces::F_t,
                   "Force on the robot by the tendons")
    .def_readwrite("L_t", &PointForces::L_t,
                   "Torque on the robot by the tendons")
    .def_readwrite("n", &PointForces::n,
                   "internal force carried by the backbone")
    .def_readwrite("m", &PointForces::m,
                   "internal torque carried by the backbone")

    // methods
    .def("residual", &PointForces::residual,
         "combined residual of external force and torque")

    // static methods
    .def_static("calc_point_forces", PointForces::calc_point_forces,
        py::arg("tau"),
        py::arg("R"),
        py::arg("u"),
        py::arg("v"),
        py::arg("K_se"),
        py::arg("K_bt"),
        py::arg("r"),
        py::arg("r_dot"),
        "Calculate the forces at a particular point along the backbone\n"
        "\n"
        "Parameters\n"
        "----------\n"
        "\n"
        "tau:    tensions (or full state)\n"
        "R:      rotation matrix at the point\n"
        "u:      u vector at the point (angular rate of change)\n"
        "v:      v vector at the point (linear rate of change)\n"
        "K_se:   stiffness matrix for shear and extension\n"
        "K_bt:   stiffness matrix for bending and torsion\n"
        "r:      tendon displacement vectors\n"
        "r_dot:  derivative of r along s (the length param of the backbone)\n"
        "\n"
        "Returns a PointForces object")

    // python-specific added methods
    .def("__copy__", [](const PointForces &obj) {
          return PointForces(obj);
        })
    .def("__deepcopy__", [](const PointForces &obj) {
          return PointForces(obj);
        })
    .def(py::pickle(
        [](const PointForces &v) { // __getstate__
          return py::make_tuple(v.F_e, v.L_e, v.F_t, v.L_t, v.n, v.m);
        },
        [](py::tuple t) { // __setstate__
          return PointForces{
              t[0].cast<E::Vector3d>(),  // F_e
              t[1].cast<E::Vector3d>(),  // L_e
              t[2].cast<E::Vector3d>(),  // F_t
              t[3].cast<E::Vector3d>(),  // L_t
              t[4].cast<E::Vector3d>(),  // n
              t[5].cast<E::Vector3d>()}; // m
        }))
    ; // end of PointForces


  py::class_<TendonRobot>(m, "TendonRobot",
      "Representation of a tendon-actuated robot")
    .def(py::init<double, BackboneSpecs,
                  std::vector<TendonSpecs>, bool, bool, double>(),
        py::arg("r") = 0.015,
        py::arg("specs") = BackboneSpecs{},
        py::arg("tendons") = std::vector<TendonSpecs>{},
        py::arg("enable_rotation") = false,
        py::arg("enable_retraction") = false,
        py::arg("residual_threshold") = 5e-6)

    // public attributes
    .def_readwrite("r", &TendonRobot::r,
        "robot radius (meters)")
    .def_readwrite("specs", &TendonRobot::specs,
        "backbone specifications")
    .def_readwrite("tendons", &TendonRobot::tendons, "tendons")
    .def_readwrite("enable_rotation", &TendonRobot::enable_rotation,
        "enable rotation control")
    .def_readwrite("enable_retraction", &TendonRobot::enable_retraction,
        "enable retraction control")
    .def_readwrite("residual_threshold", &TendonRobot::residual_threshold,
        "threshold for converged FK")

    // public methods
    .def("state_size", &TendonRobot::state_size,
        "size of the state vector used.  Equal to number of tendons plus the\n"
        "rotation and retraction dimensions (if enabled).")
    .def("random_state", &TendonRobot::random_state,
        "generate and return a random valid state")
    .def("forward_kinematics", &TendonRobot::forward_kinematics,
        py::arg("state"),
        "calculate the backbone positions for a given state")
    .def("home_shape",
        py::overload_cast<double>(&TendonRobot::home_shape,
                                  py::const_),
        py::arg("s_start") = 0.0,
        "shape at the zero tension config (efficiently computed)\n"
        "\n"
        "Note, s_start will be used even if retraction is disabled.")
    .def("home_shape",
        py::overload_cast<const std::vector<double>&>(
          &TendonRobot::home_shape, py::const_),
        py::arg("state"),
        "shape at the zero tension config (efficiently computed)\n"
        "\n"
        "Will pull the s_start value out of the state only if retraction is\n"
        "enabled, otherwise a value of 0.0 will be used.")
    .def("shape",
        py::overload_cast<const std::vector<double> &>(
          &TendonRobot::shape, py::const_),
        py::arg("state"),
        "computes and returns the robot shape from the given state.")
    .def("shape",
        py::overload_cast<const std::vector<double>&, double, double>(
          &TendonRobot::shape, py::const_),
        py::arg("tau"), py::arg("rotation"),
        py::arg("retraction"),
        "computes and returns the robot shape from the given controls.")
    .def("shape_unopt",
        py::overload_cast<const std::vector<double> &>(
          &TendonRobot::shape_unopt, py::const_),
        py::arg("state"),
        "Same as shape() but the unoptimized version")
    .def("shape_unopt",
        py::overload_cast<const std::vector<double>&, double, double>(
          &TendonRobot::shape_unopt, py::const_),
        py::arg("tau"), py::arg("rotation"),
        py::arg("retraction"),
        "Same as shape() but the unoptimized version")
    .def("general_shape",
        py::overload_cast<
            const std::vector<double>&,
            const DistFLFunc&,
            const DistFLFunc&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            int,
            double,
            double,
            double,
            double,
            bool
              >(&TendonRobot::general_shape, py::const_),
        py::arg("state"),
        py::arg("f_e"),
        py::arg("l_e"),
        py::arg("F_e"),
        py::arg("L_e"),
        py::arg("u_guess") = Eigen::Vector3d::Zero(),
        py::arg("v_guess") = Eigen::Vector3d {0, 0, 1},
        py::arg("max_iters") = 100,
        py::arg("mu_init") = 0.1,
        py::arg("stop_threshold_JT_err_inf") = 1e-9,
        py::arg("stop_threshold_Dp") = 1e-4,
        py::arg("finite_difference_delta") = 1e-6,
        py::arg("verbose") = true,
        "Convenience wrapper around the general_shape() where tau, rotation,\n"
        "and retetraction are each individually specified")
    .def("general_shape",
        py::overload_cast<
            const std::vector<double>&,
            double,
            double,
            const DistFLFunc&,
            const DistFLFunc&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            const Eigen::Vector3d&,
            int,
            double,
            double,
            double,
            double,
            bool
              >(&TendonRobot::general_shape, py::const_),
        py::arg("tau"),
        py::arg("rotation"),
        py::arg("retraction"),
        py::arg("f_e"),
        py::arg("l_e"),
        py::arg("F_e"),
        py::arg("L_e"),
        py::arg("u_guess") = Eigen::Vector3d::Zero(),
        py::arg("v_guess") = Eigen::Vector3d {0, 0, 1},
        py::arg("max_iters") = 100,
        py::arg("mu_init") = 0.1,
        py::arg("stop_threshold_JT_err_inf") = 1e-9,
        py::arg("stop_threshold_Dp") = 1e-4,
        py::arg("finite_difference_delta") = 1e-6,
        py::arg("verbose") = true,
        "Convenience wrapper around general_tension_shape(), rotating the\n"
        "result after.  The retraction and rotation values will only be used\n"
        "if they are enabled.")
    .def("general_tension_shape", &TendonRobot::general_tension_shape,
        py::arg("tau"),
        py::arg("s_start"),
        py::arg("f_e"),
        py::arg("l_e"),
        py::arg("F_e"),
        py::arg("L_e"),
        py::arg("u_guess") = E::Vector3d{0, 0, 0},
        py::arg("v_guess") = E::Vector3d{0, 0, 1},
        py::arg("max_iters") = 100,
        py::arg("mu_init") = 0.1,
        py::arg("stop_threshold_JT_err_inf") = 1e-9,
        py::arg("stop_threshold_Dp") = 1e-4,
        py::arg("finite_difference_delta") = 1e-6,
        py::arg("verbose") = true,
        "Compute the shape of the tendon robot at the given tensions and\n"
        "external loads.\n"
        "\n"
        "The f_e, l_e, F_e, and L_e are separate and in addition to the forces\n"
        "and torques on the backbone provided by the tendons.\n"
        "\n"
        "This function uses Levenberg-Marquardt causing multiple passes of\n"
        "integration.  It is much more expensive than shape(), but is able\n"
        "to handle any continuous distributed force and torque along the\n"
        "backbone.\n"
        "\n"
        "Parameters:\n"
        "\n"
        "  tau: np.array[N] with N = len(tendons)\n"
        "\n"
        "    Tendon tensions, to be between tendon limits.  This is not\n"
        "    checked.\n"
        "\n"
        "  s_start: float\n"
        "\n"
        "    Starting point for shape computation.  needs to be between 0 and\n"
        "    specs.L.  Represents control of robot insertion, i.e., only the\n"
        "    robot from s_start to specs.L is in the workspace.\n"
        "\n"
        "  f_e: Function[double, np.array[3]] -> np.array[3]\n"
        "\n"
        "    Function returning the external force per unit length along the\n"
        "    backbone at backbone parameter t (float) and at workspace position\n"
        "    p (np.array[3]).  The workspace position p is the position of the\n"
        "    backbone at backbone parameter t.\n"
        "\n"
        "  l_e: Function[double, np.array[3]] -> np.array[3]\n"
        "\n"
        "    Function returning the external torque per unit length along the\n"
        "    backbone at backbone parameter t (float) and at workspace position\n"
        "    p (np.array[3]).  The workspace position p is the position of the\n"
        "    backbone at backbone parameter t.\n"
        "\n"
        "  F_e: np.array[3]\n"
        "\n"
        "    External point force vector at the tip of the tendon robot\n"
        "\n"
        "  L_e: np.array[3]\n"
        "\n"
        "    External point torque vector at the tip of the tendon robot\n"
        "\n"
        "  u_guess: np.array[3]\n"
        "\n"
        "    Initial guess for initial bending vector u (default: [0, 0, 0])\n"
        "\n"
        "  v_guess: np.array[3]\n"
        "\n"
        "    Initial guess for initial vector v (default: [0, 0, 1])\n"
        "\n"
        )
    .def("tip_forces", &TendonRobot::tip_forces,
        py::arg("tau"), py::arg("shape"),
        "Returns the PointForces at the tip from the tensions and shape.\n"
        "The tau vector can safely contain rotation and retraction, they will\n"
        "simply be ignored.")
    .def("base_forces", &TendonRobot::base_forces,
        py::arg("tau"), py::arg("shape"),
        "Returns the PointForces at the base from the tensions and shape.\n"
        "The tau vector can safely contain rotation and retraction, they will\n"
        "simply be ignored.")
    .def("is_valid",
        py::overload_cast<const std::vector<double>&,
                          const TendonResult&>(
          &TendonRobot::is_valid, py::const_),
        py::arg("state"), py::arg("home_shape"),
        "returns True if the state is within valid tension and length limits")
    .def("is_valid",
        py::overload_cast<const std::vector<double>&,
                          const TendonResult&,
                          const TendonResult&>(
          &TendonRobot::is_valid, py::const_),
        py::arg("state"), py::arg("home_shape"),
        py::arg("shape"),
        "returns True if the state is within valid tension and length limits")
    .def("collides_self", &TendonRobot::collides_self,
        py::arg("shape"),
        "returns True if this robot shape collides with itself")
    .def("shape_and_lengths",
        [](const TendonRobot &robot,
           const std::vector<double> &state,
           const TendonResult* const home_shape)
        {
          if (home_shape) {
            return robot.shape_and_lengths(state, *home_shape);
          }
          return robot.shape_and_lengths(state);
        },
        py::arg("state"),
        py::arg("home_shape") = py::none(),
        "Calculate both the shape and the commanded lengths as a tuple\n"
        "    shape, lengths = robot.shape_and_lengths(tensions)"
        )
    .def("calc_dl", &TendonRobot::calc_dl,
        py::arg("home_tendon_lengths"), py::arg("tendon_lengths"),
        "calculates the change in tendon lengths from the home shape\n"
        "\n"
        "expected to pass in TendonResult.L_i for both arguments, one\n"
        "from the home shape and the other from the current shape")
    .def("is_within_length_limits",
        py::overload_cast<const std::vector<double>&,
                          const std::vector<double>&>(
          &TendonRobot::is_within_length_limits, py::const_),
        py::arg("home_tendon_lengths"), py::arg("tendon_lengths"),
        "returns True if the difference between the two tendon lengths are\n"
        "within the length limits.")
    .def("is_within_length_limits",
        py::overload_cast<const std::vector<double>&>(
          &TendonRobot::is_within_length_limits, py::const_),
        py::arg("dl"),
        "returns True if the given difference vector is within the length\n"
        "limits")
    .def("load_config_csv", &TendonRobot::load_config_csv,
        py::arg("csv_file"),
        "Loads configurations from a csv file with columns\n"
        "- tau_i (for i in [1 : #tendons])\n"
        "- theta (if robot.enable_rotation)\n"
        "- s_start (if robot.enable_retraction)\n"
        "Returns a list of robot configurations.")
    .def("to_toml", [](const TendonRobot &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const std::string &fname) {
          return cpptoml::from_file<TendonRobot>(fname);
        }, py::arg("filepath"),
        "load a TendonRobot object from the given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const TendonRobot &a, const TendonRobot &b) {
          return a == b;
        })
    .def("__repr__", [](const TendonRobot &robot) {
          using tendon::operator<<;
          std::ostringstream builder;
          builder << robot;
          return builder.str();
        })
    .def("__str__", [](const TendonRobot &robot) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, robot.to_toml());
          return builder.str();
        })
    .def("__copy__", [](const TendonRobot &obj) {
          return TendonRobot(obj);
        })
    .def("__deepcopy__", [](const TendonRobot &obj) {
          return TendonRobot(obj);
        })
    .def(py::pickle(
        [](const TendonRobot &r) { // __getstate__
          return py::make_tuple(r.r, r.specs, r.tendons,
                                r.enable_rotation, r.enable_retraction,
                                r.residual_threshold);
        },
        [](py::tuple t) { // __setstate__
          return TendonRobot{
              t[0].cast<double>(),                       // r
              t[1].cast<BackboneSpecs>(),                // specs
              t[2].cast<std::vector<TendonSpecs>>(),     // tendons
              t[3].cast<bool>(),                         // enable_rotation
              t[4].cast<bool>(),                         // enable_retraction
              t[5].cast<double>()};                      // residual_threshold
        }))
    ;
}

void def_class_TendonSpecs(py::module &m) {
  py::class_<TendonSpecs>(m, "TendonSpecs",
      "Specs for a tendon on a tendon-actuated robot")
    .def(py::init<E::VectorXd, E::VectorXd, double, double, double>(),
        py::arg("C") = E::VectorXd{},
        py::arg("D") = E::VectorXd{},
        py::arg("max_tension") = 20.0,
        py::arg("min_length") = -0.015,
        py::arg("max_length") = 0.035)

    // public attributes
    .def_readwrite("C", &TendonSpecs::C,
        "theta polynomial coefficients of tendon")
    .def_readwrite("D", &TendonSpecs::D,
        "r polynomial coefficients of tendons")
    .def_readwrite("max_tension", &TendonSpecs::max_tension,
        "maximum allowed tension on the tendon (Newtons)")
    .def_readwrite("min_length", &TendonSpecs::min_length,
        "minimum string extension allowed (meters)")
    .def_readwrite("max_length", &TendonSpecs::max_length,
        "maximum string extension allowed (meters)")

    // methods
    .def("is_straight", &TendonSpecs::is_straight,
        "return True if the tendon is routed straight",
        py::arg("eps") = 0.0)
    .def("is_helix", &TendonSpecs::is_helix,
        "return True if the tendon is routed in a helical shape",
        py::arg("eps") = 0.0)
    .def("to_toml", [](const TendonSpecs &obj, const std::string &fname) {
          cpptoml::to_file(obj, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml",
        [](const std::string &fname) {
          return cpptoml::from_file<TendonSpecs>(fname);
        }, py::arg("filepath"),
        "load a TendonSpecs object from a given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const TendonSpecs &a, const TendonSpecs &b) {
          return a == b;
        })
    .def("__repr__", [](const TendonSpecs &obj) {
          std::ostringstream builder;
          using tendon::operator<<;
          builder << obj;
          return builder.str();
        })
    .def("__str__", [](const TendonSpecs &obj) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, obj.to_toml());
          return builder.str();
        })
    .def("__copy__", [](const TendonSpecs &obj) {
          return TendonSpecs(obj);
        })
    .def("__deepcopy__", [](const TendonSpecs &obj) {
          return TendonSpecs(obj);
        })
    .def(py::pickle(
        [](const TendonSpecs &s) { // __getstate__
          return py::make_tuple(s.C, s.D, s.max_tension,
                                s.min_length, s.max_length);
        },
        [](py::tuple t) { // __setstate__
          return TendonSpecs{
              t[0].cast<E::VectorXd>(),  // C
              t[1].cast<E::VectorXd>(),  // D
              t[2].cast<double>(),       // max_tension
              t[3].cast<double>(),       // min_length
              t[4].cast<double>()};      // max_length
        }))
    ;
}

} // end of unnamed namespace

py::module def_submodule_tendon(py::module &m) {
  auto submodule = m.def_submodule("tendon", "The tendon robot base components");
  def_class_BackboneSpecs(submodule);
  def_class_TendonSpecs(submodule);
  def_class_TendonRobot(submodule);
  def_class_TendonResult(submodule);
  return submodule;
}
