#include "submodule_controller.h"

#include <tip-control/Controller.h>
#include <tip-control/tip_control.h>
#include <tendon/TendonRobot.h>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types
#include <pybind11/functional.h>

namespace py = pybind11;

namespace {

using tip_control::Bounds;
using tip_control::IKResult;
using tip_control::LevmarResult;

void def_class_Controller(py::module &m) {
  // ControlResult
  py::class_<Controller::ControlResult>(m, "ControlResult",
      "returned result of calling Controller.control()")
    .def(py::init<>())

    // attributes
    .def_readwrite("states", &Controller::ControlResult::states,
        "state at each control iteration")
    .def_readwrite("tip_positions", &Controller::ControlResult::tip_positions,
        "tip positions at each control iteration")
    .def_readwrite("errors", &Controller::ControlResult::errors,
        "tip position errors at each control iteration")
    .def_readwrite("backbones", &Controller::ControlResult::backbones,
        "backbone shapes at each control iteration")
    .def_readwrite("seconds", &Controller::ControlResult::seconds,
        "time the control() method took")
    .def_readwrite("success", &Controller::ControlResult::success,
        "True means the controller got close enough to the desired tip position")
    ;

  // IKResult
  py::class_<IKResult>(m, "IKResult",
      "returned result of calling Controller.inverse_kinematics()")
    .def(py::init<>())

    // attributes
    .def_readwrite("state", &IKResult::state,
        "solved solution robot state")
    .def_readwrite("tip", &IKResult::tip,
        "actual achieved tip")
    .def_readwrite("error", &IKResult::error,
        "final tip position error")
    .def_readwrite("iters", &IKResult::iters,
        "number of iterations executed")
    .def_readwrite("num_fk_calls", &IKResult::num_fk_calls,
        "number of forward kinematic calls made")
    ;

  py::class_<Controller>(m, "Controller",
      "a controller for tip position using Levenberg-Marquardt")

    // constructor
    .def(py::init<tendon::TendonRobot>(), py::arg("robot"))
    .def(py::init<>())

    // properties
    .def_property("robot",
        py::overload_cast<>(&Controller::robot), &Controller::set_robot)

    // methods
    .def("control", &Controller::control,
        py::arg("initial_state"), py::arg("desired_tip"),
        py::arg("version") = 'n',
        "Control the tip to the desired tip position using damped least\n"
        "squares (also known as Levenberg-Marquardt).  This version was\n"
        "implemented by scratch and gives more information about each\n"
        "iteration than the other version inverse_kinematics().  This\n"
        "version runs much slower, but may be more informative.  It also\n"
        "provides a clamped version of which the inverse_kinematics()\n"
        "function does not.\n"
        "\n"
        "@param initial_state: initial guess to start from\n"
        "@param desired_tip: desired tip position to achieve\n"
        "@param version: 'y' means use the clamped version, 'n' means\n"
        "    non-clamped version.")
    .def("inverse_kinematics", &Controller::inverse_kinematics,
        py::arg("initial_state"),
        py::arg("desired_tip"),
        py::arg("max_iters") = 100,
        py::arg("mu_init") = 1e-3,
        py::arg("stop_threshold_JT_err_inf") = 1e-9,
        py::arg("stop_threshold_Dp") = 1e-4,
        py::arg("stop_threshold_err") = 1e-4,
        py::arg("finite_difference_delta") = 1e-6,
        py::arg("verbose") = false,
        "Solve for the configuration that gets the tip of the\n"
        "tendon-actuated robot closest to the desired tip position (i.e.,\n"
        "inverse kinematics).  This does essentially the same thing as the\n"
        "control() method, but uses the levmar library to run\n"
        "Levenberg-Marquardt.  It gives more control about stopping\n"
        "criteria, and is more efficient than control(), but does not have a\n"
        "clamped version and does not give as much detailed information\n"
        "about each iteration, just the final result.\n"
        "\n"
        "@param initial_state: initial guess to start from\n"
        "@param desired_tip: desired tip position to achieve\n"
        "@param max_iters: maximum allowed number of iterations\n"
        "@param mu_init: mu is a parameter that influences step size.  Too\n"
        "    big or too small may cause convergence to be too slow.  The mu\n"
        "    value will be updated automatically, this is just the initial\n"
        "    guess for a good mu value.\n"
        "@param stop_threshold_JT_err_inf: when max(J.transpose() * err) is\n"
        "    less than this threshold, exit early, where J is the computed\n"
        "    Jacobian matrix (using finite differences), and err is the\n"
        "    error vector for the current tip position.  This threshold is\n"
        "    related to the current step size and makes the algorithm stop\n"
        "    if it is no longer making good progress at each iteration.\n"
        "@param stop_threshold_Dp: when Dp.norm() is less than this\n"
        "    threshold, then exit early.  Look up the Levenberg-Marquardt\n"
        "    algorithm to see what Dp represents.  This is related to the\n"
        "    relative step size and makes the algorithm stop if it is no\n"
        "    longer making good progress at each iteration.\n"
        "@param stop_threshold_err: when err.norm() is less than this\n"
        "    threshold, then exit early.  It is the desired accuracy to the\n"
        "    given desired_tip position.  If we achieve the desired\n"
        "    accuracy, then we can exit early.\n"
        "@param finite_difference_delta: the Jacobian is computed using\n"
        "    finite differences.  This is the distance from the current\n"
        "    configuration to step for computing the finite differences.\n"
        "@param verbose: True means to output information at the end of the\n"
        "    inverse kinematics solving.\n")
    .def("damped_resolved_rate_update", &Controller::damped_resolved_rate_update,
      py::arg("current_state"),
      py::arg("v_times_dt"),
      py::arg("lambda") = 0.1,
      py::arg("finite_difference_delta") = 1e-6,
      py::arg("verbose") = false,
      "Use levmar's Levenberg-Marquardt algorithm to update resolved rate control\n"
      "\n"
      "This uses LM to do the equivalent of Damped Resolved Rate Control.\n"
      "\n"
      "@param current_state\n"
      "   Current state of the robot\n"
      "\n"
      "@param v_times_dt\n"
      "   Desired velocity vector times the time step (so it's a desired position\n"
      "   update).  For example, you may use the convenience clamped_v_times_dt()\n"
      "   function.\n"
      "\n"
      "@param lambda\n"
      "   Damping factor\n"
      "\n"
      "@param finite_difference_delta\n"
      "   Distance for finite differences when numerically calculating the Jacobian.\n"
      "   Positive value means use central-differences.  Negative value means use\n"
      "   forward-differences.\n"
      "\n"
      "@param verbose\n"
      "   Print verbose information about running to the console\n"
      "\n"
      "Returns the new state for the next time step.\n")
      ;
}

void def_tip_controller_functions(py::module &m) {
  m.def("inverse_kinematics", &tip_control::inverse_kinematics,
      py::arg("fk"),
      py::arg("robot"),
      py::arg("initial_state"),
      py::arg("desired_tip"),
      py::arg("max_iters") = 100,
      py::arg("mu_init") = 1e-3,
      py::arg("stop_threshold_JT_err_inf") = 1e-9,
      py::arg("stop_threshold_Dp") = 1e-4,
      py::arg("stop_threshold_err") = 1e-4,
      py::arg("finite_difference_delta") = 1e-6,
      py::arg("verbose") = false,
      "Solve for the configuration that gets the tip of the\n"
      "tendon-actuated robot closest to the desired tip position (i.e.,\n"
      "inverse kinematics).  This uses the levmar library to run\n"
      "Levenberg-Marquardt.  This version gives you the chance to define\n"
      "your own fk function.  Using Controller.inverse_kinematics(), they\n"
      "give a good default fk function of robot.forward_kinematics().\n"
      "\n"
      "@param fk: Function[list[np.array[double]], list[float]]\n"
      "    Forward-kinematics function to use in IK.\n"
      "@param robot: TendonRobot\n"
      "@param initial_state: initial guess to start from\n"
      "@param desired_tip: desired tip position to achieve\n"
      "@param max_iters: maximum allowed number of iterations\n"
      "@param mu_init: mu is a parameter that influences step size.  Too\n"
      "    big or too small may cause convergence to be too slow.  The mu\n"
      "    value will be updated automatically, this is just the initial\n"
      "    guess for a good mu value.\n"
      "@param stop_threshold_JT_err_inf: when max(J.transpose() * err) is\n"
      "    less than this threshold, exit early, where J is the computed\n"
      "    Jacobian matrix (using finite differences), and err is the\n"
      "    error vector for the current tip position.  This threshold is\n"
      "    related to the current step size and makes the algorithm stop\n"
      "    if it is no longer making good progress at each iteration.\n"
      "@param stop_threshold_Dp: when Dp.norm() is less than this\n"
      "    threshold, then exit early.  Look up the Levenberg-Marquardt\n"
      "    algorithm to see what Dp represents.  This is related to the\n"
      "    relative step size and makes the algorithm stop if it is no\n"
      "    longer making good progress at each iteration.\n"
      "@param stop_threshold_err: when err.norm() is less than this\n"
      "    threshold, then exit early.  It is the desired accuracy to the\n"
      "    given desired_tip position.  If we achieve the desired\n"
      "    accuracy, then we can exit early.\n"
      "@param finite_difference_delta: the Jacobian is computed using\n"
      "    finite differences.  This is the distance from the current\n"
      "    configuration to step for computing the finite differences.\n"
      "@param verbose: True means to output information at the end of the\n"
      "    inverse kinematics solving.\n");

  m.def("clamped_v_times_dt", &tip_control::clamped_v_times_dt,
      py::arg("measured_tip"),
      py::arg("desired_tip"),
      py::arg("max_speed_times_dt"),
      "Return clamped velocity times dt in the direction of desired tip");

  m.def("damped_resolved_rate_update", &tip_control::damped_resolved_rate_update,
      py::arg("fk"),
      py::arg("robot"),
      py::arg("current_state"),
      py::arg("v_times_dt"),
      py::arg("lambda") = 0.1,
      py::arg("finite_difference_delta") = 1e-6,
      py::arg("verbose") = false,
      "Use levmar's Levenberg-Marquardt algorithm to update resolved rate control\n"
      "\n"
      "This uses LM to do the equivalent of Damped Resolved Rate Control.\n"
      "\n"
      "@param fk:  Function[list[np.array[double]], list[float]]\n"
      "   Function to return the predicted tip position of the robot from a given\n"
      "   state.\n"
      "\n"
      "@param robot:  TendonRobot\n"
      "   Robot description for configuration limits\n"
      "\n"
      "@param current_state\n"
      "   Current state of the robot\n"
      "\n"
      "@param v_times_dt\n"
      "   Desired velocity vector times the time step (so it's a desired position\n"
      "   update).  For example, you may use the convenience clamped_v_times_dt()\n"
      "   function.\n"
      "\n"
      "@param lambda\n"
      "   Damping factor\n"
      "\n"
      "@param finite_difference_delta\n"
      "   Distance for finite differences when numerically calculating the Jacobian.\n"
      "   Positive value means use central-differences.  Negative value means use\n"
      "   forward-differences.\n"
      "\n"
      "@param verbose\n"
      "   Print verbose information about running to the console\n"
      "\n"
      "Returns the new state for the next time step.\n");

  py::class_<LevmarResult>(m, "LevmarResult",
      "returned result of calling levenberg_marquardt()")
    .def(py::init<>())

    // attributes
    .def_readwrite("state", &LevmarResult::state,
        "solved solution function input")
    .def_readwrite("fout", &LevmarResult::fout,
        "actual achieved function output")
    .def_property_readonly("err_init"          , &LevmarResult::err_init)
    .def_property_readonly("err"               , &LevmarResult::err)
    .def_property_readonly("JT_err"            , &LevmarResult::JT_err)
    .def_property_readonly("Dp"                , &LevmarResult::Dp)
    .def_property_readonly("mu_over_JTJ"       , &LevmarResult::mu_over_JTJ)
    .def_property_readonly("iters"             , &LevmarResult::iters)
    .def_property_readonly("term_condition"    , &LevmarResult::term_condition)
    .def_property_readonly("num_fk_calls"      , &LevmarResult::num_fk_calls)
    .def_property_readonly("num_jacobian_calls", &LevmarResult::num_jacobian_calls)
    .def_property_readonly("num_linear_solves" , &LevmarResult::num_linear_solves)
    .def_property_readonly("term_reason"       , &LevmarResult::term_reason)
    ;

  py::class_<Bounds>(m, "Bounds",
      "Bounds for levenberg_marquardt() input values.")
    .def(py::init<>())

    // attributes
    .def_readwrite("lower", &Bounds::lower)
    .def_readwrite("upper", &Bounds::upper)

    // methods
    .def("center_about_state", &Bounds::center_about_state, py::arg("state"))

    // static methods
    .def_static("from_robot", &Bounds::from_robot, py::arg("robot"))
    ;

  m.def("levenberg_marquardt", &tip_control::levenberg_marquardt,
      py::arg("f"),
      py::arg("bounds"),
      py::arg("initial_state"),
      py::arg("des"),
      py::arg("max_iters") = 100,
      py::arg("mu_init") = 0.1,
      py::arg("stop_threshold_JT_err_inf") = 1e-9,
      py::arg("stop_threshold_Dp") = 1e-4,
      py::arg("stop_threshold_err") = 1e-4,
      py::arg("finite_difference_delta") = 1e-6,
      "Use levmar's Levenberg-Marquardt algorithm to argmin_x(|f(x)-des|)\n"
      "\n"
      "@param f:  Function[np.array[double], np.array[double]]\n"
      "   Function whose value should match des.\n"
      "\n"
      "@param bounds:  Bounds\n"
      "   State space box constraints\n"
      "\n"
      "@param initial_state\n"
      "   Initial guess\n"
      "\n"
      "@param des\n"
      "   Desired function output value.\n"
      "\n"
      "@param max_iters: maximum allowed number of iterations\n"
      "\n"
      "@param mu_init: mu is a parameter that influences step size.  Too\n"
      "    big or too small may cause convergence to be too slow.  The mu\n"
      "    value will be updated automatically, this is just the initial\n"
      "    guess for a good mu value.\n"
      "\n"
      "@param stop_threshold_JT_err_inf: when max(J.transpose() * err) is\n"
      "    less than this threshold, exit early, where J is the computed\n"
      "    Jacobian matrix (using finite differences), and err is the\n"
      "    error vector for the current tip position.  This threshold is\n"
      "    related to the current step size and makes the algorithm stop\n"
      "    if it is no longer making good progress at each iteration.\n"
      "\n"
      "@param stop_threshold_Dp: when Dp.norm() is less than this\n"
      "    threshold, then exit early.  Look up the Levenberg-Marquardt\n"
      "    algorithm to see what Dp represents.  This is related to the\n"
      "    relative step size and makes the algorithm stop if it is no\n"
      "    longer making good progress at each iteration.\n"
      "\n"
      "@param stop_threshold_err: when err.norm() is less than this\n"
      "    threshold, then exit early.  It is the desired accuracy to the\n"
      "    given desired_tip position.  If we achieve the desired\n"
      "    accuracy, then we can exit early.\n"
      "\n"
      "@param finite_difference_delta: the Jacobian is computed using\n"
      "    finite differences.  This is the distance from the current\n"
      "    configuration to step for computing the finite differences.\n"
      "\n"
      "Returns the new state for the next time step.\n");
}

}

py::module def_submodule_controller(py::module &m) {
  auto submodule = m.def_submodule("controller",
      "Controllers for the tendon-actuated robot");
  def_class_Controller(submodule);
  def_tip_controller_functions(submodule);
  return submodule;
}
