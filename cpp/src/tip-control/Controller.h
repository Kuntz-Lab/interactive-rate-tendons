#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "tendon/TendonRobot.h"
#include "tip-control/tip_control.h"

#include <Eigen/Core>

#include <vector>

class Controller {
public:
  using IKResult = tip_control::IKResult;

public:
  Controller(tendon::TendonRobot robot) : _robot(std::move(robot)) {}

  Controller()                                     = default; // default constructor
  Controller(const Controller &other)              = default; // copy
  Controller(Controller &&other)                   = default; // move
  Controller& operator = (const Controller &other) = default; // copy assignment
  Controller& operator = (Controller &&other)      = default; // move assignment

  tendon::TendonRobot& robot() { return _robot; }
  const tendon::TendonRobot& robot() const { return _robot; }

  void set_robot(tendon::TendonRobot robot) { _robot = std::move(robot); }

  struct ControlResult {
    std::vector<std::vector<double>> states;
    std::vector<Eigen::Vector3d> tip_positions;
    std::vector<double> errors;
    std::vector<std::vector<Eigen::Vector3d>> backbones;
    double seconds;
    bool success;
  };

  ControlResult control(const std::vector<double>& initialTau,
                        const Eigen::Vector3d& des,
                        char ver);

  std::vector<double> closed_loop_control(const std::vector<double>& initialTau,
                                          const Eigen::Vector3d& des,
                                          const Eigen::Vector3d& tip_pos,
                                          char ver
                                          );

  /** Perform inverse kinematics (IK) using levmar library
   *
   * @param finite_difference_delta
   *    Distance between invocations of FK func for numerically calculating
   *    Jacobian matrix.  By default uses central differences (for positive
   *    values).  Pass in a negative value to use forward differences.
   */
  IKResult inverse_kinematics(
      const std::vector<double>& initial_state,
      const Eigen::Vector3d& des,
      int max_iters = 100,
      double mu_init = 0.1,                    // approximately 1/step size
      double stop_threshold_JT_err_inf = 1e-9, // |J^T err|_inf
      double stop_threshold_Dp = 1e-4,         // |Dp|_2 (like relative step size)
      double stop_threshold_err = 1e-4,        // |err|_2
      double finite_difference_delta = 1e-6,   // delta for finite difference
      bool verbose = false
    ) const;

  /** Calculate damped resolved-rate update using LM alg. from levmar library
   *
   * This is done by doing one iteration of LM on a specially constructed
   * function based on the given FKFunc.
   *
   * The lambda is equivalent to mu_init and both finite_difference_delta and
   * verbose are like their associated parameters in inverse_kinematics().
   */
  std::vector<double> damped_resolved_rate_update(
      const std::vector<double> &current_state,
      const Eigen::Vector3d &v_times_dt,
      const double lambda = 0.1,
      const double finite_difference_delta = 1e-6,
      bool verbose = false
      ) const;

private:
  tip_control::FKFunc get_default_fk() const;

private:
  tendon::TendonRobot _robot;
};

#endif
