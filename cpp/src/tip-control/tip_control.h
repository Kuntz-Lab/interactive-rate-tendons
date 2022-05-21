#ifndef TIP_CONTROL
#define TIP_CONTROL

#include <tendon/TendonRobot.h>
#include <util/macros.h>
#include <3rdparty/levmar-2.6/levmar.h>

#include <Eigen/Core>

#include <vector>

namespace tip_control {

// state -> output
using LMFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

// state -> tip
using FKFunc = std::function<Eigen::Vector3d(const std::vector<double>&)>;

struct LevmarResult {
  Eigen::VectorXd state;
  Eigen::VectorXd fout;
  double _info[LM_INFO_SZ];

  double err_init()         const noexcept { return _info[0]; }
  double err()              const noexcept { return _info[1]; }
  double JT_err()           const noexcept { return _info[2]; }
  double Dp()               const noexcept { return _info[3]; }
  double mu_over_JTJ()      const noexcept { return _info[4]; }
  int iters()               const noexcept { return _info[5]; }
  int term_condition()      const noexcept { return _info[6]; }
  int num_fk_calls()        const noexcept { return _info[7]; }
  int num_jacobian_calls()  const noexcept { return _info[8]; }
  int num_linear_solves()   const noexcept { return _info[9]; }

  const std::string_view term_reason() const noexcept;
};

struct IKResult {
  std::vector<double> state;     // solved solution robot state
  Eigen::Vector3d tip;           // actual achieved tip
  double error;                  // tip position error
  int iters;                     // number of iterations
  int num_fk_calls;              // number of forward kinematic calls
};

struct Bounds {
  std::vector<double> lower;
  std::vector<double> upper;

  Bounds() = default;

  static Bounds from_robot(const tendon::TendonRobot &robot);

  // Centers these bounds about state (assuming they were centered about zero).
  void center_about_state(const std::vector<double> &state);
};

Eigen::MatrixXd Jacobian(
    tendon::TendonRobot robot,
    Eigen::Vector3d ps,
    float dist,
    const std::vector<double> &tau
    );

std::vector<double> Dls(
    float damping,
    Eigen::Vector3d err,
    Eigen::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau
    );

std::vector<double> ClampedDls(
    float damping,
    Eigen::Vector3d err,
    Eigen::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau,
    float tension_space_clamp
    );

LevmarResult levenberg_marquardt(
    const LMFunc &f,
    Bounds &bounds,
    const Eigen::VectorXd &initial_state,    // p vec
    const Eigen::VectorXd &des,              // desired f output
    int max_iters = 100,
    double mu_init = 0.1,                    // approximately 1/step size
    double stop_threshold_JT_err_inf = 1e-9, // |J^T err|_inf
    double stop_threshold_Dp = 1e-4,         // |Dp|_2 (like relative step size)
    double stop_threshold_err = 1e-4,        // |err|_2
    double finite_difference_delta = 1e-6    // delta for finite difference
    );

IKResult inverse_kinematics(
    const FKFunc &fk,
    const tendon::TendonRobot &robot,
    const std::vector<double>& initial_state,
    const Eigen::Vector3d& des,
    int max_iters = 100,
    double mu_init = 0.1,                    // approximately 1/step size
    double stop_threshold_JT_err_inf = 1e-9, // |J^T err|_inf
    double stop_threshold_Dp = 1e-4,         // |Dp|_2 (like relative step size)
    double stop_threshold_err = 1e-4,        // |err|_2
    double finite_difference_delta = 1e-6,   // delta for finite difference
    bool verbose = false
    );

/** Return clamped velocity vector times delta t */
Eigen::Vector3d clamped_v_times_dt(
    const Eigen::Vector3d &measured_tip,
    const Eigen::Vector3d &desired_tip,
    const double max_speed_times_dt
    );

/** Use levmar's Levenberg-Marquardt algorithm to update resolved rate control
 *
 * This uses LM to do the equivalent of Damped Resolved Rate Control.
 *
 * @param fk
 *    Function to return the predicted tip position of the robot from a given
 *    state.
 *
 * @param robot
 *    Robot description for configuration limits
 *
 * @param current_state
 *    Current state of the robot
 *
 * @param v_times_dt
 *    Desired velocity vector times the time step (so it's a desired position
 *    update).  For example, you may use the convenience clamped_v_times_dt()
 *    function.
 *
 * @param lambda
 *    Damping factor
 *
 * @param finite_difference_delta
 *    Distance for finite differences when numerically calculating the Jacobian.
 *    Positive value means use central-differences.  Negative value means use
 *    forward-differences.
 *
 * @param verbose
 *    Print verbose information about running to the console
 *
 * Returns the new state for the next time step.
 */
std::vector<double> damped_resolved_rate_update(
    const FKFunc &fk,
    const tendon::TendonRobot &robot,
    const std::vector<double> &current_state,
    const Eigen::Vector3d &v_times_dt,
    const double lambda = 0.1,
    const double finite_difference_delta = 1e-6,
    const bool verbose = false
    );

} // end of namespace tip_control



#endif // TIP_CONTROL
