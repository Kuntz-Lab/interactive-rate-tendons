#include "Controller.h"
#include "util/vector_ops.h"

#include <Eigen/LU>
#include <Eigen/Dense>

#include <algorithm>
#include <chrono>

#include <cmath>

namespace E = Eigen;

Controller::ControlResult
Controller::control(const std::vector<double>& initial_state,
                    const Eigen::Vector3d& des,
                    char ver)
{
    using secs = std::chrono::duration<double>;
    using util::operator<<;

    Controller::ControlResult results;
    results.success = false;

    std::vector<double> state = initial_state;

    int n = _robot.tendons.size();

    // TODO: use the tension and other limits specified by the robot
    E::MatrixXd J(3, n);
    int N=0;
    float step_size=100,damping=0.1,err_norm,prev_err_norm=0,dist=0.001,tol=1e-10,tlimit=200, tiptol=1e-5;
    //dist is the disturbance used in Jacobian calculation
    //tlimit is the tension limit
    //damping is the damping constant used in the LM algorithm
    //tol is the minimum error change desired after each iteration
    //tiptol is the tolerated error range from the desired position
    //err_norm is the norm of the error in the current iteration
    //rpev_err_norm is the norm of the error in the previous iteration
    float state_space_clamp = 0.01;

    auto before = std::chrono::high_resolution_clock::now();
    while (1) {
        auto p = _robot.forward_kinematics(state);
        results.backbones.push_back(p);
        auto &ps = p.back();

        E::Vector3d err = des - ps;

        J = tip_control::Jacobian(_robot, ps, dist, state);
        err_norm = err.norm();

        results.tip_positions.push_back(p.back());
        results.states.push_back(state);
        results.errors.push_back(err_norm);

        if (abs(err_norm - prev_err_norm) < tol || err_norm < tiptol) {
            break;
        }

        prev_err_norm = err_norm;
        err *= step_size;
        if (ver == 'y') {
            state = tip_control::ClampedDls(damping, err, J, tlimit, state, state_space_clamp);
        } else if (ver == 'n') {
            state = tip_control::Dls(damping, err, J, tlimit, state);
        } else {
            std::cout << "\n\nNot a valid option. Use y or n\n\n";
            return results;
        }
        N += 1;
    }
    auto after = std::chrono::high_resolution_clock::now();

    results.seconds = secs(after - before).count();

    if (results.errors.back() < tiptol) {
        results.success = true;
    }

    return results;
}

std::vector<double> Controller::closed_loop_control(const std::vector<double>& initial_state,
                                                    const Eigen::Vector3d& des,
                                                    const Eigen::Vector3d& tip_pos,
                                                    char ver
                                                    )
{
    using util::operator<<;


    std::vector<double> state = initial_state;

    int n = _robot.tendons.size();

    E::MatrixXd J(3, n);
    float step_size=1e-3,damping=0.1,dist=0.0001,tlimit=200;
    float state_space_clamp = 0.1;

    auto &ps = tip_pos;

    E::Vector3d err = des - ps;

    J = tip_control::Jacobian(_robot, ps, dist, state);
    if(err.norm()>step_size){
        err = step_size*err.normalized();
    }
    if (ver == 'y') {
        state = tip_control::ClampedDls(damping, err, J, tlimit, state, state_space_clamp);
    } else if (ver == 'n') {
        state = tip_control::Dls(damping, err, J, tlimit, state);
    }

    return state;
}

/** performs inverse kinematics to a desired location of the tendon robot
 *
 * @param initial_state: initial guess at the inverse kinematics solution
 * @param des: desired end-effector position
 */
Controller::IKResult Controller::inverse_kinematics(
    const std::vector<double>& initial_state, // p vec
    const Eigen::Vector3d& des,
    int max_iters,
    double mu_init,                   // approximately 1/step size
    double stop_threshold_JT_err_inf, // |J^T err|_inf: biggest cost gradient component
    double stop_threshold_Dp,         // |Dp|_2: p is state, Dp is delta p
    double stop_threshold_err,        // |err|_2 = |des - fk(p)|_2
    double finite_difference_delta,   // delta for finite differences
    bool verbose
  ) const
{
  return tip_control::inverse_kinematics(
      this->get_default_fk(),
      this->_robot,
      initial_state,
      des,
      max_iters,
      mu_init,
      stop_threshold_JT_err_inf,
      stop_threshold_Dp,
      stop_threshold_err,
      finite_difference_delta,
      verbose
  );
}

std::vector<double> Controller::damped_resolved_rate_update(
    const std::vector<double> &current_state,
    const E::Vector3d &v_times_dt,
    const double lambda,
    const double finite_difference_delta,
    const bool verbose
  ) const
{
  return tip_control::damped_resolved_rate_update(
      this->get_default_fk(),
      this->_robot,
      current_state,
      v_times_dt,
      lambda,
      finite_difference_delta,
      verbose
      );
}

tip_control::FKFunc Controller::get_default_fk() const {
  return [this](const std::vector<double> &state) -> Eigen::Vector3d {
    return this->_robot.forward_kinematics(state).back();
  };
}
