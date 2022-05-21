#include "tip-control/tip_control.h"
#include "util/vector_ops.h"
#include "util/angles.h"

#include <3rdparty/levmar-2.6/levmar.h>

#include <Eigen/LU>     // for Eigen::Matrix::inverse() definition
#include <Eigen/Dense>  // TODO: is this used?

namespace E = Eigen;

namespace tip_control {

namespace {

std::string_view lm_term_reason(const int val) {
  switch (val) {
    case 1:  return "small |J^T e|_inf";
    case 2:  return "small |Dp|";
    case 3:  return "max iter";
    case 4:  return "singular matrix (recommend increasing mu from result)";
    case 5:  return "progres not possible (recommend restart and increasing mu)";
    case 6:  return "small |e|";
    case 7:  return "invalid FK values (NaN or Inf)";
    default: return "";
  }
}

struct ExtendedIKResult {
  IKResult main;
  double info[LM_INFO_SZ];
};

// more general version that can be used from public inverse_kinematics()
ExtendedIKResult inverse_kinematics_impl(
    const FKFunc &fk,                         // forward-kinematics function
    Bounds &bounds,
    const bool enable_rotation,
    const bool enable_retraction,
    const double robot_L,
    const std::vector<double>& initial_state, // p vec
    const E::Vector3d& des,
    int max_iters,
    double mu_init,                    // approximately 1/step size
    double stop_threshold_JT_err_inf,  // |J^T err|_inf
    double stop_threshold_Dp,          // |Dp|_2 (like relative step size)
    double stop_threshold_err,         // |err|_2
    double finite_difference_delta     // delta for finite difference
  )
{
  static thread_local size_t workspace_size = 0;
  static thread_local double *workspace = nullptr;

  ExtendedIKResult soln;
  std::vector<double> solution = initial_state;
  E::Vector3d des_copy = des;

  int m = initial_state.size();
  int n = des.size();
  double levmar_opt[LM_OPTS_SZ];

  // allocate the levmar workspace buffer, known only at runtime, but local to
  // this thread
  size_t new_workspace_size = LM_DIF_WORKSZ(m, n);
  if (workspace != nullptr && workspace_size < new_workspace_size) {
    delete[] workspace; // free the old one that's too small
    workspace = nullptr;
  }
  if (workspace == nullptr) {
    workspace_size = new_workspace_size;
    workspace = new double[workspace_size];
  }

  levmar_opt[0] = mu_init;
  levmar_opt[1] = stop_threshold_JT_err_inf;

  // looks like the thresholds are on the square norm, so square them
  levmar_opt[2] = stop_threshold_Dp * stop_threshold_Dp;
  levmar_opt[3] = stop_threshold_err * stop_threshold_err;

  // negative for central-difference instead of forward-difference
  levmar_opt[4] = -finite_difference_delta;

  const auto n_tendons =
      initial_state.size() - size_t(enable_retraction) - size_t(enable_rotation);

  struct FKWrapArgs {
    const bool enable_retraction;
    const double robot_L;
    const FKFunc &fk;
  };

  FKWrapArgs fk_args {enable_retraction, robot_L, fk};

  // wrapper function to give to levmar
  // Note: lambdas with no captures can be cast to a raw function pointer.
  auto fk_wrap =
    [](double *p, double *x_out, int p_dim, int x_dim, void *in_args) -> void {
      auto *args = static_cast<FKWrapArgs*>(in_args);
      const auto &fk = args->fk;
      // other controls are allowed to be out of range, but retraction doesn't make
      // sense.  Hard-code a return value in that case.
      if (args->enable_retraction) {
        if (p[p_dim-1] > args->robot_L) {
          for (int i = 0; i < x_dim-1; ++i) {
            x_out[i] = 0.0;
          }
          x_out[x_dim-1] = args->robot_L - p[p_dim-1]; // negative z value
          return;
        }
      }
      std::vector<double> state(p, p + p_dim);
      decltype(fk(state)) tip;
      try {
        tip = fk(state);
      } catch (std::exception &ex) {
        using util::operator<<;
        std::cerr << "exception thrown from FKFunc fk() for state " << state;
        throw;
      }
      for (int i = 0; i < std::min(x_dim, 3); i++) {
        x_out[i] = tip[i];
      }
    };

  dlevmar_bc_dif(fk_wrap,
                 solution.data(),
                 des_copy.data(),
                 m, n,
                 bounds.lower.data(),
                 bounds.upper.data(),
                 nullptr, // no diagonal scaling constraints
                 max_iters,
                 levmar_opt,
                 soln.info,
                 workspace,
                 nullptr, // do not need the covariance matrix
                 static_cast<void*>(&fk_args));

  const auto reached_tip = fk(solution);

  if (enable_rotation) {
    solution[n_tendons] = util::canonical_angle(solution[n_tendons]);
  }

  soln.main.state        = std::move(solution);
  soln.main.tip          = std::move(reached_tip);
  soln.main.error        = std::sqrt(soln.info[1]);
  soln.main.iters        = soln.info[5];
  soln.main.num_fk_calls = soln.info[7];

  return soln;
} // end of function inverse_kinematics_impl()

} // end of unnamed namespace


const std::string_view LevmarResult::term_reason() const noexcept {
  return lm_term_reason(term_condition());
}

Bounds Bounds::from_robot(const tendon::TendonRobot &robot) {
  Bounds bounds;
  const auto n = robot.tendons.size();
  const auto m = robot.state_size();
  bounds.lower.resize(m, 0.0);
  bounds.upper.resize(m);
  std::transform(robot.tendons.begin(), robot.tendons.end(),
                 bounds.upper.begin(),
                 [](auto &tendon) { return tendon.max_tension; });
  if (robot.enable_rotation) {
    bounds.lower[n] = std::numeric_limits<double>::lowest();
    bounds.upper[n] = std::numeric_limits<double>::max();
  }
  if (robot.enable_retraction) {
    bounds.upper.back() = robot.specs.L;
  }
  return bounds;
}

// Centers these bounds about state (assuming they were centered about zero).
void Bounds::center_about_state(const std::vector<double> &state) {
  for (size_t i = 0; i < state.size(); ++i) {
    this->lower[i] -= state[i];
    this->upper[i] -= state[i];
  }
}

//Function Definitions
std::vector<double> Dls(
    float damping,
    E::Vector3d err,
    E::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau
    )
{
  E::Matrix3d I = E::Matrix3d::Identity();
  E::VectorXd tdiff;
  std::vector<double> t = tau;

  tdiff = (J * J.transpose() + damping * I).inverse() * J * err;
  for(size_t h = 0; h < tau.size(); h++) {
    t[h] += tdiff(h);
    if (t[h] > tlimit) {
      t[h] = tlimit;
    } else if (t[h] < 0) {
      t[h] = 0;
    }
  }
  return t;
}

std::vector<double> ClampedDls(
    float damping,
    E::Vector3d err,
    E::MatrixXd J,
    float tlimit,
    const std::vector<double> &tau,
    float tension_space_clamp)
{
  E::Matrix3d I = E::Matrix3d::Identity();
  E::VectorXd tdiff;
  std::vector<double> t(tau.size());

  t = tau;
  tdiff = (J * J.transpose() + damping * I).inverse() * J * err;
  if (tdiff.norm() > tension_space_clamp) {
    tdiff = tdiff.normalized() * tension_space_clamp;
  }
  for (size_t h = 0; h < tau.size(); h++) {
    t[h] += tdiff(h);
    if (t[h] > tlimit) {
      t[h] = tlimit;
    } else if (t[h] < 0) {
      t[h]=0;
    }
  }
  return t;
}


E::MatrixXd Jacobian(
    tendon::TendonRobot robot,
    E::Vector3d ps,
    float dist,
    const std::vector<double> &tau)
{
  E::MatrixXd J(3, tau.size());
  E::Vector3d pos;
  std::vector<double> tau2;

  //#pragma omp parallel for
  for (size_t i = 0; i < tau.size(); i++) {
    tau2 = tau;
    tau2[i] = tau[i] + dist;
    auto p = robot.forward_kinematics(tau2);
    pos = p.back();
    for(size_t j = 0; j < 3; j++) {
      J(j,i) = (pos[j] - ps[j]) / dist;
    }
  }
  return J;
}

LevmarResult levenberg_marquardt(
    const LMFunc &f,
    Bounds &bounds,
    const E::VectorXd &initial_state,  // p vec
    const E::VectorXd &des,            // desired f output
    int max_iters,      
    double mu_init,                    // approximately 1/step size
    double stop_threshold_JT_err_inf,  // |J^T err|_inf
    double stop_threshold_Dp,          // |Dp|_2 (like relative step size)
    double stop_threshold_err,         // |err|_2
    double finite_difference_delta     // delta for finite difference
    )
{
  static thread_local size_t workspace_size = 0;
  static thread_local double *workspace = nullptr;
  double levmar_opt[LM_OPTS_SZ];

  LevmarResult soln;
  soln.state = initial_state;
  soln.fout = des;

  int m = soln.state.size();
  int n = soln.fout.size();

  // allocate the levmar workspace buffer, known only at runtime, but local to
  // this thread
  size_t new_workspace_size = LM_DIF_WORKSZ(m, n);
  if (workspace != nullptr && workspace_size < new_workspace_size) {
    delete[] workspace; // free the old one that's too small
    workspace = nullptr;
  }
  if (workspace == nullptr) {
    workspace_size = new_workspace_size;
    workspace = new double[workspace_size];
  }

  levmar_opt[0] = mu_init;
  levmar_opt[1] = stop_threshold_JT_err_inf;

  // looks like the thresholds are on the square norm, so square them
  levmar_opt[2] = stop_threshold_Dp * stop_threshold_Dp;
  levmar_opt[3] = stop_threshold_err * stop_threshold_err;

  // negative for central-difference instead of forward-difference
  levmar_opt[4] = -finite_difference_delta;

  struct FWrapArg {
    const LMFunc &f;
  };
  FWrapArg argwrap {f};

  // wrapper function to give to levmar
  // Note: lambdas with no captures can be cast to a raw function pointer.
  auto f_wrap =
    [](double *p, double *x_out, int p_dim, int x_dim, void *in_args) -> void {
      auto *arg = static_cast<FWrapArg*>(in_args);
      const auto &f = arg->f;
      Eigen::VectorXd state = Eigen::VectorXd::Map(p, p_dim); // copy p
      auto fout = f(state);
      std::memcpy(x_out, fout.data(), x_dim * sizeof(double));
    };

  dlevmar_bc_dif(f_wrap,
                 soln.state.data(),
                 soln.fout.data(),
                 m, n,
                 bounds.lower.data(),
                 bounds.upper.data(),
                 nullptr, // no diagonal scaling constraints
                 max_iters,
                 levmar_opt,
                 soln._info,
                 workspace,
                 nullptr, // do not need the covariance matrix
                 static_cast<void*>(&argwrap));

  soln.fout = f(soln.state);

  return soln;
}

IKResult inverse_kinematics(
    const FKFunc &fk,                         // forward-kinematics function
    const tendon::TendonRobot &robot,
    const std::vector<double>& initial_state, // p vec
    const E::Vector3d& des,
    int max_iters,
    double mu_init,                    // approximately 1/step size
    double stop_threshold_JT_err_inf,  // |J^T err|_inf
    double stop_threshold_Dp,          // |Dp|_2 (like relative step size)
    double stop_threshold_err,         // |err|_2
    double finite_difference_delta,    // delta for finite difference
    bool verbose
  )
{
  auto bounds = Bounds::from_robot(robot);
  auto soln = inverse_kinematics_impl(
      fk, bounds,
      robot.enable_rotation, robot.enable_retraction, robot.specs.L,
      initial_state, des, max_iters, mu_init,
      stop_threshold_JT_err_inf, stop_threshold_Dp, stop_threshold_err,
      finite_difference_delta
  );

  if (verbose) {
    using util::operator<<;

    auto initial_tip = fk(initial_state);
    std::cout <<
        "\n"
        "inverse_kinematics:\n"
        "  initial state:   " << initial_state                      << "\n"
        "  initial tip:     [" << initial_tip.transpose()           << "]\n"
        "  desired tip:     [" << des.transpose()                   << "]\n"
        "  reached tip:     [" << soln.main.tip.transpose()         << "]\n"
        "  solution:        " << soln.main.state                    << "\n"
        "  |e|_init:        " << std::sqrt(soln.info[0])            << "\n"
        "  |e|:             " << std::sqrt(soln.info[1])
        << "    (threshold: " << stop_threshold_err                 << ")\n"
        "  |J^T e|_inf:     " << soln.info[2]
        << "    (threshold: " << stop_threshold_JT_err_inf          << ")\n"
        "  |Dp|:            " << std::sqrt(soln.info[3])
        << "    (threshold: " << stop_threshold_Dp                  << ")\n"
        "  mu/max(J^T J):   " << soln.info[4]
        << "    (mu_init: "   << mu_init                            << ")\n"
        "  iters:           " << soln.info[5]                       << "\n"
        "  term condition:  " << int(soln.info[6])
        << ",   reason: "     << lm_term_reason(int(soln.info[6]))  << "\n"
        "  # fk calls:      " << soln.info[7]                       << "\n"
        "  # J calls:       " << soln.info[8]                       << "\n"
        "  # linear solves: " << soln.info[9]                       << "\n"
        "\n" << std::flush;
  }

  return soln.main;
} // end of function inverse_kinematics()

E::Vector3d clamped_v_times_dt(
    const Eigen::Vector3d &measured_tip,
    const Eigen::Vector3d &desired_tip,
    const double max_speed_times_dt
    )
{
  auto e = desired_tip - measured_tip;
  auto e_norm = e.norm();
  if (e_norm > max_speed_times_dt) {
    return e * max_speed_times_dt / e_norm;
  } else {
    return e;
  }
}

std::vector<double> damped_resolved_rate_update(
    const FKFunc &fk,
    const tendon::TendonRobot &robot,
    const std::vector<double> &current_state,
    const E::Vector3d &v_times_dt,
    const double lambda,
    const double finite_difference_delta,
    bool verbose
    )
{
  const auto &q = current_state;
  // wrap around fk to convert LM to resolved-rates
  const auto f = [&fk, v_times_dt, fk_of_q = fk(q), &q]
           (const std::vector<double>& x) -> E::Vector3d
  {
    std::vector<double> q_next(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
      q_next[i] = q[i] + x[i];
    }
    return v_times_dt + fk_of_q - fk(q_next);
  };

  const std::vector<double> x0(q.size(), 0.0); // zero vector
  const auto f_des = E::Vector3d::Zero();
  const int max_iters = 1;
  const double stop_thresh = 1e-9;
  auto bounds = Bounds::from_robot(robot);
  bounds.center_about_state(q);

  const auto soln = inverse_kinematics_impl(
      f, bounds,
      robot.enable_rotation, robot.enable_retraction, robot.specs.L,
      x0, f_des, max_iters, lambda,
      stop_thresh, stop_thresh, stop_thresh,
      finite_difference_delta
  );

  const auto &dq = soln.main.state;
  std::vector<double> q_next(q.size());
  for (size_t i = 0; i < q.size(); ++i) {
    q_next[i] = q[i] + dq[i];
  }

  if (verbose) {
    using util::operator<<;

    std::cout <<
        "\n"
        "damped_resolved_rate_update:\n"
        "  lambda:             " << lambda                             << "\n"
        "  fdiff delta:        " << finite_difference_delta            << "\n"
        "  desired    v * dt:  " << v_times_dt                         << "\n"
        "   error  in v * dt:  " << soln.main.tip                      << "\n"
        "  |error| in v * dt:  " << soln.main.error                    << "\n"
        "  q:                  " << q                                  << "\n"
        "  dq:                 " << dq                                 << "\n"
        "  q_next:             " << q_next                             << "\n"
        "  term condition:     " << int(soln.info[6])
        << ",   reason: "        << lm_term_reason(int(soln.info[6]))  << "\n"
        "  # fk calls:         " << soln.main.num_fk_calls + 1         << "\n"
        "\n" << std::flush;
        ;
  }

  return q_next;
}

} // end of namespace tip_control
