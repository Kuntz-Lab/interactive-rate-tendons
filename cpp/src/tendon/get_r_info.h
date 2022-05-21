#ifndef GET_R_INFO_H
#define GET_R_INFO_H

#include "TendonSpecs.h"

#include <Eigen/Core>

#include <unordered_map>
#include <vector>

namespace tendon {

struct TendonSpecs;

struct rInfo {
  std::vector<Eigen::Vector3d> r;
  std::vector<Eigen::Vector3d> r_dot;
  std::vector<Eigen::Vector3d> r_ddot;

  void resize(size_t new_size) {
    r.resize(new_size);
    r_dot.resize(new_size);
    r_ddot.resize(new_size);
  }
};

// cache for get_r_info2 to not have to allocate and free memory
struct rInfoCache {
  // To store t^i and its dirivatives
  Eigen::VectorXd S;   // S[i]   = t^i
  Eigen::VectorXd Sd;  // Sd[i]  = i * t^(i-1)
  Eigen::VectorXd Sdd; // Sdd[i] = i * (i-1) * t^(i-2)

  void resize(const std::vector<TendonSpecs> &tendons) {
    const size_t N_a = tendons[0].C.size();
    const size_t N_m = tendons[0].D.size();
    const size_t N_s = std::max(N_a, N_m);
    this->resize(N_s);
  }

  void resize(size_t poly_max) {
    S.resize(poly_max);
    Sd.resize(poly_max);
    Sdd.resize(poly_max);
  }
};

rInfo get_r_info(const std::vector<TendonSpecs> &tendons, double t);
void get_r_info2(const std::vector<TendonSpecs> &tendons, double t,
                 rInfo &info, rInfoCache &cache);

} // end of namespace tendon

#endif // GET_R_INFO_H
