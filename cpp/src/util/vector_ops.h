#ifndef UTIL_VECTOR_OPS_H
#define UTIL_VECTOR_OPS_H

#include <Eigen/Core>

#include <iostream>
#include <numeric> // for std::accumulate
#include <vector>

namespace util {

struct SimpleStats {
  double min;
  double mean;
  double median;
  double max;
  double total;
};

//
// declarations (so you know what's in this file)
//

inline Eigen::Matrix3d hat(const Eigen::Vector3d &u);
inline std::vector<double> range(double begin, double end, double diff);
inline std::vector<std::vector<double>> range(
    const std::vector<double> &begin,
    const std::vector<double> &end,
    double diff_length);
inline std::vector<double> linspace(double begin, double end, size_t N);
inline std::vector<std::vector<double>> linspace(
    std::vector<double> begin,
    std::vector<double> end,
    size_t N);

template <typename T>
inline SimpleStats calc_stats(std::vector<T> vals);

template<typename Iterable, typename T>
bool contains(const Iterable &haystack, const T &needle);

template <typename S, int R, int O, int MR, int MC>
std::ostream& operator<<(std::ostream& out,
                         const std::vector<Eigen::Matrix<S,R,1,O,MR,MC>> &mat);

inline std::ostream& operator<<(std::ostream& out,
                                const std::vector<double> &vec);

//
// definitions
//

inline Eigen::Matrix3d hat(const Eigen::Vector3d &u) {
  Eigen::Matrix3d u_hat;
  u_hat <<     0, -u[2],  u[1],
            u[2],     0, -u[0],
           -u[1],  u[0],     0;
  return u_hat;
}

/**
 * Creates a vector from begin to end at increments of diff.
 *
 * The first element is begin, the last element is end.  The end is guaranteed
 * to be at least diff away from the previous element (may be up to 2*diff).
 */
inline std::vector<double> range(double begin, double end, double diff) {
  std::vector<double> vals;
  vals.reserve(size_t((end - begin) / diff) + 2);
  for (double p = begin; p <= end - (diff / 2); p += diff) {
    vals.emplace_back(p);
  }
  vals.emplace_back(end);
  return vals;
}

/// Similar to range of doubles, but in diff length
inline std::vector<std::vector<double>> range(
    const std::vector<double> &begin,
    const std::vector<double> &end,
    double diff_length)
{
  std::vector<std::vector<double>> vals;
  using V = Eigen::VectorXd;
  auto emplace_vector = [&vals](const V &vec) {
    vals.emplace_back(vec.data(), vec.data() + vec.size());
  };

  if (begin.size() != end.size()) {
    throw std::out_of_range("begin is not the same size as end");
  }
  V vbegin(begin.size());
  V vend(end.size());
  for (size_t i = 0; i < begin.size(); i++) {
    vbegin[i] = begin[i];
    vend[i] = end[i];
  }
  V diff = vend - vbegin;
  double total_length = diff.norm();
  if (total_length <= std::numeric_limits<double>::epsilon()) {
    return {end};
  }
  V unit_diff = diff / total_length;
  auto interp = range(0.0, total_length, diff_length);
  for (auto interp_val : interp) {
    emplace_vector(vbegin + interp_val * unit_diff);
  }
  return vals;
}

inline std::vector<double> linspace(double begin, double end, size_t N) {
  if (N == 0) { return {}; }
  if (N == 1) { return {begin}; }
  if (N == 2) { return {begin, end}; }

  double dx = (end - begin) / (N - 1);
  std::vector<double> vals;
  vals.reserve(N);
  for (size_t i = 0; i < N; ++i) {
    vals.emplace_back(begin);
    begin += dx;
  }
  return vals;
}

inline std::vector<std::vector<double>> linspace(
    std::vector<double> begin,
    std::vector<double> end,
    size_t N)
{
  if (begin.size() != end.size()) {
    throw std::out_of_range("begin is not the same size as end");
  }

  if (N == 0) { return {}; }
  if (N == 1) { return {begin}; }
  if (N == 2) { return {begin, end}; }

  std::vector<double> diff;
  for (size_t i = 0; i < begin.size(); ++i) {
    diff.emplace_back((end[i] - begin[i]) / (N - 1));
  }

  std::vector<std::vector<double>> result;
  result.reserve(N);
  std::vector<double> current = begin;
  for (size_t i = 0; i < N; ++i) {
    result.emplace_back(current);
    for (size_t j = 0; j < current.size(); ++j) {
      current[j] += diff[j];
    }
  }

  return result;
}

template <typename T>
inline SimpleStats calc_stats(std::vector<T> vals) {
  if (vals.empty()) {
    double nan {std::numeric_limits<double>::quiet_NaN()};
    return {nan, nan, nan, nan, nan};
  }

  std::sort(vals.begin(), vals.end());

  SimpleStats stats;

  auto N = vals.size();
  stats.min = vals.front();
  stats.max = vals.back();
  if (N % 2 == 0) {
    stats.median = 0.5 * (vals[N/2] + vals[N/2 - 1]);
  } else {
    stats.median = vals[N/2];
  }
  stats.total = std::accumulate(vals.begin(), vals.end(), 0.0);
  stats.mean  = stats.total / double(N);

  return stats;
}

template<typename Iterable, typename T>
bool contains(const Iterable &haystack, const T &needle) {
  return std::find(std::begin(haystack), std::end(haystack), needle)
            != std::end(haystack);
}

template <typename S, int R, int O, int MR, int MC>
std::ostream& operator<<(std::ostream& out,
                         const std::vector<Eigen::Matrix<S,R,1,O,MR,MC>> &mat)
{
  if (mat.size() == 0) {
    return out;
  }
  Eigen::MatrixXd Cm(mat.size(), mat[0].size());
  for (size_t i = 0; i < mat.size(); i++) {
    Cm.row(i) = mat[i];
  }
  return out << Cm;
}

inline std::ostream& operator<<(std::ostream& out,
                                const std::vector<double> &vec)
{
  Eigen::VectorXd evec(vec.size());
  for (size_t i = 0; i < vec.size(); i++) {
    evec[i] = vec[i];
  }
  return out << "[" << evec.transpose() << "]";
}


} // end of namespace util

#endif // UTIL_VECTOR_OPS_H
