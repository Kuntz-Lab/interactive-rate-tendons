#ifndef UTIL_LIFETIME_TIMER_H
#define UTIL_LIFETIME_TIMER_H

#include <chrono>

namespace util {

/** calculates the lifetime of this object
 *
 * use like so:
 *   float time;
 *   try {
 *     LifetimeTimer(time);
 *     function_to_time();  // time will be set even if this throws
 *   } catch (...) {
 *     std::cout << "Exception thrown after " << time << " seconds"
 *               << std::endl;
 *     throw; // rethrow
 *   }
 *   std::cout << "Function completed successfully after " << time
 *             << " seconds" << std::endl;
 */
template <typename T = float>
struct LifetimeTimer {
  T &time;
  std::chrono::time_point<std::chrono::steady_clock> start;
  LifetimeTimer(T &t) : time(t), start(std::chrono::steady_clock::now()) { }
  ~LifetimeTimer() {
    auto end = std::chrono::steady_clock::now();
    auto nanosec = std::chrono::duration_cast<
        std::chrono::nanoseconds>(end - start).count();
    // set time in the destructor
    time = nanosec / T(1.0e9); // convert to seconds as a floating-point number
  }
};

}

#endif // UTIL_LIFETIME_TIMER_H
