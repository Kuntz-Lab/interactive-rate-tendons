#ifndef UTIL_SINGLE_STOP_WATCH_H
#define UTIL_SINGLE_STOP_WATCH_H

#include <chrono>
#include <stdexcept>

namespace util {

/** Used to measure time with start() and stop()
 *
 * Not reentrant.
 */
class SingleStopWatch {
public:
  SingleStopWatch() = default;

  // disable copy
  SingleStopWatch(const SingleStopWatch &other) = delete;
  SingleStopWatch& operator= (const SingleStopWatch &other) = delete;

  // enable move
  SingleStopWatch(SingleStopWatch &&other) = default;
  SingleStopWatch& operator= (SingleStopWatch &&other) = default;

  float time() const { return _secs; }
  float secs() const { return _secs; }

  bool is_enabled() const { return _enabled; }
  void disable() { _enabled = false; }
  void enable(bool to_enable = true) { _enabled = to_enable; }

  bool is_started() const { return _is_running; }
  bool is_stopped() const { return !_is_running; }

  /** start the timer.  Throws std::runtime_error() if already started. */
  void start() {
    if (is_started()) {
      throw std::runtime_error("Cannot start a started SingleStopWatch");
    }
    _start = std::chrono::steady_clock::now();
    _is_running = true;
  }

  /** stop and reset the timer.  Do not store the timing.
   *
   * Okay to call if not started
   */
  void reset() { _is_running = false; }

  /** stop the timer and store in times vector.  Throws if not started. */
  void stop() {
    if (is_stopped()) {
      throw std::runtime_error("Cannot stop a stopped SingleStopWatch");
    }
    auto end = std::chrono::steady_clock::now();
    auto nanosec = std::chrono::duration_cast<
        std::chrono::nanoseconds>(end - _start).count();
    _secs = nanosec / 1.0e9f; // seconds as a float
    _is_running = false;
  }

private:
  bool _enabled    = true;
  float _secs      = 0.0f;
  bool _is_running = false;
  std::chrono::time_point<std::chrono::steady_clock> _start;
}; // end of class SingleStopWatch

} // end of namespace util

#endif // UTIL_SINGLE_STOP_WATCH_H
