#ifndef UTIL_STOP_WATCH_H
#define UTIL_STOP_WATCH_H

#include "util/SingleStopWatch.h"

#include <vector>

namespace util {

/** Used to measure time with start() and stop()
 *
 * Simply wraps around SingleStopWatch, capturing the times into a vector
 *
 * Not reentrant.
 */
class StopWatch {
public:
  StopWatch() = default;

  // disable copy
  StopWatch(const StopWatch &other) = delete;
  StopWatch& operator= (const StopWatch &other) = delete;

  // enable move
  StopWatch(StopWatch &&other) = default;
  StopWatch& operator= (StopWatch &&other) = default;

  void clear() { _times.clear(); }
  const std::vector<float>& get_times() const { return _times; }

  bool is_enabled() const             { return _timer.is_enabled(); }
  void disable()                      { _timer.disable(); }
  void enable(bool to_enable = true)  { _timer.enable(to_enable); }
  bool is_started() const             { return _timer.is_started(); }
  bool is_stopped() const             { return _timer.is_stopped(); }
  void start()                        { _timer.start(); }
  void reset()                        { _timer.reset(); }
  void stop() {
    _timer.stop();
    _times.emplace_back(_timer.time());
  }

private:
  SingleStopWatch _timer;
  std::vector<float> _times;
}; // end of class StopWatch

} // end of namespace util

#endif // UTIL_STOP_WATCH_H
