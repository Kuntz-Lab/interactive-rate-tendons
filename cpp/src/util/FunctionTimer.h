#ifndef UTIL_FUNCTION_TIMER_H
#define UTIL_FUNCTION_TIMER_H

#include "time_function_call.h"

#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace util {

/** Thread-safe function timer */
class FunctionTimer {
public:
  FunctionTimer() = default;

  // disable copy
  FunctionTimer(const FunctionTimer &other) = delete;
  FunctionTimer& operator= (const FunctionTimer &other) = delete;

  // move
  FunctionTimer(FunctionTimer &&other)
    : _enabled(std::move(other._enabled))
    , _times(std::move(other._times))
    , _times_mutex() { }
  FunctionTimer& operator= (FunctionTimer &&other) {
    _enabled = std::move(other._enabled);
    _times = std::move(other._times);
    return *this;
  }

  void clear() { _times.clear(); }
  const std::vector<float>& get_times() const { return _times; }

  bool is_enabled() const { return _enabled; }
  void disable() { _enabled = false; }
  void enable(bool to_enable = true) { _enabled = to_enable; }

  /// Returns a new function that behaves like f, but will be timed
  template <typename Func>
  decltype(auto) wrap(Func &&f) {
    return
      [ this, f = std::forward<Func>(f) ]
      (auto &&... args) mutable -> decltype(auto) {
        return this->time(
            std::forward<Func>(f),
            std::forward<decltype(args)>(args)...
        );
      };
  }

  /// Time f(args...) and return its value while storing its timing
  template <typename Func, typename...Args>
  decltype(auto) time(Func &&f, Args&&... args) {
    if (!_enabled) {
      return std::forward<Func>(f)(std::forward<Args>(args)...);
    }

    // struct for appending timing results in destructor, even if exception is
    // thrown
    struct AppendTiming {
      std::mutex &_my_mutex;
      const float &_my_timing;
      std::vector<float> &_my_times;

      AppendTiming(std::mutex &my_mutex, const float &my_timing,
                   std::vector<float> &my_times)
        : _my_mutex(my_mutex), _my_timing(my_timing), _my_times(my_times) { }

      // magic happens here: acquire mutex and append timing
      ~AppendTiming() {
        std::lock_guard<std::mutex> guard(_my_mutex);
        _my_times.emplace_back(_my_timing);
      }
    };

    float timing;
    AppendTiming appender(_times_mutex, timing, this->_times);
    return time_function_call(
        std::forward<Func>(f),
        timing,
        std::forward<Args>(args)...
    );
  }

private:
  bool _enabled = true;
  std::vector<float> _times;
  std::mutex _times_mutex;
}; // end of class FunctionTimer

} // end of namespace util

#endif // UTIL_FUNCTION_TIMER_H
