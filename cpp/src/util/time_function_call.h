#ifndef UTIL_TIME_FUNCTION_CALL_H
#define UTIL_TIME_FUNCTION_CALL_H

#include "LifetimeTimer.h"

#include <type_traits>
#include <utility>

namespace util {

/** use this variant if you want the return value from f()
 *
 * A lot of effort went into making this usable if the function f() does not
 * have a return type (i.e. void).
 */
template <typename Func, typename...Args>
decltype(auto) time_function_call(Func &&f, float &time_out, Args&& ...args)
{
  LifetimeTimer timer(time_out);
  return std::forward<Func>(f)(std::forward<Args>(args)...);
}

/// time a single invocation of f(), throwing away its return value
template <typename Func, typename...Args>
float time_function_call(Func &&f, Args&&...args) {
  float time;
  time_function_call(std::forward<Func>(f), time, std::forward<Args>(args)...);
  return time;
}

} // end of namespace util

#endif // UTIL_TIME_FUNCTION_CALL_H
