#ifndef UTIL_STRING_OPS_H
#define UTIL_STRING_OPS_H

#include <string>

namespace util {

inline bool contains(const std::string &str, const std::string &needle) {
  return std::string::npos != str.find(needle);
}

inline bool startswith(const std::string &str, const std::string &beginning) {
  if (str.length() < beginning.length()) {
    return false;
  }
  return 0 ==
      str.compare(
          0,
          beginning.length(),
          beginning);
}

inline bool endswith(const std::string &str, const std::string &ending) {
  if (str.length() < ending.length()) {
    return false;
  }
  return 0 ==
      str.compare(
          str.length() - ending.length(),
          ending.length(),
          ending);
}

} // end of namespace util

#endif // UTIL_STRING_OPS_H
