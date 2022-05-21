#ifndef UTIL_OMPL_LOGGING_H
#define UTIL_OMPL_LOGGING_H

#include <ompl/util/Console.h>

#include <stdexcept>

namespace util {

inline ompl::msg::LogLevel ompl_log_level_from_name(const std::string &name) {
  if      (name == "DEV2" ) { return ompl::msg::LOG_DEV2;  }
  else if (name == "DEV1" ) { return ompl::msg::LOG_DEV1;  }
  else if (name == "DEBUG") { return ompl::msg::LOG_DEBUG; }
  else if (name == "INFO" ) { return ompl::msg::LOG_INFO;  }
  else if (name == "WARN" ) { return ompl::msg::LOG_WARN;  }
  else if (name == "ERROR") { return ompl::msg::LOG_ERROR; }
  else if (name == "NONE" ) { return ompl::msg::LOG_NONE;  }
  else {
    throw std::invalid_argument("Unrecognized log level name: " + name);
  }
}

inline void set_ompl_log_level(ompl::msg::LogLevel level) {
  ompl::msg::setLogLevel(level);
}

inline void set_ompl_log_level(const std::string &name) {
  set_ompl_log_level(ompl_log_level_from_name(name));
}

} // end of namespace util

#endif // UTIL_OMPL_LOGGING_H
