#ifndef UTIL_OPENFILE_CHECK_H
#define UTIL_OPENFILE_CHECK_H

#include <fstream>
#include <iostream>
#include <string>

namespace util {

template <typename T>
void openfile_check(T& filestream, const std::string &filename) {
    // turn on exceptions on failure
    filestream.exceptions(std::ios::failbit);

  try {
    // opening will throw if the file does not exist or is not readable
    filestream.open(filename);
  } catch (std::exception &ex) {
    std::cerr << "Error: could not open " << filename << "\n"
              << "  " << ex.what() << std::endl;
    throw;
  }

    // turn off all exceptions (back to the default behavior)
    filestream.exceptions(std::ios::goodbit);
}

template <typename T>
void openfile_check(T& filestream, const std::string &filename,
                    std::ios_base::openmode mode)
{
  // turn on exceptions on failure
  filestream.exceptions(std::ios::failbit);

  try {
    // opening will throw if the file does not exist or is not readable
    filestream.open(filename, mode);
  } catch (std::exception &ex) {
    std::cerr << "Error: could not open " << filename << "\n"
              << "  " << ex.what() << std::endl;
    throw;
  }

  // turn off all exceptions (back to the default behavior)
  filestream.exceptions(std::ios::goodbit);
}

} // end of namespace util

#endif // UTIL_OPENFILE_CHECK_H
