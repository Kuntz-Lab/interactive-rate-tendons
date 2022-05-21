#include "json_io.h"
#include "openfile_check.h"

#include <3rdparty/nlohmann/json.hpp>

#include <algorithm>  // for std::transform()
#include <cctype>     // for std::tolower()
#include <functional> // for std::function
#include <fstream>    // for std::ifstream and std::ofstream
#include <iostream>   // for std::istream and std::ostream
#include <iterator>   // for std::distance
#include <optional>   // for std::optional and std::nullopt
#include <string>     // for std::string
#include <vector>     // for std::vector

namespace util {

namespace {

using ReaderFunc = std::function<json(std::istream&)>;
using WriterFunc = std::function<void(const json&, std::ostream&)>;

ReaderFunc get_reader(JsonFormat type) {
  switch (type) {
    case JsonFormat::JSON:
      return [](std::istream& in) { json data; in >> data; return data; };
    case JsonFormat::BSON:
      return [](std::istream& in) { return json::from_bson   (in);      };
    case JsonFormat::CBOR:
      return [](std::istream& in) { return json::from_cbor   (in);      };
    case JsonFormat::MSGPACK:
      return [](std::istream& in) { return json::from_msgpack(in);      };
    case JsonFormat::UBJSON:
      return [](std::istream& in) { return json::from_ubjson (in);      };
    default:
      throw std::invalid_argument("Unsupported json type in get_reader(): "
                                  + std::to_string(int(type)));
  }
}

WriterFunc get_writer(JsonFormat type) {
  switch (type) {
    case JsonFormat::JSON:
      return [](const json &j, std::ostream& out) { out << j;                 };
    case JsonFormat::BSON:
      return [](const json &j, std::ostream& out) { json::to_bson   (j, out); };
    case JsonFormat::CBOR:
      return [](const json &j, std::ostream& out) { json::to_cbor   (j, out); };
    case JsonFormat::MSGPACK:
      return [](const json &j, std::ostream& out) { json::to_msgpack(j, out); };
    case JsonFormat::UBJSON:
      return [](const json &j, std::ostream& out) { json::to_ubjson (j, out); };
    default:
      throw std::invalid_argument("Unsupported json type in get_reader(): "
                                  + std::to_string(int(type)));
  }
}

// not thread safe
inline const std::vector<std::string>& extension_map() {
  static std::vector<std::string> mapping;
  if (mapping.empty()) {
    mapping.resize(int(JsonFormat::count));
    mapping[int(JsonFormat::JSON   )] = "json";
    mapping[int(JsonFormat::BSON   )] = "bson";
    mapping[int(JsonFormat::CBOR   )] = "cbor";
    mapping[int(JsonFormat::MSGPACK)] = "msgpack";
    mapping[int(JsonFormat::UBJSON )] = "ubjson";
  }
  return mapping;
}

inline std::string file_extension(const std::string &filename) {
  auto last_dot = filename.find_last_of('.');
  auto ending_beginning = (last_dot == std::string::npos) ?
                          std::string::npos : last_dot + 1;
  auto ending = filename.substr(ending_beginning);
  return ending;
}

inline std::string to_lower(std::string s) {
  std::string before = s;
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return s;
}

} // end of unnamed namespace

std::string json_extension(JsonFormat type) {
  const auto &mapping = extension_map();
  if (size_t(type) >= mapping.size()) {
    throw std::runtime_error("Unsupported json type " + std::to_string(int(type)));
  }
  return mapping[size_t(type)];
}

std::optional<JsonFormat> try_json_format(const std::string &extension) {
  const auto &mapping = extension_map();
  auto found = std::find(mapping.begin(), mapping.end(), to_lower(extension));
  if (found == mapping.end()) {
    return std::nullopt;
  }
  auto index = std::distance(mapping.begin(), found);
  return JsonFormat(index);
}

JsonFormat json_format(const std::string &extension) {
  auto try_format = try_json_format(extension);
  if (!try_format.has_value()) {
    throw std::invalid_argument("Unsupported json extension: " + extension);
  }
  return *try_format;
}

std::optional<JsonFormat> try_json_format_from_path(const std::string &filename) {
  return try_json_format(file_extension(filename));
}

JsonFormat json_format_from_path(const std::string &filename) {
  return json_format(file_extension(filename));
}

json read_json(const std::string &filename) {
  auto type = try_json_format_from_path(filename);
  if (type.has_value()) {
    return read_json(filename, *type);
  }
  throw std::invalid_argument("Could not determine json file type from " + filename);
}

json read_json(const std::string &filename, JsonFormat type) {
  std::ifstream in;
  openfile_check(in, filename);
  return read_json(in, type);
}

json read_json(std::istream &in, JsonFormat type) {
  auto reader = get_reader(type);
  return reader(in);
}

void write_json(const std::string &filename, const json &data) {
  auto type = json_format_from_path(filename);
  write_json(filename, data, type);
}

void write_json(const std::string &filename, const json &data, JsonFormat type) {
  std::ofstream out;
  openfile_check(out, filename);
  write_json(out, data, type);
}

void write_json(std::ostream &out, const json &data, JsonFormat type) {
  auto writer = get_writer(type);
  return writer(data, out);
}

} // end of namespace util
