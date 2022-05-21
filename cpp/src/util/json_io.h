#ifndef UTIL_JSON_IO_H
#define UTIL_JSON_IO_H

//#include <3rdparty/nlohmann/json.hpp>

#include <iostream>
#include <optional>
#include <string>
#include <map>
#include <vector>
#include <memory>

#include <cstdint>

// forward declarations
namespace nlohmann {

template<typename T, typename SFINAE>
struct adl_serializer;

template<template<typename U, typename V, typename... Args> class ObjectType,
         template<typename U, typename... Args> class ArrayType,
         class StringType,
         class BooleanType,
         class NumberIntegerType,
         class NumberUnsignedType,
         class NumberFloatType,
         template<typename U> class AllocatorType,
         template<typename T, typename SFINAE> class JSONSerializer,
         class BinaryType>
class basic_json;

using json = basic_json<std::map,
                        std::vector,
                        std::string,
                        bool,
                        std::int64_t,
                        std::uint64_t,
                        double,
                        std::allocator,
                        adl_serializer,
                        std::vector<std::uint8_t>>;

} // end of namespace nlohmann

namespace util {

// make an alias in the util namespace
using json = nlohmann::json;

enum class JsonFormat {
  JSON     = 0, // plain text json
  BSON     = 1, // Binary JSON
  CBOR     = 2, // Concise Binary Object Representation
  MSGPACK  = 3, // MessagePack
  UBJSON   = 4, // Universal Binary JSON
  count
};

/** Return the extension used for each type (in lowercase)
 *
 * Example:
 *   assert("json" == json_extension(JsonFormat::JSON));
 *   assert("bson" == json_extension(JsonFormat::BSON));
 */
std::string json_extension(JsonFormat type);

/** Return the format type for the given extension
 *
 * It returns a std::optional to be able to indicate an unsupported file
 * extension, which is done  by return std::nullopt (i.e., and empty optional
 * object).
 */
std::optional<JsonFormat> try_json_format(const std::string &extension);

/// Same as try_json_format() but this throws std::invalid_argument if invalid
JsonFormat json_format(const std::string &extension);

/// Same as try_json_format() but first extracts file extension
std::optional<JsonFormat> try_json_format_from_path(const std::string &filename);

/// Same as json_format() but first extracts file extension
JsonFormat json_format_from_path(const std::string &filename);

// read the json object
json read_json(const std::string &filename); // use file extension to guess type
json read_json(const std::string &filename, JsonFormat type);
json read_json(std::istream &in, JsonFormat type);

// write the json object to a file
void write_json(const std::string &filename, const json &data); // use file extension
void write_json(const std::string &filename, const json &data, JsonFormat type);
void write_json(std::ostream &out, const json &data,
                JsonFormat type = JsonFormat::JSON);

} // end of namespace util

#endif // UTIL_JSON_IO_H
