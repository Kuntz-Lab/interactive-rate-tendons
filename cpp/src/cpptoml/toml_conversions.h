#ifndef TOML_CONVERSIONS_H
#define TOML_CONVERSIONS_H

#include <cpptoml/cpptoml.h>
#include <util/openfile_check.h>
#include <util/string_ops.h>

#include <istream>
#include <ostream>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Core>

namespace cpptoml {

using array_ptr = std::shared_ptr<array>;
using table_ptr = std::shared_ptr<table>;
using table_array_ptr = std::shared_ptr<table_array>;

template <typename T>
std::ostream& to_stream(std::ostream& out, std::shared_ptr<T> toml_object) {
  if (!toml_object) { throw std::invalid_argument("null toml object given"); }
  toml_writer writer(out, "  ");
  toml_object->accept(writer);
  return out;
}

template <typename T>
std::string to_string(std::shared_ptr<T> toml_object) {
  std::ostringstream out;
  to_stream(out, toml_object);
  return out.str();
}

inline array_ptr to_toml(const Eigen::VectorXd &vec) {
  auto arr = make_array();
  for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
    arr->push_back(vec[i]);
  }
  return arr;
}

inline array_ptr to_toml(const Eigen::Vector3d &point) {
  auto arr = make_array();
  arr->push_back(point[0]);
  arr->push_back(point[1]);
  arr->push_back(point[2]);
  return arr;
}

inline table_ptr to_toml(const Eigen::Matrix3d &matrix) {
  auto tbl = make_table();
  tbl->insert("row_1", to_toml(Eigen::Vector3d(matrix.row(0))));
  tbl->insert("row_2", to_toml(Eigen::Vector3d(matrix.row(1))));
  tbl->insert("row_3", to_toml(Eigen::Vector3d(matrix.row(2))));
  return tbl;
}

// for all other types, expect a const method to_toml()
template <typename T>
auto to_toml(const T &val) {
  return val.to_toml();
}

template <typename T>
std::enable_if_t<std::is_arithmetic<T>::value ||
                 std::is_same<T, std::string>::value
                 , array_ptr>
to_toml(const std::vector<T> &vec) {
  auto arr = make_array();
  for (auto &val : vec) { arr->push_back(val); }
  return arr;
}

template <typename T>
std::enable_if_t<!std::is_arithmetic<T>::value &&
                 !std::is_same<T, std::string>::value
                 , table_array_ptr>
to_toml(const std::vector<T> &vec) {
  auto tbl_arr = make_table_array();
  for (auto &val : vec) { tbl_arr->push_back(to_toml(val)); }
  return tbl_arr;
}

// functions to handle gzip if present
// pass a function that takes the stream for writing or parsing
// Do not use std::move with the stream, in general it is not safe
void act_fout(const std::string &fname,
              const std::function<void(std::ostream&)> &f);
void act_fin(const std::string &fname,
             const std::function<void(std::istream&)> &f);

inline void to_file(std::shared_ptr<::cpptoml::table> tbl,
                    const std::string &fname)
{
  act_fout(fname, [&tbl](std::ostream &out) {
    ::cpptoml::to_stream(out, tbl);
  });
}

template <typename T>
void to_file(const T &object, const std::string &fname) {
  ::cpptoml::to_file(::cpptoml::to_toml(object), fname);
}

template <typename T>
T from_stream(std::istream &in) {
  auto toml_parser = ::cpptoml::parser(in);
  return T::from_toml(toml_parser.parse());
}

template <typename T>
T from_file(const std::string &fname) {
  T val;
  act_fin(fname, [&val](std::istream &in) {
    val = ::cpptoml::from_stream<T>(in);
  });
  return val;
}

inline Eigen::VectorXd to_vector(array_ptr arr) {
  if (!arr) { throw std::invalid_argument("null array given"); }
  Eigen::VectorXd vec(arr->get().size());
  for (decltype(vec.size()) i = 0; i < vec.size(); i++) {
    auto val = arr->at(i)->as<double>();
    if (!val) {
      throw parse_exception("non-double found in vector");
    }
    vec[i] = val->get();
  }
  return vec;
}

inline Eigen::Vector3d to_point(array_ptr arr) {
  if (!arr) {
    throw std::invalid_argument("null array given");
  }
  if (arr->get().size() < 3) {
    throw std::out_of_range("point array too short (shorter than 3)");
  }
  if (arr->get().size() > 3) {
    throw cpptoml::parse_exception("point array too long (longer than 3)");
  }
  auto v0 = arr->at(0)->as<double>();
  auto v1 = arr->at(1)->as<double>();
  auto v2 = arr->at(2)->as<double>();
  if (!(v0 && v1 && v2)) {
    throw cpptoml::parse_exception("non-double found in point");
  }
  return {
    v0->get(),
    v1->get(),
    v2->get(),
  };
}

inline Eigen::Matrix3d to_matrix(table_ptr tbl) {
  if (!tbl) { throw std::invalid_argument("null table given"); }
  auto row_1 = tbl->get("row_1")->as_array();
  auto row_2 = tbl->get("row_2")->as_array();
  auto row_3 = tbl->get("row_3")->as_array();
  if (!(row_1 && row_2 && row_3)) {
    throw cpptoml::parse_exception("Wrong type detected");
  }
  Eigen::Matrix3d matrix;
  matrix.row(0) = to_point(row_1);
  matrix.row(1) = to_point(row_2);
  matrix.row(2) = to_point(row_3);
  return matrix;
}

template <typename T>
std::vector<T> to_stdvec(array_ptr arr) {
  if (!arr) { throw std::invalid_argument("null array given"); }
  std::vector<T> vec;
  for (auto val : arr->get()) {
    auto as_T = val->as<T>();
    if (!as_T) {
      throw parse_exception("Wrong type found");
    }
    vec.emplace_back(as_T->get());
  }
  return vec;
}

template <typename T>
std::vector<T> to_stdvec(table_array_ptr tbl_arr) {
  if (!tbl_arr) { throw std::invalid_argument("null table array given"); }
  std::vector<T> vec;
  for (auto sub_tbl : tbl_arr->get()) {
    vec.emplace_back(T::from_toml(sub_tbl));
  }
  return vec;
}

} // end of namespace cpptoml

#endif // TOML_CONVERSIONS_H
