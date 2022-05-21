/*********************************************************************
* Copied from OMPL 1.5.0 source code from LazyPRM.cpp on 01 Dec 2020
*
*   src/ompl/geometric/planners/prm/src/LazyPRM.cpp
*
* Git repo:
*
*   https://github.com/ompl/ompl
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Willow Garage
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ryan Luna, Henning Kayser
 * Modified by: Michael Bentley
 */

#include "VoxelCachedLazyPRM.h"

#include "AbstractVoxelMotionValidator.h"
#include "AbstractVoxelValidityChecker.h"
#include "AStarGoalVisitor.h"
#include "motion-planning/io/RoadmapParser.h"
#include "motion-planning/io/RoadmapWriter.h"
#include <collision/VoxelOctree.h>
#include <cpptoml/cpptoml.h>
#include <cpptoml/toml_conversions.h>
#include <tip-control/Controller.h>
#include <util/FunctionTimer.h>
#include <util/json_io.h>
#include <util/openfile_check.h>
#include <util/vector_ops.h>
#include <util/SingleStopWatch.h>

#include <3rdparty/nlohmann/json.hpp>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Time.h>

#include <Eigen/Core>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <cstdio>  // for std::remove()

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;
namespace otime = ompl::time;
namespace bio = boost::iostreams;

namespace E = Eigen;

using motion_planning::io::ParsedType;
using motion_planning::io::ParsedVertex;
using motion_planning::io::ParsedEdge;
using motion_planning::io::RoadmapParser;
using motion_planning::io::RoadmapWriter;

namespace std {
// to be able to use range-based for loops with graph vertices and edges
template <typename Iter>
Iter begin(const ::std::pair<Iter, Iter> &iter_pair) {
  return iter_pair.first;
}

template <typename Iter>
Iter end(const ::std::pair<Iter, Iter> &iter_pair) {
  return iter_pair.second;
}
} // end of namespace std

namespace magic {

/** \brief The number of nearest neighbors to consider by
 * default in the construction of the PRM roadmap
 */
static const uint32_t DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

/** \brief When optimizing solutions with lazy planners, this is the minimum
 *  number of path segments to add before attempting a new optimized solution
 *  extraction
 */
static const uint32_t MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;

} // end of namespace magic

// for use with std::visit
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

namespace motion_planning {

namespace {

class LazyJsonParser : public RoadmapParser {
public:

  LazyJsonParser(const nlohmann::json &data)
    : _vertices(nullptr), _edges(nullptr), _voxels(nullptr), v_idx(0), e_idx(0)
  {
    const nlohmann::json *roadmap = &data;
    if (data.contains("VoxelCachedLazyPRM_roadmap")) {
      roadmap = &data.at("VoxelCachedLazyPRM_roadmap");
    }
    _vertices = &roadmap->at("vertices");
    _edges    = &roadmap->at("edges");
  }

  ParsedType next() override {
    _voxels = nullptr;

    // see if we're on the vertices still
    if (v_idx < _vertices->size()) {
      auto &vertex = _vertices->at(v_idx);
      vertex["index"].get_to(_current_vertex.index);
      vertex["state"].get_to(_current_vertex.state);
      _current_vertex.tip_pos.reset();
      _current_vertex.voxels.reset();
      if (vertex.contains("tip_pos")) {
        auto &tip = vertex["tip_pos"];
        _current_vertex.tip_pos.emplace(
            tip[0].get<double>(),
            tip[1].get<double>(),
            tip[2].get<double>()
            );
      }
      if (vertex.contains("voxels")) { _voxels = &vertex["voxels"]; }
      v_idx++;
      _current_type = ParsedType::VERTEX;
      return _current_type;
    }

    // see if we're on the edges still
    if (e_idx < _edges->size()) {
      auto &edge = _edges->at(e_idx);
      edge["source"].get_to(_current_edge.source);
      edge["target"].get_to(_current_edge.target);
      edge["weight"].get_to(_current_edge.weight);
      _current_edge.voxels.reset();
      if (edge.contains("voxels")) { _voxels = &edge["voxels"]; }
      e_idx++;
      _current_type = ParsedType::EDGE;
      return _current_type;
    }

    // we're done
    _current_type = ParsedType::DONE;
    return _current_type;
  }

  virtual bool handles_voxels_lazily() const override { return true; }
  virtual void populate_voxels() override {
    if (_voxels == nullptr) { return; }
    auto voxels = std::make_shared<collision::VoxelOctree>(
            collision::VoxelOctree::from_json(*_voxels));
    switch(_current_type) {
      case ParsedType::VERTEX: _current_vertex.voxels = voxels; break;
      case ParsedType::EDGE:   _current_edge.voxels   = voxels; break;
      default:
        throw std::runtime_error("unsupported ParsedType");
    }
  }

protected:
  const nlohmann::json *_vertices;
  const nlohmann::json *_edges;
  const nlohmann::json *_voxels; // current voxels object
  size_t v_idx = 0;
  size_t e_idx = 0;
};

// does all the parsing at the beginning
class ParallelJsonParser : public RoadmapParser {
public:

  ParallelJsonParser(const nlohmann::json &data) {
    const nlohmann::json *roadmap = &data;
    if (data.contains("VoxelCachedLazyPRM_roadmap")) {
      roadmap = &data.at("VoxelCachedLazyPRM_roadmap");
    }
    auto json_vertices = roadmap->at("vertices");
    auto json_edges    = roadmap->at("edges");

    _vertices.resize(json_vertices.size());
    _edges   .resize(json_edges.size());

    // parse all of the vertices
    #pragma omp parallel for
    for (size_t i = 0; i < json_vertices.size(); ++i) {
      auto &json_vertex = json_vertices[i];
      auto &vertex = _vertices[i];
      json_vertex["index"].get_to(vertex.index);
      json_vertex["state"].get_to(vertex.state);
      if (json_vertex.contains("tip_pos")) {
        auto &tip = json_vertex["tip_pos"];
        vertex.tip_pos.emplace(
            tip[0].get<double>(),
            tip[1].get<double>(),
            tip[2].get<double>()
            );
      }
      if (json_vertex.contains("voxels")) {
        vertex.voxels = std::make_shared<collision::VoxelOctree>(
            collision::VoxelOctree::from_json(json_vertex["voxels"]));
      }
    }

    // parse all of the edges
    #pragma omp parallel for
    for (size_t i = 0; i < json_edges.size(); ++i) {
      auto &json_edge = json_edges[i];
      auto &edge = _edges[i];
      json_edge["source"].get_to(edge.source);
      json_edge["target"].get_to(edge.target);
      json_edge["weight"].get_to(edge.weight);
      if (json_edge.contains("voxels")) {
        edge.voxels = std::make_shared<collision::VoxelOctree>(
            collision::VoxelOctree::from_json(json_edge["voxels"]));
      }
    }
  }

  ParsedType next() override {
    // see if we're on the vertices still
    if (v_idx < _vertices.size()) {
      _current_vertex = _vertices.at(v_idx);
      _current_type = ParsedType::VERTEX;
      v_idx++;
      return _current_type;
    }

    // see if we're on the edges still
    if (e_idx < _edges.size()) {
      _current_edge = _edges.at(e_idx);
      _current_type = ParsedType::EDGE;
      e_idx++;
      return _current_type;
    }

    // we're done
    _current_type = ParsedType::DONE;
    return _current_type;
  }

protected:
  std::vector<ParsedVertex> _vertices;
  std::vector<ParsedEdge>   _edges;
  size_t v_idx = 0;
  size_t e_idx = 0;
}; // end of class ParallelJsonParser

class LazyTomlParser : public RoadmapParser {
public:

  LazyTomlParser(cpptoml::table_ptr tbl)
    : _vertices(), _edges(), _voxels(), v_idx(0), e_idx(0)
  {
    if (!tbl) { throw std::invalid_argument("null table given"); }

    // check if the container is inside of tbl, otherwise assume we are already in
    if (tbl->contains("VoxelCachedLazyPRM_roadmap")) {
      auto container = tbl->get("VoxelCachedLazyPRM_roadmap")->as_table();
      if (!container) {
        throw cpptoml::parse_exception (
            "Wrong type detected for 'VoxelCachedLazyPRM_roadmap': not a table");
      }
      tbl = container;
    }
    _vertices = tbl->get("vertices")->as_table_array();
    _edges    = tbl->get("edges")->as_table_array();
  }

  ParsedType next() override {
    _voxels.reset();

    // see if we're on the vertices still
    if (v_idx < _vertices->size()) {
      auto &vertex = _vertices->get()[v_idx];
      _current_vertex.index = vertex->get("index")->as<int64_t>()->get();
      _current_vertex.state = cpptoml::to_stdvec<double>(
          vertex->get("state")->as_array());
      _current_vertex.tip_pos.reset();
      _current_vertex.voxels.reset();
      if (vertex->contains("tip_pos")) {
        auto tip = vertex->get("tip_pos")->as_array();
        _current_vertex.tip_pos.emplace(
            tip->at(0)->as<double>()->get(),
            tip->at(1)->as<double>()->get(),
            tip->at(2)->as<double>()->get()
            );
      }
      if (vertex->contains("voxels")) {
        _voxels = vertex->get("voxels")->as_table();
      }
      v_idx++;
      _current_type = ParsedType::VERTEX;
      return _current_type;
    }

    // see if we're on the edges still
    if (e_idx < _edges->size()) {
      auto &edge = _edges->get()[e_idx];
      _current_edge.source = edge->get("source")->as<int64_t>()->get();
      _current_edge.target = edge->get("target")->as<int64_t>()->get();
      _current_edge.weight = edge->get("weight")->as<double>() ->get();
      _current_edge.voxels.reset();
      if (edge->contains("voxels")) {
        _voxels = edge->get("voxels")->as_table();
      }
      e_idx++;
      _current_type = ParsedType::EDGE;
      return _current_type;
    }

    // we're done
    _current_type = ParsedType::DONE;
    return _current_type;
  }

  virtual bool handles_voxels_lazily() const override { return true; }
  virtual void populate_voxels() override {
    if (!_voxels) { return; }
    auto voxels = std::make_shared<collision::VoxelOctree>(
            collision::VoxelOctree::from_toml(_voxels));
    switch(_current_type) {
      case ParsedType::VERTEX: _current_vertex.voxels = voxels; break;
      case ParsedType::EDGE:   _current_edge.voxels   = voxels; break;
      default:
        throw std::runtime_error("unsupported ParsedType");
    }
  }

protected:
  cpptoml::table_array_ptr _vertices;
  cpptoml::table_array_ptr _edges;
  cpptoml::table_ptr       _voxels;
  size_t v_idx = 0;
  size_t e_idx = 0;
}; // end of class LazyTomlParser

/** parser for my custom file type for the roadmap */
class ParallelDatParser : public RoadmapParser {
public:

  ParallelDatParser(std::istream &in,
            util::JsonFormat format = util::JsonFormat::JSON)
  {
    if (!in) { return; }
    std::string line;
    std::vector<ParsedVertex> vertices;
    std::vector<ParsedEdge>   edges;

    // parse the whole input, populating the vertices and edges
    #pragma omp parallel
    {
      #pragma omp single
      while(std::getline(in, line)) {
        #pragma omp task firstprivate(line) shared(vertices, edges)
        {
          nlohmann::json obj;
          {
            std::istringstream line_in(line);
            obj = util::read_json(line_in, format);
          }

          // parse the vertex
          if (obj["type"] == "vertex") {
            ParsedVertex v;
            obj["index"].get_to(v.index);
            obj["state"].get_to(v.state);
            if (obj.contains("tip_pos")) {
              auto &tip = obj["tip_pos"];
              v.tip_pos.emplace(
                  tip[0].get<double>(),
                  tip[1].get<double>(),
                  tip[2].get<double>()
                  );
            }
            if (obj.contains("voxels")) {
              v.voxels = std::make_shared<collision::VoxelOctree>(
                  collision::VoxelOctree::from_json(obj["voxels"]));
            }
            #pragma omp critical
            vertices.emplace_back(std::move(v));
          }

          // parse the edge
          if (obj["type"] == "edge") {
            ParsedEdge e;
            obj["source"].get_to(e.source);
            obj["target"].get_to(e.target);
            obj["weight"].get_to(e.weight);
            e.voxels.reset();
            if (obj.contains("voxels")) {
              e.voxels = std::make_shared<collision::VoxelOctree>(
                  collision::VoxelOctree::from_json(obj["voxels"]));
            }
            #pragma omp critical
            edges.emplace_back(std::move(e));
          }

          //throw std::runtime_error("unsupported object type detected");
        } // end of omp task
      } // end of while
    } // end of omp parallel

    _vertices = vertices;
    _edges    = edges;
  }

  ParsedType next() override {
    // see if we're on the vertices still
    if (v_idx < _vertices.size()) {
      _current_vertex = _vertices.at(v_idx);
      _current_type = ParsedType::VERTEX;
      v_idx++;
      return _current_type;
    }

    // see if we're on the edges still
    if (e_idx < _edges.size()) {
      _current_edge = _edges.at(e_idx);
      _current_type = ParsedType::EDGE;
      e_idx++;
      return _current_type;
    }

    // we're done
    _current_type = ParsedType::DONE;
    return _current_type;
  }

private:
  std::vector<ParsedVertex> _vertices;
  std::vector<ParsedEdge>   _edges;
  size_t v_idx = 0;
  size_t e_idx = 0;
}; // end of class ParallelDatParser

/** parser for my custom file type for the roadmap */
class LazyDatParser : public RoadmapParser {
public:

  LazyDatParser(std::istream &in,
            util::JsonFormat format = util::JsonFormat::JSON)
    : _in(in), _format(format), _voxels()
  {}

  ParsedType next() override {
    using nlohmann::json;

    _voxels = {}; // empty previous voxels object

    if (!_in) {
      _current_type = ParsedType::DONE;
      return _current_type;
    }

    std::string line;
    if (!std::getline(_in, line)) {
      _current_type = ParsedType::DONE;
      return _current_type;
    }

    // we have each line as a separate json object
    json obj;
    {
      std::istringstream line_in(line);
      obj = util::read_json(line_in, _format);
    }

    // parse the vertex
    if (obj["type"] == "vertex") {
      obj["index"].get_to(_current_vertex.index);
      obj["state"].get_to(_current_vertex.state);
      _current_vertex.tip_pos.reset();
      _current_vertex.voxels.reset();
      if (obj.contains("tip_pos")) {
        auto &tip = obj["tip_pos"];
        _current_vertex.tip_pos.emplace(
            tip[0].get<double>(),
            tip[1].get<double>(),
            tip[2].get<double>()
            );
      }
      if (obj.contains("voxels")) { _voxels = obj["voxels"]; }
      _current_type = ParsedType::VERTEX;
      return _current_type;
    }

    // parse the edge
    if (obj["type"] == "edge") {
      obj["source"].get_to(_current_edge.source);
      obj["target"].get_to(_current_edge.target);
      obj["weight"].get_to(_current_edge.weight);
      _current_edge.voxels.reset();
      if (obj.contains("voxels")) { _voxels = obj["voxels"]; }
      _current_type = ParsedType::EDGE;
      return _current_type;
    }

    throw std::runtime_error("unsupported object type detected");
  }

  virtual bool handles_voxels_lazily() const override { return true; }
  virtual void populate_voxels() override {
    if (_voxels.empty()) { return; }
    auto voxels = std::make_shared<collision::VoxelOctree>(
            collision::VoxelOctree::from_json(_voxels));
    switch(_current_type) {
      case ParsedType::VERTEX: _current_vertex.voxels = voxels; break;
      case ParsedType::EDGE:   _current_edge.voxels   = voxels; break;
      default:
        throw std::runtime_error("unsupported ParsedType");
    }
  }

private:
  std::istream &_in;
  util::JsonFormat _format;
  nlohmann::json _voxels;
}; // end of class LazyDatParser

/** output to my custom dat roadmap type in a streaming fashion
 *
 * Example:
 *   std::ofstream fout("roadmap.dat");
 *   DatStreamer out(fout);
 *   out.write_vertex(idx, state, tip_pos, voxels);
 *   out.write_edge(source, target, weight, voxels);
 */
class DatStreamer : public RoadmapWriter {
public:
  DatStreamer(std::ostream &out,
              util::JsonFormat format = util::JsonFormat::JSON)
    : _out(out), _format(format)
  {}

  virtual void write_vertex(
      uint32_t index,
      const std::vector<double> state,
      motion_planning::VoxelCachedLazyPRM::TipPosition tip_pos,
      motion_planning::VoxelCachedLazyPRM::VoxelPtr voxels)
    override
  {
    using nlohmann::json;
    json vertex;
    vertex["type"] = "vertex";
    vertex["index"] = index;
    vertex["state"] = state;
    if (tip_pos) {
      vertex["tip_pos"] = {(*tip_pos)[0], (*tip_pos)[1], (*tip_pos)[2]};
    }
    if (voxels) {
      vertex["voxels"] = voxels->to_json();
    }
    util::write_json(_out, vertex, _format);
    _out << std::endl;
  }

  virtual void write_edge(
      uint32_t source,
      uint32_t target,
      double weight,
      motion_planning::VoxelCachedLazyPRM::VoxelPtr voxels)
    override
  {
    using nlohmann::json;
    json edge;
    edge["type"]   = "edge";
    edge["source"] = source;
    edge["target"] = target;
    edge["weight"] = weight;
    if (voxels) {
      edge["voxels"] = voxels->to_json();
    }
    util::write_json(_out, edge, _format);
    _out << std::endl;
    _out.flush();
  }

private:
  std::ostream &_out;
  util::JsonFormat _format;
}; // end of class DatStreamer

// write the inner voxel object, just the blocks
template <typename Archive>
void serialize_inner(Archive &ar, const collision::VoxelOctree &v) {
  ar & uint32_t(v.nblocks());
  v.visit_leaves([&ar](size_t bx, size_t by, size_t bz, uint64_t val) {
      std::array<uint8_t, 3> bxyz = { bx, by, bz };
    ar & bxyz & uint64_t(val);
  });
}

// read the inner voxel object, just the blocks
template <typename Archive>
void serialize_inner(Archive &ar, collision::VoxelOctree &v) {
  v = v.empty_copy();
  uint32_t Nblocks;
  ar & Nblocks;
  for (; Nblocks --> 0;) {
    std::array<uint8_t, 3> bxyz;
    uint64_t val;
    ar & bxyz & val;
    v.set_block(bxyz[0], bxyz[1], bxyz[2], val);
  }
}

// write any trivially copyable type
// (including bool, char types, int, and floating-point types)
template <typename BinWriter, typename T,
  typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
void binary_write(BinWriter &out, const T &v) {
  out.write(v);
}

// write std::array<T, N>
// TODO: handle when T is not trivially copyable
template <typename BinWriter, typename T, std::size_t N>
void binary_write(BinWriter &out, const std::array<T, N> &a) {
  out.write(a.data(), a.size());
}

// write std::optional<T>
template <typename BinWriter, typename T>
void binary_write(BinWriter &out, const std::optional<T> &v) {
  const bool exists {v};
  out.write(exists);
  if (exists) {
    binary_write(out, *v);
  }
}

// write std::vector<T> (only supporting up to 32-bit sizes)
template <typename BinWriter, typename T, typename SizeType = uint32_t>
void binary_write(BinWriter &out, const std::vector<T> &v) {
  out.write(static_cast<SizeType>(v.size()));
  out.write(v.data(), v.size());
}

// write Eigen::Matrix<T, N, M> for N,M > 0
template <typename BinWriter, typename T, int N, int M,
    typename = std::enable_if_t<(N > 0 && M > 0)>>
void binary_write(BinWriter &out, const Eigen::Matrix<T, N, M> &v) {
  static_assert(N > 0 && M > 0, "Only can input static sized matrices");
  out.write(v.data(), N*M);
}

// read any trivially copyable type
// (including bool, char types, int, and floating-point types)
template <typename BinReader, typename T,
  typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
void binary_read(BinReader &in, T &v) {
  in.read(&v);
}

// read std::array<T, N>
// TODO: handle when T is not trivially copyable
template <typename BinReader, typename T, std::size_t N>
void binary_read(BinReader &in, std::array<T, N> &a) {
  in.read(a.data(), a.size());
}

// read std::optional<T>
template <typename BinReader, typename T>
void binary_read(BinReader &in, std::optional<T> &v) {
  bool exists = false;
  in.read(&exists);
  if (exists) {
    T val{};
    binary_read(in, val);
    v = std::move(val);
  }
}

// read std::vector<T> (only supporting up to 32-bit sizes)
template <typename BinReader, typename T>
void binary_read(BinReader &in, std::vector<T> &v) {
  uint32_t count;
  in.read(&count);
  v.resize(count);
  in.read(v.data(), count);
}

// Eigen::Matrix<T, N, M>
template <typename BinReader, typename T, int N, int M,
    typename = std::enable_if_t<(N > 0 && M > 0)>>
void binary_read(BinReader &in, Eigen::Matrix<T, N, M> &v) {
  static_assert(N > 0 && M > 0, "Only can input static sized matrices");
  in.read(v.data(), N*M);
}

template <typename BinReader>
void binary_read_inner(BinReader &in, collision::VoxelOctree &v) {
  v = v.empty_copy();
}

class BinaryIFStream {
public:
  BinaryIFStream(std::string fname)
    : _fname(std::move(fname))
    , _in(fopen(_fname.c_str(), "rb"))
  {
    if (!_in) {
      throw std::runtime_error("Failed to open " + _fname);
    }
  }

  template <typename T>
  void read(T *dest, std::size_t count = 1) {
    static_assert(std::is_trivially_copyable_v<T>);
    auto num_read = fread((void*)dest, sizeof(T), count, _in);
    if (num_read != count) {
      std::ostringstream msg;
      msg << "Failed to read "
          << count << " object(s) of size " << sizeof(T)
          << " from '" << _fname << "'";
      throw std::runtime_error(msg.str());
    }
  }

  template <typename T>
  BinaryIFStream& operator>>(T &v) { binary_read(*this, v); return *this; }

  template <typename T>
  BinaryIFStream& operator&(T &v) { return (*this) >> v; }

private:
  std::string _fname;
  std::FILE* _in;
};

/** Simple binary input string stream.  content must live longer than this */
class BinaryISStream {
public:
  BinaryISStream(const char *content, std::size_t size)
    : _head(content) , _n_remaining(size) {}

  BinaryISStream(const BinaryISStream &other) = default;
  BinaryISStream(BinaryISStream &&other) = default;

  const char* head() const { return _head; }
  std::size_t n_remaining() const { return _n_remaining; }

  BinaryISStream& operator= (BinaryISStream &other) {
    this->_head        = other._head;
    this->_n_remaining = other._n_remaining;
    return *this;
  }

  template <typename T>
  void read(T *dest, std::size_t count = 1) {
    static_assert(std::is_trivially_copyable_v<T>);
    const auto n_bytes = count * sizeof(T);
    if (n_bytes > _n_remaining) {
      throw std::runtime_error("Asked for too many bytes from BinaryISStream");
    }
    std::memcpy(dest, _head, n_bytes);
    _n_remaining -= n_bytes;
    _head += n_bytes;
  }

  template <typename T>
  BinaryISStream& operator>>(T &v) { binary_read(*this, v); return *this; }

  template <typename T>
  BinaryISStream& operator&(T &v) { return (*this) >> v; }

private:
  const char *_head;
  std::size_t _n_remaining;
};

class BinaryOFStream {
public:
  BinaryOFStream(std::string fname)
    : _fname(std::move(fname))
    , _out(fopen(_fname.c_str(), "wb"))
  {
    if (!_out) {
      throw std::runtime_error("Failed to open " + _fname);
    }
  }

  template <typename T>
  void write(T src) { write(&src, 1); }

  template <typename T>
  void write(T *src, std::size_t count) {
    static_assert(std::is_trivially_copyable<T>::value);
    auto num_written = fwrite((void*)src, sizeof(T), count, _out);
    if (num_written != count) {
      std::ostringstream msg;
      msg << "Failed to write "
          << count << " object(s) of size " << sizeof(T)
          << " to '" << _fname << "'";
      throw std::runtime_error(msg.str());
    }
  }

  template <typename T>
  BinaryOFStream& operator<<(const T &v) { binary_write(*this, v); return *this; }
  
  template <typename T>
  BinaryOFStream& operator&(const T &v) { return (*this) << v; }

private:
  std::string _fname;
  std::FILE* _out;
}; // class BinaryOFStream

/** parser for my custom binary file type .rmp for the roadmap */
template <typename IStream = BinaryIFStream>
class LazyRmpParser : public RoadmapParser {
public:

  LazyRmpParser(IStream &in)
    : _in(in)
    , _n_verts(0)
    , _n_edges(0)
    , _has_voxels(false)
    , _reference()
    , _voxel_buf(sizeof(uint32_t), '\0') // enough for a 32-bit uint, at least
    , _has_local_voxels(false)
  {
    _in & _n_verts & _n_edges
        & _has_voxels;
    // create a reference voxel object
    // these parameters are stored only once for the whole roadmap
    if (_has_voxels) {
      uint8_t Nb; // # blocks in each direction for whole space
      std::array<double, 6> lims;
      _in & Nb & lims;
      _reference = collision::VoxelOctree(Nb*4);
      _reference.set_xlim(lims[0], lims[1]);
      _reference.set_ylim(lims[2], lims[3]);
      _reference.set_zlim(lims[4], lims[5]);
    }
  }

  ParsedType next() override {
    // exit criteria
    if (_n_verts == 0 && _n_edges == 0) {
      _current_type = ParsedType::DONE;
      return _current_type;
    }

    if (_n_verts > 0) {
      _n_verts--;
      _current_type = ParsedType::VERTEX;
      _in & _current_vertex.index
          & _current_vertex.state
          & _current_vertex.tip_pos;
      _current_vertex.voxels.reset();
      this->try_read_voxels();
      return _current_type;
    }

    if (_n_edges > 0) {
      _n_edges--;
      _current_type = ParsedType::EDGE;
      _in & _current_edge.source
          & _current_edge.target
          & _current_edge.weight;
      _current_edge.voxels.reset();
      this->try_read_voxels();
      return _current_type;
    }

    throw std::runtime_error("unsupported object type detected");
  }

  virtual bool handles_voxels_lazily() const override { return true; }
  virtual void populate_voxels() override {
    if (!_has_local_voxels) { return; }
    auto voxels =
        std::make_shared<collision::VoxelOctree>(_reference.empty_copy());
    BinaryISStream in_stream(_voxel_buf.c_str(), _voxel_buf.size());
    serialize_inner(in_stream, *voxels);
    switch(_current_type) {
      case ParsedType::VERTEX: _current_vertex.voxels = voxels; break;
      case ParsedType::EDGE:   _current_edge.voxels   = voxels; break;
      default:
        throw std::runtime_error("unsupported ParsedType");
    }
  }

private:

  // see if there's a voxel object next, and if so, capture and store the chars
  void try_read_voxels() {
    constexpr size_t n_bytes_per_block = 3*sizeof(uint8_t) + sizeof(uint64_t);

    // if we know there are no voxels, don't even bother reading a boolean each time.
    if (!this->_has_voxels) { return; }

    _in & _has_local_voxels;
    if (_has_local_voxels) {
      uint32_t N_blocks;
      _in & N_blocks;
      const size_t n_block_bytes = N_blocks * n_bytes_per_block;
      _voxel_buf.resize(sizeof(uint32_t) + n_block_bytes);
      memcpy(_voxel_buf.data(), &N_blocks, sizeof(uint32_t));
      _in.read(_voxel_buf.data() + sizeof(uint32_t), n_block_bytes);
    }
  }

private:
  IStream &_in;

  uint32_t _n_verts;
  uint32_t _n_edges;
  bool _has_voxels;
  collision::VoxelOctree _reference;
  std::string _voxel_buf;
  bool _has_local_voxels;
}; // end of class LazyRmpParser

/** Serialize directly to a binary file of my own design (not transferrable)
 *
 * This implementation only stores one copy of the voxel dimensions and limits,
 * and must be done at the beginning of the file.  If a reference is not given,
 * then it is an error to give a non-empty voxel object.
 *
 * Example:
 *   BinaryOFStream fout("roadmap.rmp");
 *   RmpStreamer out(fout, boost::num_vertices(g_), boost::num_edges(g_));
 *   out.write_reference_voxels(voxels);
 *   for (auto v : boost::vertices(g_)) {
 *     out.write_vertex(idx[v], state[v], tip_pos[v], voxels[v]);
 *   }
 *   for (auto e : boost::edges(g_)) {
 *     out.write_edge(source[e], target[e], weight[e], voxels[e]);
 *   }
 */
template <typename OStream = BinaryOFStream>
class RmpStreamer : public RoadmapWriter {
public:
  RmpStreamer(OStream &out, uint32_t n_verts, uint32_t n_edges)
    : _out(out)
    , _n_verts(n_verts)
    , _n_edges(n_edges)
    , _started_writing(false)
    , _has_voxels(false)
  {
    _out & _n_verts & _n_edges;
    OMPL_DEBUG("RmpStreamer: set up to write %lu vertices and %lu edges",
               _n_verts, _n_edges);
  }

  ~RmpStreamer() {
    if (!_started_writing) {
      OMPL_INFORM("RpmStreamer: no reference voxels given for output roadmap");
    }

    // just in case of an empty graph, we still output a boolean indicating no
    // reference voxels.
    this->start_writing();

    if (_n_verts > 0) {
      OMPL_WARN("RmpStreamer: did not finish writing all vertices, %u left",
                _n_verts);
    }
    if (_n_edges > 0) {
      OMPL_WARN("RmpStreamer: did not finish writing all edges, %u left",
                _n_edges);
    }
  }

  void write_reference_voxels(const collision::VoxelOctree &ref) {
    if (_started_writing) {
      throw ompl::Exception("RmpStreamer",
          "Writing reference voxels after a vertex or edge is not allowed");
    }
    _has_voxels = true;
    this->start_writing();

    std::array<double, 6> lims;
    auto lower_left = ref.lower_left();
    auto upper_right = ref.upper_right();
    lims[0] = lower_left[0];
    lims[1] = upper_right[0];
    lims[2] = lower_left[1];
    lims[3] = upper_right[1];
    lims[4] = lower_left[2];
    lims[5] = upper_right[2];

    uint8_t Nb = ref.Nbx();

    _out & Nb & lims;
  }

  virtual void write_vertex(
      uint32_t index,
      const std::vector<double> state,
      motion_planning::VoxelCachedLazyPRM::TipPosition tip_pos,
      motion_planning::VoxelCachedLazyPRM::VoxelPtr voxels)
    override
  {
    if (_n_verts == 0) {
      throw ompl::Exception("RmpStreamer",
          "Writing more vertices than declared when constructed");
    }
    start_writing();
    _n_verts--;
    _out & index & state & tip_pos;
    try_write_voxels(voxels);
  }

  virtual void write_edge(
      uint32_t source,
      uint32_t target,
      double weight,
      motion_planning::VoxelCachedLazyPRM::VoxelPtr voxels)
    override
  {
    if (_n_verts > 0) {
      throw ompl::Exception("RmpStreamer",
          "Writing an edge before done with writing all vertices");
    }
    if (_n_edges == 0) {
      throw ompl::Exception("RmpStreamer",
          "Writing more edges than declared when constructed");
    }
    start_writing();
    _n_edges--;
    _out & source & target & weight;
    try_write_voxels(voxels);
  }

private:
  void try_write_voxels(motion_planning::VoxelCachedLazyPRM::VoxelPtr voxels) {
    if (voxels && !_has_voxels) {
      throw ompl::Exception("RmpStreamer",
          "Writing a voxel object without giving a reference one first");
    }
    if (_has_voxels) {
      _out & bool(voxels);
      // if there are voxels, then output the occupied blocks
      if (voxels) {
        _out & static_cast<uint32_t>(voxels->nblocks()); // number of leaf blocks
        voxels->visit_leaves(
            [this](uint8_t bx, uint8_t by, uint8_t bz, uint64_t val) {
              _out & bx & by & bz & val;
            });
      }
    }
  }

  void start_writing() {
    if (!_started_writing) {
      _started_writing = true;
      _out & _has_voxels;
    }
  }

private:
  OStream &_out;

  uint32_t _n_verts;
  uint32_t _n_edges;
  bool _started_writing;
  bool _has_voxels;
}; // end of class RmpStreamer

} // end of unnamed namespace in namespace motion_planning

// ================
//  Public Methods
// ================

VoxelCachedLazyPRM::VoxelCachedLazyPRM(const ob::SpaceInformationPtr &si,
                                       bool starStrategy)
  : ob::Planner(si, "VoxelCachedLazyPRM")
  , starStrategy_(starStrategy)
  , indexProperty_(boost::get(boost::vertex_index_t(), g_))
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , vertexVoxelsProperty_(boost::get(vertex_voxel_cache_t(), g_))
  , tipPositionProperty_(boost::get(vertex_tip_position_t(), g_))
  , weightProperty_(boost::get(boost::edge_weight, g_))
  , vertexComponentProperty_(boost::get(vertex_component_t(), g_))
  , vertexValidityProperty_(boost::get(vertex_flags_t(), g_))
  , edgeVoxelsProperty_(boost::get(edge_voxel_cache_t(), g_))
  , edgeValidityProperty_(boost::get(edge_flags_t(), g_))
  , timers_()
  , ikController_()
{
  timers_.emplace("astar", util::FunctionTimer());
  timers_.emplace("reindex", util::FunctionTimer());
  timers_.emplace("remove_vertices", util::FunctionTimer());
  timers_.emplace("ik_controller", util::FunctionTimer());
  // TODO: do we want more timers for IK or something?

  specs_.recognizedGoal = ob::GOAL_SAMPLEABLE_REGION;
  specs_.approximateSolutions = false;
  specs_.optimizingPaths = true;

  Planner::declareParam<double>(
      "range",
      this, &VoxelCachedLazyPRM::setRange, &VoxelCachedLazyPRM::getRange,
      "0.:1.:10000.");
  if (!starStrategy_) {
    Planner::declareParam<size_type>(
        "max_nearest_neighbors",
        this, &VoxelCachedLazyPRM::setMaxNearestNeighbors,
        std::string("8:1000"));
  }

  addPlannerProgressProperty("iterations INTEGER",
      [this] { return getIterationCount(); });
  addPlannerProgressProperty("best cost REAL",
      [this] { return getBestCost(); });
  addPlannerProgressProperty("milestone count INTEGER",
      [this] { return getMilestoneCountString(); });
  addPlannerProgressProperty("edge count INTEGER",
      [this] { return getEdgeCountString(); });
}

VoxelCachedLazyPRM::VoxelCachedLazyPRM(const ob::PlannerData &data,
                                       bool starStrategy)
  : VoxelCachedLazyPRM(data.getSpaceInformation(), starStrategy)
{
  if (data.numVertices() > 0) {
    // mapping between vertex id from PlannerData and Vertex in Boost.Graph
    std::map<size_type, Vertex> vertices;
    // helper function to create vertices as needed and update the vertices mapping
    const auto &getOrCreateVertex = [&](size_type vertex_index) {
      if (!vertices.count(vertex_index)) {
        const auto &data_vertex = data.getVertex(vertex_index);
        Vertex graph_vertex = boost::add_vertex(g_);
        stateProperty_[graph_vertex] = si_->cloneState(data_vertex.getState());
        vertexValidityProperty_[graph_vertex] = VALIDITY_UNKNOWN;
        auto newComponent = componentCount_++;
        vertexComponentProperty_[graph_vertex] = newComponent;
        vertices[vertex_index] = graph_vertex;
      }
      return vertices.at(vertex_index);
    };

    // temporarily set to false since nn_ is used only in single thread
    specs_.multithreaded = false;
    nn_.reset(ot::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nnTip_.reset(ot::SelfConfig::getDefaultNearestNeighbors<
        VertexAndTip>(this));
    specs_.multithreaded = true;
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b) {
          return distanceFunction(a, b);
        });
    nnTip_->setDistanceFunction(
        [this](const VertexAndTip &a, const VertexAndTip &b) {
          return tipDistanceFunction(a, b);
        });

    for (size_type vertex_index = 0; vertex_index < data.numVertices(); ++vertex_index) {
      Vertex m = getOrCreateVertex(vertex_index);
      std::vector<size_type> neighbor_indices;
      data.getEdges(vertex_index, neighbor_indices);
      for (const auto neighbor_index : neighbor_indices) {
        Vertex n = getOrCreateVertex(neighbor_index);
        ob::Cost weight;
        data.getEdgeWeight(vertex_index, neighbor_index, &weight);
        connectVertices(m, n, weight);
      }
      nn_->add(m);
    }
  }
}

VoxelCachedLazyPRM::~VoxelCachedLazyPRM() {
  this->freeMemory();
}

const tendon::TendonRobot& VoxelCachedLazyPRM::robot() {
  if (!voxelStateChecker_) {
    voxelStateChecker_ = std::dynamic_pointer_cast<AbstractVoxelValidityChecker>(
        si_->getStateValidityChecker());
    if (!voxelStateChecker_) {
      throw ompl::Exception(name_,
          "Cannot retrieve robot from state validity checker, not set to an"
          " AbstractVoxelValidityChecker object.");
    }
  }
  return voxelStateChecker_->robot();
}

void VoxelCachedLazyPRM::setup() {
  // base setup() function:
  // - makes sure si_->isSetup()
  // - sets _setup=true;
  Planner::setup();
  ot::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!nn_) {
    nn_.reset(ot::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction(
        [this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
  }
  if (!nnTip_) {
    nnTip_.reset(
        ot::SelfConfig::getDefaultNearestNeighbors<VertexAndTip>(this));
    nnTip_->setDistanceFunction(
        [this](const VertexAndTip &a, const VertexAndTip &b) {
          return tipDistanceFunction(a, b);
        });
  }
  if (!connectionStrategy_) {
    setDefaultConnectionStrategy();
  }
  if (!connectionFilter_) {
    connectionFilter_ = [](const Vertex &, const Vertex &) {
      return true;
    };
  }

  // Setup optimization objective
  //
  // If no optimization objective was specified, then default to
  // optimizing path length as computed by the distance() function
  // in the state space.
  if (pdef_) {
    if (pdef_->hasOptimizationObjective()) {
      opt_ = pdef_->getOptimizationObjective();
    } else {
      opt_ = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
      if (!starStrategy_) {
        opt_->setCostThreshold(opt_->infiniteCost());
      }
    }
  } else {
    OMPL_WARN("%s: problem definition is not set, deferring setup completion...",
              getName().c_str());
    setup_ = false;
  }

  voxelStateChecker_ = std::dynamic_pointer_cast<AbstractVoxelValidityChecker>(
      si_->getStateValidityChecker());
  if (!voxelStateChecker_) {
    OMPL_WARN("%s: state validity checker is not set to an "
              "AbstractVoxelValidityChecker, deferring setup completion...",
              getName().c_str());
    setup_ = false;
  }

  voxelMotionValidator_ = std::dynamic_pointer_cast<AbstractVoxelMotionValidator>(
      si_->getMotionValidator());
  if (!voxelMotionValidator_) {
    OMPL_WARN("%s: motion validator is not set to an "
              "AbstractVoxelMotionValidator, deferring setup completion...",
              getName().c_str());
    setup_ = false;
  }

  sampler_ = si_->allocStateSampler();

  if (!ikController_) {
    setDefaultIkController();
    if (!ikController_) {
      OMPL_WARN("%s: could not set default IK controller,"
                " deferring setup completion...",
                getName().c_str());
      setup_ = false;
    }
  }

  //if (setup_) { precomputeVoxelCache(); }
}

void VoxelCachedLazyPRM::setRange(double distance) {
  maxDistance_ = distance;
  if (!userSetConnectionStrategy_) {
    setDefaultConnectionStrategy();
  }
  if (isSetup()) {
    setup();
  }
}

void VoxelCachedLazyPRM::setMaxNearestNeighbors(size_type k) {
  if (starStrategy_) {
    throw ompl::Exception("Cannot set the maximum nearest neighbors for " + getName());
  }
  if (!nn_) {
    nn_.reset(ot::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction(
        [this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
  }
  if (!userSetConnectionStrategy_) {
    connectionStrategy_ = og::KBoundedStrategy<Vertex>(k, maxDistance_, nn_);
  }
  if (isSetup()) {
    setup();
  }
}

void VoxelCachedLazyPRM::setStarConnectionStrategy() {
  if (!nn_) {
    nn_.reset(ot::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction(
        [this](const Vertex a, const Vertex b) { return distanceFunction(a, b); });
  }
  connectionStrategy_ = og::KStarStrategy<Vertex>(
      [this]() { return milestoneCount(); }, nn_, si_->getStateDimension());
  starStrategy_ = true;
}

void VoxelCachedLazyPRM::setDefaultConnectionStrategy() {
  userSetConnectionStrategy_ = false;
  if (starStrategy_) {
    setStarConnectionStrategy();
  } else {
    setMaxNearestNeighbors(magic::DEFAULT_NEAREST_NEIGHBORS_LAZY);
  }
}

void VoxelCachedLazyPRM::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) {
  Planner::setProblemDefinition(pdef);
  clearQuery();
}

void VoxelCachedLazyPRM::clearQuery() {
  startM_.clear();
  goalM_.clear();
  pis_.restart();
  if (pdef_) {
    pdef_->clearSolutionPaths(); // forget previously returned solutions
  }
}

void VoxelCachedLazyPRM::createRoadmap(
    size_type N, VoxelCachedLazyPRM::CreateRoadmapOption opt)
{
  // TODO: add method addToRoadmap() to add N vertices, then call that from this
  OMPL_DEBUG("%s: createRoadmap(%u, %u)",
             getName().c_str(), N, opt);

  auto Nv = boost::num_vertices(g_);
  if (N <= Nv) {
    OMPL_WARN("%s: Graph is already at or bigger than %u, skipping roadmap creation",
              getName().c_str(), N);
    return;
  }

  OMPL_INFORM("%s: Creating roadmap from size %u to %u",
              getName().c_str(), boost::num_vertices(g_), N);

  renumberIndices();

  checkValidity();
  Nv = boost::num_vertices(g_);

  struct VertexToAdd {
    ob::State   *state;
    TipPosition  tip_pos;
    VoxelPtr     voxels;
  };

  const bool validate_verts = opt & ValidateVertices;
  const bool validate_edges = opt & ValidateEdges;
  const bool voxelize_verts = validate_verts || (opt & VoxelizeVertices);
  const bool voxelize_edges = validate_edges || (opt & VoxelizeEdges);

  auto space = si_->getStateSpace();

  const auto resample_vertex =
    [this, voxelize_verts, validate_verts, space]
    (VertexToAdd &v, std::vector<double> &state_buffer)
  {
    this->sampler_->sampleUniform(v.state);
    if (!voxelize_verts) {
      return true;    // accept sample
    }
    space->copyToReals(state_buffer, v.state);
    auto fk_result = this->voxelStateChecker_->fk(state_buffer);
    auto &[fk_shape, home_shape] = fk_result;
    bool is_valid_shape = this->voxelStateChecker_->is_valid_shape(fk_shape, home_shape);
    if (!is_valid_shape) {
      return false;   // reject sample
    }

    v.voxels = std::make_shared<collision::VoxelOctree>(
        this->voxelStateChecker_->voxelize(fk_shape));
    if (validate_verts && this->voxelStateChecker_->collides(*v.voxels)) {
      return false;   // reject sample
    }

    v.tip_pos = fk_shape.p.back();
    return true;      // accept sample
  };

  const auto resample_valid_vertex = [resample_vertex](VertexToAdd &v) {
    std::vector<double> robot_state_buffer;
    while (!resample_vertex(v, robot_state_buffer)) {} // rejection sampling
  };

  // rejection sampling in parallel
  std::vector<VertexToAdd> new_vertices(N - Nv);
  #pragma omp parallel for \
      shared(new_vertices, space) \
      schedule(dynamic, 1)
  for (size_type idx = Nv; idx < N; ++idx) {
    auto &v = new_vertices[idx-Nv]; // now just populate v
    v.state = si_->allocState();
    resample_valid_vertex(v);
  } // end of omp parallel for

  OMPL_DEBUG("%s: Finished creating %u vertices, now to add them to the roadmap",
             getName().c_str(), new_vertices.size());

  // add the vertices to the roadmap
  // we do this first so that the total number of vertices influence the number
  // of connections to make if using the PRM* strategy
  std::vector<Vertex> added_vertices(new_vertices.size());
  for (size_type idx = Nv; idx < N; ++idx) {
    auto &vert = new_vertices[idx-Nv];
    Vertex &v = added_vertices[idx-Nv];
    bool was_added = false;
    v = addMilestone(vert.state, false, &was_added);
    while (!was_added) {
      OMPL_DEBUG("%s: Resampling vertex and trying to add again (idx: %d)",
          getName().c_str(), idx);
      vert.state = si_->allocState(); // addMilestone deletes it if it's duplicate
      resample_valid_vertex(vert);
      v = addMilestone(vert.state, false, &was_added);
    }
    indexProperty_[v] = idx;
    vertexVoxelsProperty_[v] = vert.voxels;
    vertexValidityProperty_[v] = validate_verts ? VALIDITY_TRUE : VALIDITY_UNKNOWN;
    tipPositionProperty_[v] = vert.tip_pos;
    if (vert.tip_pos.has_value()) {
      nnTip_->add(VertexAndTip{v, *vert.tip_pos});
    }
  }
  new_vertices = {}; // free memory

  OMPL_DEBUG("%s: Finished adding %u vertices for a total of %u vertices",
             getName().c_str(), added_vertices.size(), boost::num_vertices(g_));

  // generate all empty edges sequentially, since it does graph operations
  // also since connectionStrategy_() may or may not be reentrant
  std::vector<Edge> added_edges;
  for (auto v : added_vertices) {
    const auto &neighbors = connectionStrategy_(v);
    for (auto n : neighbors) {
      if (!getEdge(v, n)) { // edge can exist from previous iterations
        auto e = connectVertices(v, n);
        if (e.has_value()) {
          added_edges.emplace_back(*e);
        }
      }
    }
  }
  added_vertices = {}; // free memory

  OMPL_DEBUG("%s: Finished adding %u empty edges for a total of %u edges",
             getName().c_str(), added_edges.size(), boost::num_edges(g_));

  if (voxelize_edges) {
    if (validate_edges) {
      OMPL_DEBUG("%s: Now to voxelize and validate %u edges in parallel",
                 getName().c_str(), added_edges.size());
    } else {
      OMPL_DEBUG("%s: Now to voxelize %u edges in parallel",
                 getName().c_str(), added_edges.size());
    }

    std::vector<Edge> edges_to_remove;
    const auto ereport_n = std::max(size_t(1000), added_edges.size() / 100);
    size_type n_done = 0;
    #pragma omp parallel for shared(edges_to_remove, n_done)
    for (size_type i = 0; i < added_edges.size(); ++i) {
      Edge e = added_edges[i];
      bool is_valid;
      if (validate_edges) {
        is_valid = computeEdgeValidity(e); // the slow part
      } else {
        is_valid = voxelizeEdge(e); // the slow part
      }

      #pragma omp critical // contains just fast stuff
      {
        if (!is_valid) { edges_to_remove.emplace_back(e); }

        // print progress
        n_done++;
        if (n_done % ereport_n == 0) {
          OMPL_DEBUG("%s: Finished voxelizing %d/%d edges (%d%%)",
                     getName().c_str(), n_done, added_edges.size(),
                     int(std::round(double(100 * n_done) / added_edges.size())));
        }
      }
    }

    OMPL_DEBUG("%s: Found %u/%u invalid edges (of %u total), removing now",
               getName().c_str(), edges_to_remove.size(), added_edges.size(),
               boost::num_edges(g_));

    for (auto e : edges_to_remove) {
      removeEdge(e);
    }

    OMPL_DEBUG("%s: Removed %u invalid edges, now there are %u edges",
               getName().c_str(), edges_to_remove.size(), boost::num_edges(g_));
  }
  added_edges = {}; // free memory

  OMPL_DEBUG("%s: max distance for connection strategy is %lf",
             getName().c_str(), maxDistance_);
  OMPL_INFORM("%s: after createRoadmap, Graph has %u vertices and %u edges",
              getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));
}

void VoxelCachedLazyPRM::precomputeVertexValidity() {
  checkValidity();

  if (!setup_) {
    OMPL_ERROR("%s: Could not complete setup before precomputeValidity()",
               getName().c_str());
    return;
  }

  //
  // Vertices
  //
  OMPL_INFORM("%s: Precomputing validity on %d states",
              getName().c_str(), boost::num_vertices(g_));
  std::set<Vertex> vertices_to_remove;

  // copy over the vertex indices, then loop
  // within a scope to release this vector
  {
    auto [vbegin, vend] = boost::vertices(g_);
    std::vector<Vertex> all_verts(vbegin, vend);
    #pragma omp parallel for schedule(dynamic, 100)
    for (size_type i = 0; i < all_verts.size(); ++i) {
      Vertex v = all_verts[i];
      if (!computeVertexValidity(v)) {
        #pragma omp critical
        vertices_to_remove.insert(v);
      }
    }
  }
  const auto num_edges_before = boost::num_edges(g_);
  removeVertices(vertices_to_remove);
  OMPL_INFORM("%s: Removed %u invalid states (and %u connecting edges)",
              getName().c_str(), vertices_to_remove.size(),
              num_edges_before - boost::num_edges(g_));
}

void VoxelCachedLazyPRM::precomputeEdgeValidity() {
  checkValidity();

  if (!setup_) {
    OMPL_ERROR("%s: Could not complete setup before precomputeValidity()",
               getName().c_str());
    return;
  }

  //
  // Edges
  //
  OMPL_INFORM("%s: Precomputing validity on %d edges",
              getName().c_str(), boost::num_edges(g_));
  std::set<Edge> edges_to_remove;
  // There's generally lots more edges than vertices.  I don't want to copy
  // over all edge indices.  Plus the edge work is much more intensive, making
  // this OpenMP approach more viable.
  const auto n_edges = boost::num_edges(g_);
  size_type n_done = 0;
  const auto ereport_n = std::max(size_t(1000), n_edges / 100);
  #pragma omp parallel shared(edges_to_remove, n_done)
  #pragma omp single
  for (Edge e : boost::edges(g_)) {
    #pragma omp task firstprivate(e)
    {
      bool is_valid = computeEdgeValidity(e); // the slow part

      #pragma omp critical
      {
        if (!is_valid) { edges_to_remove.insert(e); }

        // print progress
        n_done++;
        if (n_done % ereport_n == 0) {
          OMPL_DEBUG("%s: Finished voxelizing %d/%d edges (%d%%)",
                     getName().c_str(), n_done, n_edges,
                     int(std::round(double(100 * n_done) / n_edges)));
        }
      }
    }
  }
  for (auto e : edges_to_remove) {
    removeEdge(e);
  }
  OMPL_INFORM("%s: Removed %u invalid edges", getName().c_str(),
              edges_to_remove.size());
}

void VoxelCachedLazyPRM::precomputeValidity() {
  precomputeVertexValidity();
  precomputeEdgeValidity();
  OMPL_INFORM("%s: Done precomputing validity, now have %u vertices and %u edges",
              getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));
}

void VoxelCachedLazyPRM::clearValidity() {
  for (const Vertex v : boost::vertices(g_)) {
    vertexValidityProperty_[v] = VALIDITY_UNKNOWN;
  }
  for (const Edge e : boost::edges(g_)) {
    edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
  }
}

void VoxelCachedLazyPRM::clearDisconnectedVertices() {
  size_type largest_component = 0;
  for (size_type i = 0; i < componentCount_; ++i) {
    if (componentSize_[i] > componentSize_[largest_component]) {
      largest_component = i;
    }
  }

  std::set<Vertex> vertices_to_remove;
  for (Vertex v : boost::vertices(g_)) {
    if (vertexComponentProperty_[v] != largest_component) {
      vertices_to_remove.insert(v);
    }
  }

  const auto num_edges_before = boost::num_edges(g_);
  removeVertices(vertices_to_remove);
  OMPL_INFORM("%s: Removed %u disconnected states (and %u connecting edges)",
              getName().c_str(), vertices_to_remove.size(),
              num_edges_before - boost::num_edges(g_));
}

void VoxelCachedLazyPRM::precomputeVertexVoxelCache() {
  checkValidity();

  int num_to_precompute = 0;
  for (const Vertex v : boost::vertices(g_)) {
    if (!vertexVoxelsProperty_[v] || !tipPositionProperty_[v]) {
      ++num_to_precompute;
    }
  }
  OMPL_INFORM("%s: Precomputing voxel cache on %d/%d states",
              getName().c_str(), num_to_precompute, boost::num_vertices(g_));
  std::set<Vertex> vertices_to_remove;

  // copy over the vertex indices, then loop
  {
    auto [vbegin, vend] = boost::vertices(g_);
    std::vector<Vertex> all_verts(vbegin, vend);
    #pragma omp parallel for schedule(dynamic, 100)
    for (size_t i = 0; i < all_verts.size(); ++i) {
      Vertex v = all_verts[i];
      bool is_valid_shape = voxelizeVertex(v);
      if (!is_valid_shape) {
        #pragma omp critical
        vertices_to_remove.insert(v);
      }
    }
  }

  {
    auto before_num_verts = boost::num_vertices(g_);
    auto before_num_edges = boost::num_edges(g_);
    OMPL_DEBUG("%s: Removing %u invalid states (of %u total states)",
               getName().c_str(), vertices_to_remove.size(), before_num_verts);

    removeVertices(vertices_to_remove);

    auto after_num_verts = boost::num_vertices(g_);
    auto after_num_edges = boost::num_edges(g_);
    OMPL_DEBUG("%s: Removed %u/%u states and %u/%u edges (now %u states, %u edges)",
               getName().c_str(),
               vertices_to_remove.size(), before_num_verts,
               before_num_edges - after_num_edges, before_num_edges,
               after_num_verts, after_num_edges);
  }

  OMPL_INFORM("%s: Done precomputing vertex voxel cache", getName().c_str());
}

void VoxelCachedLazyPRM::precomputeEdgeVoxelCache() {
  checkValidity();

  size_type num_to_precompute = 0;
  for (const Edge e : boost::edges(g_)) {
    if (!edgeVoxelsProperty_[e]) {
      ++num_to_precompute;
    }
  }
  auto num_edges = boost::num_edges(g_);
  OMPL_INFORM("%s: Precomputing voxel cache on %d/%d edges",
              getName().c_str(), num_to_precompute, num_edges);
  std::set<Edge> edges_to_remove;
  const auto ereport_n = std::max(decltype(num_edges)(1000), num_edges / 100);
  decltype(num_edges) n_edges_voxelized = 0;
  // need to do openMP this way because the iterators are not random-access
  #pragma omp parallel
  {
    #pragma omp single
    for (Edge e : boost::edges(g_)) {
      #pragma omp task default(none) firstprivate(e) \
                       shared(edges_to_remove, \
                              n_edges_voxelized, \
                              num_edges)
      {
        bool is_valid_shape = voxelizeEdge(e); // this is the slow part
        #pragma omp critical // this should just be fast stuff
        {
          if (!is_valid_shape) {
            edges_to_remove.insert(e);
          }
          n_edges_voxelized++;
          if (n_edges_voxelized % ereport_n == 0) {
            OMPL_DEBUG("%s: Finished voxelizing %d/%d edges (%d%%)",
                       getName().c_str(), n_edges_voxelized, num_edges,
                       int(std::round(double(100 * n_edges_voxelized) / num_edges)));
          }
        }
      }
    }
  }

  OMPL_DEBUG("%s: removing %u edges", getName().c_str(), edges_to_remove.size());
  for (Edge e : edges_to_remove) {
    removeEdge(e);
  }
  OMPL_INFORM("%s: Done precomputing edge voxel cache", getName().c_str());
}

void VoxelCachedLazyPRM::precomputeVoxelCache() {
  precomputeVertexVoxelCache();

  // // create a temporary place to store progress
  // {
  //   const std::string progress_file = "/tmp/roadmap-voxels-progress.dat";
  //   std::ofstream fout;
  //   util::openfile_check(fout, progress_file);
  //   DatStreamer writer(fout);
  //   OMPL_INFORM("%s: saving intermediate voxelization results to %s",
  //               getName().c_str(), progress_file.c_str());

  //   // write all of the vertices
  //   auto space = si_->getStateSpace();
  //   for (const Vertex v: boost::vertices(g_)) {
  //     std::vector<double> robot_state;
  //     space->copyToReals(robot_state, stateProperty_[v]);
  //     writer.write_vertex(
  //         indexProperty_[v],
  //         robot_state,
  //         tipPositionProperty_[v],
  //         vertexVoxelsProperty_[v]);
  //   }
  // }

  precomputeEdgeVoxelCache();

  OMPL_INFORM("%s: Done precomputing voxel cache", getName().c_str());
}

void VoxelCachedLazyPRM::clearVertexVoxelCache() {
  for (const Vertex v : boost::vertices(g_)) {
    vertexVoxelsProperty_[v].reset(); // delete
  }
}

void VoxelCachedLazyPRM::clearEdgeVoxelCache() {
  for (const Edge e : boost::edges(g_)) {
    edgeVoxelsProperty_[e].reset();   // delete
  }
}

void VoxelCachedLazyPRM::clearVoxelCache() {
  clearVertexVoxelCache();
  clearEdgeVoxelCache();
}

void VoxelCachedLazyPRM::clear() {
  Planner::clear();
  freeMemory();
  if (nn_) {
    nn_->clear();
  }
  if (nnTip_) {
    nnTip_->clear();
  }
  clearQuery();

  componentCount_ = 0;
  iterations_ = 0;
  bestCost_ = ob::Cost(std::numeric_limits<double>::quiet_NaN());
}

void VoxelCachedLazyPRM::freeMemory() {
  for (const Vertex v : boost::vertices(g_)) {
    si_->freeState(stateProperty_[v]);
  }
  g_.clear();
}

VoxelCachedLazyPRM::Vertex VoxelCachedLazyPRM::addMilestone(
    ob::State *state, bool connect, bool *ok)
{
  auto [m, was_added] = this->tryAddToGraph(state);
  if (ok) { *ok = was_added; }
  if (!was_added) { // if it was already there, then do nothing
    if (state != stateProperty_[m]) { // if the pointers don't match
      OMPL_DEBUG("%s::addMilestone: deleting state 0x%x (!= 0x%x)",
                 getName().c_str(), state, stateProperty_[m]);
      si_->freeState(state);
    }
    return m;
  }

  // finish adding the milestone
  vertexValidityProperty_[m] = VALIDITY_UNKNOWN;
  auto newComponent = componentCount_++;
  vertexComponentProperty_[m] = newComponent;
  componentSize_[newComponent] = 1;

  if (connect) {
    // Which milestones will we attempt to connect to?
    const std::vector<Vertex> &neighbors = connectionStrategy_(m);
    for (Vertex n : neighbors) {
      connectVertices(m, n);
    }
  }

  nn_->add(m);

  return m;
}

ob::PlannerStatus VoxelCachedLazyPRM::solve(
    const ob::PlannerTerminationCondition &ptc)
{
  auto status = solvePrep(ptc);
  if (status != ob::PlannerStatus::APPROXIMATE_SOLUTION) {
    return status;
  }

  auto nrStartStates = boost::num_vertices(g_);
  OMPL_INFORM("%s::solve() Starting planning with %lu states already in"
              " datastructure", getName().c_str(), nrStartStates);

  bestCost_ = opt_->infiniteCost();
  ob::State *workState = si_->allocState();
  std::pair<size_type, size_type> startGoalPair;
  ob::PathPtr bestSolution;
  bool fullyOptimized = false;
  bool someSolutionFound = false;
  size_type optimizingComponentSegments = 0;

  // Grow roadmap in lazy fashion -- add vertices and edges without checking
  // validity
  while (!ptc) {
    ++iterations_;
    sampler_->sampleUniform(workState);
    Vertex addedVertex = addMilestone(si_->cloneState(workState));

    const long int solComponent = solutionComponent(&startGoalPair);
    // If the start & goal are connected and we either did not find any solution
    // so far or the one we found still needs optimizing and we just added an edge
    // to the connected component that is used for the solution, we attempt to
    // construct a new solution.
    if (solComponent != -1 &&
        (!someSolutionFound ||
         (long int)vertexComponentProperty_[addedVertex] == solComponent))
    {
      // If we already have a solution, we are optimizing. We check that we
      // added at least a few segments to the connected component that
      // includes the previously found solution before attempting to
      // construct a new solution.
      if (someSolutionFound) {
        if (++optimizingComponentSegments
              < magic::MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION)
        {
          continue;
        }
        optimizingComponentSegments = 0;
      }
      Vertex startV = startM_[startGoalPair.first];
      Vertex goalV = goalM_[startGoalPair.second];
      ob::PathPtr solution;
      do {
        solution = constructSolution(startV, goalV);
      } while (!solution &&
               vertexComponentProperty_[startV]
                 == vertexComponentProperty_[goalV]);
      if (solution) {
        someSolutionFound = true;
        ob::Cost c = solution->cost(opt_);
        if (opt_->isSatisfied(c)) {
          fullyOptimized = true;
          bestSolution = solution;
          bestCost_ = c;
          break;
        }
        if (opt_->isCostBetterThan(c, bestCost_)) {
          bestSolution = solution;
          bestCost_ = c;
        }
      }
    }
  }

  si_->freeState(workState);

  if (bestSolution) {
    ob::PlannerSolution psol(bestSolution);
    psol.setPlannerName(getName());
    // if the solution was optimized, we mark it as such
    psol.setOptimized(opt_, bestCost_, fullyOptimized);
    pdef_->addSolutionPath(psol);
  }

  OMPL_INFORM("%s: Created %d states",
              getName().c_str(), boost::num_vertices(g_) - nrStartStates);

  return bestSolution ? ob::PlannerStatus::EXACT_SOLUTION :
                        ob::PlannerStatus::TIMEOUT;
}

ob::PlannerStatus VoxelCachedLazyPRM::solveWithRoadmap(
      const ompl::base::PlannerTerminationCondition &ptc)
{
  auto myptc = plannerOrTerminationCondition(ptc,
      ob::exactSolnPlannerTerminationCondition(pdef_));

  auto status = solvePrep(myptc);
  if (status != ob::PlannerStatus::APPROXIMATE_SOLUTION) {
    return status;
  }

  OMPL_INFORM("%s::solveWithRoadmap(): Starting planning with %lu states"
              " already in datastructure",
              getName().c_str(), boost::num_vertices(g_));

  ob::PathPtr solution;
  bestCost_ = opt_->infiniteCost();
  bool fullyOptimized = false;

  auto state2str = [this](auto v) {
    using util::operator<<;
    std::vector<double> state_vals;
    std::ostringstream vec_buffer;
    auto space = this->si_->getStateSpace();
    space->copyToReals(state_vals, this->stateProperty_[v]);
    vec_buffer << state_vals;
    return vec_buffer.str();
  };

  // This method does not do sampling to refine the roadmap, but we allow
  // multiple start, goal pairs to be queried in sequence.
  while (!myptc) {
    ++iterations_;

    // TODO: Solve A* with multiple goals simultaneously rather than just
    // TODO- matching up the first start and goal within the same connected
    // TODO- component.

    // solutionComponent() returns the first start and goal in the same component
    std::pair<size_type, size_type> startGoalPair;
    const long int solComponent = solutionComponent(&startGoalPair);

    auto start_str = state2str(startM_[0]);
    auto goal_str = state2str(goalM_[0]);
    OMPL_DEBUG("%s: start[0]: %s (%d edges, component %d),"
               " goal[0]: %s (%d edges, component %d)",
               getName().c_str(),
               start_str.c_str(), boost::degree(startM_[0], g_),
               vertexComponentProperty_[startM_[0]],
               goal_str.c_str(), boost::degree(goalM_[0], g_),
               vertexComponentProperty_[goalM_[0]]
               );

    // If the start and goal do not belong to the same connected component, then
    // give up and return APPROXIMATE_SOLUTION.
    // TODO: find the closest state and plan to that
    // TODO: grab the closest state in WSpace
    // TODO: if cannot plan to nearest WSpace state, get closest one in CSpace?
    if (solComponent == -1) {
      OMPL_DEBUG("%s: start and goal states belong to different connected"
                 " components.", getName().c_str());
      OMPL_DEBUG("%s: goal state has %d edges", getName().c_str(),
                 boost::out_degree(goalM_[startGoalPair.second], g_));
      OMPL_DEBUG("%s: we have %d start states and %d goal states",
                 getName().c_str(), startM_.size(), goalM_.size());
      return ob::PlannerStatus::TIMEOUT;
    }

    Vertex startV = startM_[startGoalPair.first];
    Vertex goalV = goalM_[startGoalPair.second];

    {
      using util::operator<<;
      std::vector<double> state_vals;
      std::ostringstream vec_buffer;
      auto space = si_->getStateSpace();
      space->copyToReals(state_vals, stateProperty_[startV]);
      vec_buffer << state_vals;
      OMPL_DEBUG("%s: planning: START STATE = %s",
                 getName().c_str(), vec_buffer.str().c_str());
      space->copyToReals(state_vals, stateProperty_[goalV]);
      vec_buffer.str("");
      vec_buffer << state_vals;
      OMPL_DEBUG("%s: planning: GOAL STATE  = %s",
                 getName().c_str(), vec_buffer.str().c_str());
    }

    do {
      solution = constructSolution(startV, goalV);
    } while (!myptc
          && !solution
          &&  vertexComponentProperty_[startV]
                == vertexComponentProperty_[goalV]);

    // TODO: find the closest vertex to the goal within the start state component
    // TODO- and start the search over again to return an approximate solution
    // TODO: add timeout
    // TODO: if timeout, return partial solution

    if (solution) {
      break;
    }
  }

  if (!solution) {
    return ob::PlannerStatus::TIMEOUT;
  }

  ob::Cost c = solution->cost(opt_);
  bestCost_ = c;
  if (opt_->isSatisfied(c)) {
    fullyOptimized = true;
  }
  ob::PlannerSolution psol(solution);
  psol.setPlannerName(getName());
  psol.setOptimized(opt_, bestCost_, fullyOptimized);
  pdef_->addSolutionPath(psol);

  return ob::PlannerStatus::EXACT_SOLUTION;
}

ompl::base::PlannerStatus VoxelCachedLazyPRM::solveWithRoadmap(double timeout) {
  return solveWithRoadmap(ob::timedPlannerTerminationCondition(timeout));
}

nlohmann::json VoxelCachedLazyPRM::toRoadmapJson() const {
  using nlohmann::json;
  auto space = si_->getStateSpace();

  auto vertices = json::array();
  for (const Vertex v : boost::vertices(g_)) {
    auto state = stateProperty_[v];
    std::vector<double> robot_state;
    space->copyToReals(robot_state, state);
    json vertex;
    vertex["index"] = indexProperty_[v];
    vertex["state"] = robot_state;
    auto voxels = vertexVoxelsProperty_[v];
    if (voxels) {
      vertex["voxels"] = voxels->to_json();
    }
    auto tip = tipPositionProperty_[v];
    if (tip.has_value()) {
      vertex["tip_pos"] = {(*tip)[0], (*tip)[1], (*tip)[2]};
    }
    vertices.push_back(vertex);
  }

  auto edges = json::array();
  for (const Edge e : boost::edges(g_)) {
    json edge;
    edge["source"] = indexProperty_[boost::source(e, g_)];
    edge["target"] = indexProperty_[boost::target(e, g_)];
    edge["weight"] = weightProperty_[e].value();
    auto voxels = edgeVoxelsProperty_[e];
    if (voxels) {
      edge["voxels"] = voxels->to_json();
    }
    edges.push_back(edge);
  }

  json container;
  container = {
    {"VoxelCachedLazyPRM_roadmap", {
      {"vertices", vertices},
      {"edges", edges},
    }},
  };
  return container;
}

void VoxelCachedLazyPRM::saveRoadmapJson(const std::string &filename) const {
  OMPL_INFORM("%s: writing roadmap to %s", getName().c_str(), filename.c_str());
  util::write_json(filename, this->toRoadmapJson());
}

void VoxelCachedLazyPRM::fromRoadmapJson(const nlohmann::json &data,
                                         bool check_vertex_validity,
                                         bool check_edge_validity)
{
  LazyJsonParser parser(data);
  fromRoadmapParser(parser, check_vertex_validity, check_edge_validity);
}

void VoxelCachedLazyPRM::loadRoadmapJson(const std::string &filename,
                                         bool check_vertex_validity,
                                         bool check_edge_validity)
{
  OMPL_INFORM("%s: reading roadmap from %s", getName().c_str(), filename.c_str());
  auto data = util::read_json(filename);
  this->fromRoadmapJson(data, check_vertex_validity, check_edge_validity);
}

std::shared_ptr<cpptoml::table> VoxelCachedLazyPRM::toRoadmapToml() const {
  auto space = si_->getStateSpace();

  auto container = cpptoml::make_table();
  auto tbl = cpptoml::make_table();
  container->insert("VoxelCachedLazyPRM_roadmap", tbl);

  auto vertices = cpptoml::make_table_array();
  tbl->insert("vertices", vertices);

  auto edges = cpptoml::make_table_array();
  tbl->insert("edges", edges);

  // populate vertices
  for (const Vertex v : boost::vertices(g_)) {
    auto local_tbl = cpptoml::make_table();
    vertices->push_back(local_tbl);
    local_tbl->insert("index", indexProperty_[v]);

    auto state = stateProperty_[v];
    std::vector<double> robot_state;
    space->copyToReals(robot_state, state);
    local_tbl->insert("state", cpptoml::to_toml(robot_state));

    auto voxels = vertexVoxelsProperty_[v];
    if (voxels) {
      local_tbl->insert("voxels", voxels->to_toml());
    }

    auto tip = tipPositionProperty_[v];
    if (tip.has_value()) {
      local_tbl->insert("tip_pos", cpptoml::to_toml(*tip));
    }
  }

  // populate edges
  for (const Edge e : boost::edges(g_)) {
    auto local_tbl = cpptoml::make_table();
    edges->push_back(local_tbl);
    local_tbl->insert("source", indexProperty_[boost::source(e, g_)]);
    local_tbl->insert("target", indexProperty_[boost::target(e, g_)]);
    local_tbl->insert("weight", weightProperty_[e].value());

    auto voxels = edgeVoxelsProperty_[e];
    if (voxels) {
      local_tbl->insert("voxels", voxels->to_toml());
    }
  }

  return container;
}

void VoxelCachedLazyPRM::saveRoadmapToml(const std::string &filename) const {
  OMPL_INFORM("%s: writing roadmap to %s", getName().c_str(), filename.c_str());
  cpptoml::to_file(this->toRoadmapToml(), filename);
}

void VoxelCachedLazyPRM::fromRoadmapToml(std::shared_ptr<cpptoml::table> tbl,
                                         bool check_vertex_validity,
                                         bool check_edge_validity)
{
  LazyTomlParser parser(tbl);
  fromRoadmapParser(parser, check_vertex_validity, check_edge_validity);
}

void VoxelCachedLazyPRM::loadRoadmapToml(const std::string &fname,
                                         bool check_vertex_validity,
                                         bool check_edge_validity)
{
  OMPL_INFORM("%s: reading roadmap from %s", getName().c_str(), fname.c_str());
  auto start = otime::now();
  std::shared_ptr<cpptoml::table> tbl;
  cpptoml::act_fin(fname, [&tbl](std::istream &in) {
    auto toml_parser = ::cpptoml::parser(in);
    tbl = toml_parser.parse();
  });
  auto stop = otime::now();
  OMPL_INFORM("%s: loading of roadmap file completed in %lf seconds",
              getName().c_str(), otime::seconds(stop - start));

  start = otime::now();
  fromRoadmapToml(tbl, check_vertex_validity, check_edge_validity);
  stop = otime::now();
  OMPL_INFORM("%s: generation of roadmap datastructure completed in %lf seconds",
              getName().c_str(), otime::seconds(stop - start));
}

void VoxelCachedLazyPRM::saveRoadmapDat(std::ostream &out) const {
  auto space = si_->getStateSpace();
  DatStreamer writer(out);
  this->toRoadmapWriter(writer);
}

void VoxelCachedLazyPRM::loadRoadmapDat(std::istream &in,
                                        bool check_vertex_validity,
                                        bool check_edge_validity)
{
  util::SingleStopWatch timer;
  OMPL_DEBUG("%s: starting parallel parser", getName().c_str());

  timer.start();
  ParallelDatParser parser(in, util::JsonFormat::JSON);
  timer.stop();

  OMPL_DEBUG("%s: finished parallel parser (%f secs)",
             getName().c_str(), timer.secs());

  fromRoadmapParser(parser, check_vertex_validity, check_edge_validity);
}

void VoxelCachedLazyPRM::saveRoadmapToFile(const std::string &fname) const {
  auto start = otime::now();
  if (util::endswith(fname, ".toml") || util::endswith(fname, ".toml.gz")) {
    saveRoadmapToml(fname);
  } else if (util::endswith(fname, ".dat") || util::endswith(fname, ".dat.gz")) {
    cpptoml::act_fout(fname, [this](std::ostream &out) {
      this->saveRoadmapDat(out);
    });
  } else if (util::endswith(fname, ".rmp")) {
    VoxelPtr reference;
    for (auto v : boost::vertices(g_)) {
      reference = vertexVoxelsProperty_[v];
      if (reference) { break; }
    }
    if (!reference) {
      for (auto e : boost::edges(g_)) {
        reference = edgeVoxelsProperty_[e];
        if (reference) { break; }
      }
    }
    BinaryOFStream out(fname);
    RmpStreamer writer(out, boost::num_vertices(g_), boost::num_edges(g_));
    if (reference) { writer.write_reference_voxels(*reference); }
    this->toRoadmapWriter(writer);
  } else {
    saveRoadmapJson(fname);
  }
  auto stop = otime::now();
  OMPL_INFORM("%s: roadmap saving completed in %lf seconds",
              getName().c_str(), otime::seconds(stop - start));
}

void VoxelCachedLazyPRM::loadRoadmapFromFile(const std::string &fname,
                                             bool check_vertex_validity,
                                             bool check_edge_validity)
{
  auto start = otime::now();
  if (util::endswith(fname, ".toml") || util::endswith(fname, ".toml.gz")) {
    return loadRoadmapToml(fname, check_vertex_validity, check_edge_validity);
  } else if (util::endswith(fname, ".dat") || util::endswith(fname, ".dat.gz")) {
    cpptoml::act_fin(fname,
        [this, check_vertex_validity, check_edge_validity] (std::istream &in) {
          this->loadRoadmapDat(in, check_vertex_validity, check_edge_validity);
        });
  } else if (util::endswith(fname, ".rmp")) {
    BinaryIFStream in(fname);
    LazyRmpParser parser(in);
    fromRoadmapParser(parser, check_vertex_validity, check_edge_validity);
  } else {
    return loadRoadmapJson(fname, check_vertex_validity, check_edge_validity);
  }
  auto stop = otime::now();
  OMPL_INFORM("%s: roadmap loading completed in %lf seconds from '%s'",
              getName().c_str(), otime::seconds(stop - start), fname.c_str());
}

void VoxelCachedLazyPRM::addMissingEdges() {
  size_type before = boost::num_edges(g_);
  for (auto v : boost::vertices(g_)) {
    const auto &neighbors = connectionStrategy_(v);
    for (auto n : neighbors) {
      if (!getEdge(v, n)) {
        connectVertices(v, n);
      }
    }
  }
  OMPL_INFORM("%s: added missing edges, from %d to %d",
              getName().c_str(), before, boost::num_edges(g_));
}


// ================
//  Protected Methods
// ================


// TODO: separate roadmap parsing out as its own thing?
void VoxelCachedLazyPRM::fromRoadmapParser(RoadmapParser &parser,
                                           bool check_vertex_validity,
                                           bool check_edge_validity)
{
  // Note: all roadmap formats that we create write all vertices before writing
  // any edges.  We can make use of that assumption to reduce memory usage
  // while loading if checking for validity of vertices and edges.
  // But, we check this assumption and throw an exception if that happens.

  util::SingleStopWatch timer;
  checkValidity();
  if (check_vertex_validity || check_edge_validity) {
    // make sure we have a valid motion and state validator
    if (!setup_) {
      OMPL_ERROR("%s: Could not complete setup before fromRoadmapParser(), "
                 "skipping validity check", getName().c_str());
      check_vertex_validity = false;
      check_edge_validity = false;
    }
  }

  size_type collided_vertices = 0;
  size_type collided_edges = 0;

  OMPL_DEBUG("%s: (loading) parsing all vertices", getName().c_str());
  timer.start();
  std::vector<std::pair<ParsedVertex, bool>> parsed_vertices;
  while (parser.next() == ParsedType::VERTEX) {
    parser.populate_voxels();
    parsed_vertices.emplace_back(parser.current_vertex(), true);
  }
  timer.stop();
  OMPL_DEBUG("%s: (loading) parsing %lu vertices finished (%f secs)",
             getName().c_str(), parsed_vertices.size(), timer.secs());

  // check vertex validity in parallel
  if (check_vertex_validity) {
    OMPL_DEBUG("%s: (loading) validating %lu vertices in parallel",
               getName().c_str(), parsed_vertices.size());
    timer.start();
    #pragma omp parallel for
    for (size_t i = 0; i < parsed_vertices.size(); ++i) {
      auto &[v, is_valid] = parsed_vertices[i];
      if (!v.voxels) {
        auto fk_result = voxelStateChecker_->fk(v.state);
        auto &fk_shape = fk_result.first;
        auto &home_shape = fk_result.second;
        is_valid = voxelStateChecker_->is_valid_shape(fk_shape, home_shape);
        if (is_valid) {
          v.voxels = std::make_shared<collision::VoxelOctree>(
              voxelStateChecker_->voxelize(fk_shape));
        }
      }
      is_valid = v.voxels && !voxelStateChecker_->collides(*v.voxels);
    }
    timer.stop();
    OMPL_DEBUG("%s: (loading) done validating %lu vertices in parallel (%f secs)",
               getName().c_str(),  parsed_vertices.size(), timer.secs());
  }

  std::unordered_map<size_type, Vertex> vertex_map;
  auto space = si_->getStateSpace();

  // add vertices to graph
  OMPL_DEBUG("%s: (loading) placing vertices into graph", getName().c_str());
  timer.start();
  for (auto &[v, is_valid] : parsed_vertices) {
    if (!is_valid) {
      ++collided_vertices;
      continue;
    }

    auto state = this->si_->allocState();
    space->copyFromReals(state, v.state);
    Vertex graph_vertex = addMilestone(state, false);

    indexProperty_[graph_vertex] = v.index;
    vertex_map[v.index] = graph_vertex;
    vertexVoxelsProperty_[graph_vertex] = v.voxels;
    tipPositionProperty_[graph_vertex] = v.tip_pos;
    if (v.tip_pos.has_value()) {
      nnTip_->add(VertexAndTip{graph_vertex, *v.tip_pos});
    }
    vertexValidityProperty_[graph_vertex] =
        check_vertex_validity ? VALIDITY_TRUE : VALIDITY_UNKNOWN;
  }
  timer.stop();
  OMPL_DEBUG("%s: (loading) done placing %d of %d vertices into graph "
             "(%d in collision) (%f secs)",
             getName().c_str(), vertex_map.size(), parsed_vertices.size(),
             collided_vertices, timer.secs());

  // parse edges
  std::vector<std::pair<ParsedEdge, bool>> parsed_edges;
  {
    OMPL_DEBUG("%s: (loading) parsing all edges",
               getName().c_str());
    timer.start();
    size_t n_skipped = 0;
    while (parser.current_type() == ParsedType::EDGE) {
      auto e = parser.current_edge();
      bool is_skipped = (vertex_map.count(e.source) == 0
                      || vertex_map.count(e.target) == 0);
      n_skipped += int(is_skipped);
      if (!is_skipped) {
        parser.populate_voxels();  // only populate voxels if not skipped
        parsed_edges.emplace_back(parser.current_edge(), true);
      }
      parser.next(); // go to the next item
    }
    if (parser.current_type() != ParsedType::DONE) {
      throw ompl::Exception(getName(),
                            "Parser returned a non-edge after parsing edges");
    }
    timer.stop();
    OMPL_DEBUG("%s: (loading) done parsing %lu/%lu edges,"
               " skipped %lu b/c of removed vertices (%f secs)",
               getName().c_str(), parsed_edges.size(),
               parsed_edges.size() + n_skipped, n_skipped,
               timer.secs());
  }

  // check edge validity
  if (check_edge_validity) {
    OMPL_DEBUG("%s: (loading) validating %lu edges in parallel",
               getName().c_str(), parsed_edges.size());
    timer.start();
    const auto ereport_n = std::max(size_t(1000), parsed_edges.size() / 100);
    size_t n_done = 0, n_collides = 0, n_voxelized = 0;
    #pragma omp parallel for shared(n_done, n_collides, n_voxelized)
    for (size_t i = 0; i < parsed_edges.size(); ++i) {
      auto &[e, is_valid] = parsed_edges[i];
      bool is_voxelized = false;

      // see if we need to voxelize
      bool is_in_collision = false;
      if (!e.voxels) {
        is_voxelized = true;
        Vertex source = vertex_map[e.source];
        Vertex target = vertex_map[e.target];
        auto state_a = voxelMotionValidator_->extract_from_state(
            stateProperty_[source]);
        auto state_b = voxelMotionValidator_->extract_from_state(
            stateProperty_[target]);
        auto partial = voxelMotionValidator_->voxelize(state_a, state_b);
        is_valid = partial.is_fully_valid;
        if (is_valid) {
          e.voxels = std::make_shared<collision::VoxelOctree>(partial.voxels);
        }
      }
      if (e.voxels) {
        is_in_collision = voxelMotionValidator_->collides(*e.voxels);
        is_valid = !is_in_collision;
      }

      #pragma omp critical
      {
        n_done++;
        n_collides  += int(is_in_collision);
        n_voxelized += int(is_voxelized);
        const auto n_valid = n_done - n_collides;
        if (n_done % ereport_n == 0) {
          OMPL_DEBUG("%s: Finished with %d/%d edges (%d%%)"
                     ", %d collisions, %d voxelized, %d valid",
                     getName().c_str(), n_done, parsed_edges.size(),
                     int (std::round(double(100 * n_done) / parsed_edges.size())),
                     n_collides, n_voxelized, n_valid);
        }
      }
    }
    timer.stop();
    OMPL_DEBUG("%s: (loading) done validating edges in parallel (%f secs)",
               getName().c_str(), timer.secs());
  }

  // add edges to graph
  OMPL_DEBUG("%s: (loading) adding edges to graph", getName().c_str());
  timer.start();
  for (auto &[e, is_valid] : parsed_edges) {
    if (!is_valid) {
      ++collided_edges;
      continue;
    }

    auto edge = connectVertices(vertex_map[e.source], vertex_map[e.target],
                                ob::Cost(e.weight));
    if (edge) {
      edgeVoxelsProperty_[edge.value()] = e.voxels;
      if (check_edge_validity) {
        edgeValidityProperty_[edge.value()] = VALIDITY_TRUE;
      }
    }
  }
  timer.stop();
  OMPL_DEBUG("%s: (loading) done adding %d of %d edges to graph "
             "(%d in collision) (%f secs)",
             getName().c_str(), parsed_edges.size() - collided_edges,
             parsed_edges.size(), collided_edges, timer.secs());

  OMPL_DEBUG("%s: after roadmap loaded from parser: %u verts, %u edges",
             getName().c_str(), boost::num_vertices(g_), boost::num_edges(g_));

  // check connected components
  int num_nonempty_components = 0;
  size_type largest_component_size = 0;
  size_type num_vertices = 0;
  for (size_type i = 0; i < componentCount_; ++i) {
    if (componentSize_[i] > boost::num_vertices(g_)) {
      OMPL_WARN("%s: component %d has negative vertices at %d",
                getName().c_str(), i, componentSize_[i]);
    } else if (componentSize_[i] > 0) {
      ++num_nonempty_components;
      num_vertices += componentSize_[i];
      //OMPL_DEBUG("%s: component %d has %d vertices", getName().c_str(), i,
      //           componentSize_[i]);
      if (componentSize_[i] > largest_component_size) {
        largest_component_size = componentSize_[i];
      }
    }
  }
  OMPL_DEBUG("%s: %d components", getName().c_str(), num_nonempty_components);
  OMPL_DEBUG("%s: largest component has %d of %d vertices",
             getName().c_str(), largest_component_size, boost::num_vertices(g_));
}

void VoxelCachedLazyPRM::toRoadmapWriter(RoadmapWriter &writer) const {
  // write all of the vertices
  {
    auto space = si_->getStateSpace();
    std::vector<double> robot_state;
    for (const Vertex v: boost::vertices(g_)) {
      space->copyToReals(robot_state, stateProperty_[v]);
      writer.write_vertex(
          indexProperty_[v],
          robot_state,
          tipPositionProperty_[v],
          vertexVoxelsProperty_[v]);
    }
  }

  // write all of the edges
  for (const Edge e : boost::edges(g_)) {
    writer.write_edge(
        indexProperty_[boost::source(e, g_)],
        indexProperty_[boost::target(e, g_)],
        weightProperty_[e].value(),
        edgeVoxelsProperty_[e]);
  }
}

bool VoxelCachedLazyPRM::computeVertexValidity(Vertex v) {
  auto &vd = vertexValidityProperty_[v];
  // if we don't know it to be valid, then check and set
  if ((vd & VALIDITY_TRUE) == 0) {
    const bool is_valid_shape = voxelizeVertex(v);
    auto voxels = vertexVoxelsProperty_[v];
    if (is_valid_shape && !voxelStateChecker_->collides(*voxels)) {
       vd |= VALIDITY_TRUE;
    }
  }
  return (vd & VALIDITY_TRUE);
}

bool VoxelCachedLazyPRM::computeEdgeValidity(Edge e) {
  auto &evd = edgeValidityProperty_[e];
  // if we don't know it to be valid, then check and set
  if ((evd & VALIDITY_TRUE) == 0) {
    const bool is_valid_shape = voxelizeEdge(e);
    auto voxels = edgeVoxelsProperty_[e];
    if (is_valid_shape && !voxelMotionValidator_->collides(*voxels)) {
      evd |= VALIDITY_TRUE;
    }
  }
  return (evd & VALIDITY_TRUE);
}

void VoxelCachedLazyPRM::uniteComponents(Vertex a, Vertex b) {
  auto componentA = vertexComponentProperty_[a];
  auto componentB = vertexComponentProperty_[b];
  if (componentA == componentB) {
    return;
  }
  if (componentSize_[componentA] > componentSize_[componentB]) {
    std::swap(componentA, componentB);
    std::swap(a, b);
  }
  markComponent(a, componentB);
}

void VoxelCachedLazyPRM::markComponent(Vertex v, uint32_t newComponent) {
  std::queue<Vertex> q;
  q.push(v);
  while (!q.empty()) {
    Vertex n = q.front();
    q.pop();
    auto &component = vertexComponentProperty_[n];
    if (component == newComponent) {
      continue;
    }
    if (componentSize_[component] == 1) {
        componentSize_.erase(component);
    } else {
        componentSize_[component]--;
    }
    component = newComponent;
    componentSize_[newComponent]++;
    boost::graph_traits<Graph>::adjacency_iterator nbh, last;
    for (boost::tie(nbh, last) = boost::adjacent_vertices(n, g_);
         nbh != last; ++nbh)
    {
      q.push(*nbh);
    }
  }
}

long int VoxelCachedLazyPRM::solutionComponent(
    std::pair<size_type, size_type> *startGoalPair) const
{
  for (size_type startIndex = 0; startIndex < startM_.size(); ++startIndex) {
    long int startComponent = vertexComponentProperty_[startM_[startIndex]];
    for (size_type goalIndex = 0; goalIndex < goalM_.size(); ++goalIndex) {
      if (startComponent == (long int)vertexComponentProperty_[goalM_[goalIndex]]) {
        startGoalPair->first = startIndex;
        startGoalPair->second = goalIndex;
        return startComponent;
      }
    }
  }
  return -1;
}

// TODO: return [is_exact, pathptr] = std::pair<bool, ob::PathPtr>
ob::PathPtr VoxelCachedLazyPRM::constructSolution(const Vertex &start,
                                                  const Vertex &goal)
{
  // Need to update the index map here, because nodes may have been removed and
  // the numbering will not be 0 .. N-1 otherwise.
  renumberIndices();

  if (start == goal) {
    OMPL_INFORM("%s: start and goal states are identical", getName().c_str());
    auto p(std::make_shared<og::PathGeometric>(si_));
    p->append(stateProperty_[start]);
    return p;
  }

  auto prev = astarSearch(start, goal);

  // First, get the solution states without copying them, and check them for
  // validity.  We do all the node validity checks for the vertices, as this
  // may remove a larger part of the graph (compared to removing an edge).
  // TODO: parallelize with OpenMP?
  std::vector<const ob::State *> states(1, stateProperty_[goal]);
  std::set<Vertex> milestonesToRemove;
  for (Vertex pos = prev[goal]; prev[pos] != pos; pos = prev[pos]) {
    if (!computeVertexValidity(pos)) { milestonesToRemove.insert(pos); }
    if (milestonesToRemove.empty()) { states.push_back(stateProperty_[pos]); }
  }

  // We remove *all* invalid vertices. This is not entirely as described in the
  // original VoxelCachedLazyPRM paper, as the paper suggest removing the first
  // vertex only, and then recomputing the shortest path. However, the paper
  // says the focus is on efficient vertex & edge removal, rather than
  // collision checking, so this modification is in the spirit of the paper.
  if (!milestonesToRemove.empty()) {
    OMPL_DEBUG("%s: astar path is invalid, removing %d vertices",
               getName().c_str(), milestonesToRemove.size());
    timers_["remove_vertices"].time([this, &start, &milestonesToRemove]() {
          auto comp = vertexComponentProperty_[start];
          auto neighbors = removeVertices(milestonesToRemove);
          // Update the connected component ID for neighbors.
          for (auto neighbor : neighbors) {
            if (comp == vertexComponentProperty_[neighbor]) {
              auto newComponent = componentCount_++;
              componentSize_[newComponent] = 0;
              markComponent(neighbor, newComponent);
            }
          }
        });
    // TODO: return the partial path up until the first unknown edge or invalid
    // TODO- vertex from the candidate path for a partial solution.
    return ob::PathPtr(); // return an empty path
  }

  // start is checked for validity already
  states.push_back(stateProperty_[start]);

  // Check the edges too, if the vertices were valid. Remove the first invalid
  // edge only.
  std::vector<const ob::State *>::const_iterator prevState = states.begin();
  std::vector<const ob::State *>::const_iterator state = prevState + 1;
  Vertex prevVertex = goal, pos = prev[goal];
  do {
    Edge e = boost::lookup_edge(pos, prevVertex, g_).first;
    if (!computeEdgeValidity(e)) {
      OMPL_DEBUG("%s: astar path is invalid, removing 1 edge", getName().c_str());
      removeEdge(e);
      // TODO: return the partial path up until this edge instead
      // TODO- for a partial solution
      return ob::PathPtr();
    }
    prevState = state;
    ++state;
    prevVertex = pos;
    pos = prev[pos];
  } while (prevVertex != pos);

  auto p(std::make_shared<og::PathGeometric>(si_));
  for (std::vector<const ob::State *>::const_reverse_iterator st = states.rbegin();
       st != states.rend(); ++st)
  {
      p->append(*st);
  }
  return p;
}

ob::Cost VoxelCachedLazyPRM::costHeuristic(Vertex u, Vertex v) const {
  return opt_->motionCostHeuristic(stateProperty_[u], stateProperty_[v]);
}

std::pair<VoxelCachedLazyPRM::Vertex, bool> VoxelCachedLazyPRM::tryAddToGraph(
    ob::State *state)
{
  // if the nearest neighbors data structure has a state in it...
  // use nearest neighbors to find the nearest vertex to the state
  //   - this is done by adding the state to the graph,
  //     doing nearest neighbors using the new Vertex
  //   - check for equal states using the space information
  //   - remove the new vertex if they are the same
  // if the states are equal, then return the found vertex.
  // otherwise return the new vertex.

  Vertex vnew = boost::add_vertex(g_);
  stateProperty_[vnew] = state;
  if (nn_->size() > 0) {
    Vertex near = nn_->nearest(vnew);
    auto near_state = stateProperty_[near];
    if (si_->equalStates(near_state, state)) {
      OMPL_DEBUG("%s: state already in graph", getName().c_str());
      boost::remove_vertex(vnew, g_);
      return {near, false};
    }
  }
  return {vnew, true};
}

bool VoxelCachedLazyPRM::voxelizeVertex(const Vertex v) {
  // make sure we have voxelStateChecker_
  checkValidity();
  if (!voxelStateChecker_) {
    OMPL_WARN("%s: Could not precompute voxels, missing voxel state checker",
              getName().c_str());
    return true;
  }

  // generate the voxel cache and the tip position
  bool is_valid_shape = true;
  if (!vertexVoxelsProperty_[v] || !tipPositionProperty_[v]) {
    auto st = stateProperty_[v];
    auto robot_state = voxelStateChecker_->extract_from_state(st);
    auto fk_result = voxelStateChecker_->fk(robot_state);
    auto &fk_shape = fk_result.first;
    auto &home_shape = fk_result.second;
    is_valid_shape = voxelStateChecker_->is_valid_shape(fk_shape, home_shape);
    if (is_valid_shape) {
      vertexVoxelsProperty_[v] = std::make_shared<collision::VoxelOctree>(
          voxelStateChecker_->voxelize(fk_shape));
      auto &tip = tipPositionProperty_[v];
      if (tip && fk_shape.p.back() != *tip) {
        nnTip_->remove(VertexAndTip{v, *tip});
        tip = {};
      }
      if (!tip) {
        tipPositionProperty_[v] = fk_shape.p.back();
        #pragma omp critical
        nnTip_->add(VertexAndTip{v, fk_shape.p.back()});
      }
    }
  }
  return is_valid_shape;
}

std::optional<VoxelCachedLazyPRM::Edge>
VoxelCachedLazyPRM::getEdge(const Vertex v, const Vertex n) {
  auto [edge, edge_exists] = boost::edge(v, n, g_);
  if (edge_exists) { return edge; }
  return {};
}

std::optional<VoxelCachedLazyPRM::Edge>
VoxelCachedLazyPRM::connectVertices(const Vertex a, const Vertex b) {
  // Note: we don't check for an existing edge because there are times when we
  // know it cannot exist, and checking is potentially expensive.
  // If the edge may exist, check before calling this function
  if (a == b || (connectionFilter_ && !connectionFilter_(a, b))) {
    return {}; // no edge
  }
  const ob::Cost weight = opt_->motionCost(stateProperty_[a], stateProperty_[b]);
  const Graph::edge_property_type properties(weight);
  const Edge &e = boost::add_edge(a, b, properties, g_).first;
  edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
  uniteComponents(a, b);
  return e; // yes edge
}

std::optional<VoxelCachedLazyPRM::Edge>
VoxelCachedLazyPRM::connectVertices(const Vertex a, const Vertex b,
                                    ob::Cost weight)
{
  // Note: we don't check for an existing edge because there are times when we
  // know it cannot exist, and checking is potentially expensive.
  // If the edge may exist, check before calling this function
  if (a == b || (connectionFilter_ && !connectionFilter_(a, b))) {
    return {}; // no edge
  }
  const Graph::edge_property_type properties(weight);
  const Edge &e = boost::add_edge(a, b, properties, g_).first;
  edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
  uniteComponents(a, b);
  return e; // yes edge
}

bool VoxelCachedLazyPRM::voxelizeEdge(const Edge e) {
  // make sure we have voxelStateChecker_
  checkValidity();
  if (!voxelMotionValidator_) {
    OMPL_WARN("%s: Could not precompute voxels, missing voxel motion validator",
              getName().c_str());
    return true;
  }

  bool is_valid = true;
  if (!edgeVoxelsProperty_[e]) { // generate the voxel cache
    auto robot_state_a = voxelMotionValidator_->extract_from_state(
        stateProperty_[boost::source(e, g_)]);
    auto robot_state_b = voxelMotionValidator_->extract_from_state(
        stateProperty_[boost::target(e, g_)]);
    auto partial = voxelMotionValidator_->voxelize(robot_state_a, robot_state_b);
    is_valid = partial.is_fully_valid;
    if (is_valid) {
      edgeVoxelsProperty_[e] =
          std::make_shared<collision::VoxelOctree>(partial.voxels);
    }
  }
  return is_valid;
}

std::set<VoxelCachedLazyPRM::Vertex> VoxelCachedLazyPRM::removeVertices(
    const std::set<Vertex> &vertices)
{
  // Remember the current neighbors.
  std::set<Vertex> neighbors;
  for (auto v : vertices) {
    boost::graph_traits<Graph>::adjacency_iterator nbh, last;
    for (auto neighbor : boost::adjacent_vertices(v, g_)) {
      if (vertices.find(neighbor) == vertices.end()) {
        neighbors.insert(neighbor);
      }
    }
    // Remove vertex from nearest neighbors data structure.
    nn_->remove(v);
    auto &tip = tipPositionProperty_[v];
    if (tip.has_value()) {
      nnTip_->remove(VertexAndTip{v, *tip});
    }
    // Free vertex state.
    si_->freeState(stateProperty_[v]);
    // Remove all edges.
    boost::clear_vertex(v, g_);
    // Remove the vertex.
    boost::remove_vertex(v, g_);
  }
  return neighbors;
}

void VoxelCachedLazyPRM::removeEdge(const Edge e) {
  boost::remove_edge(e, g_);
  auto newComponent = componentCount_++;
  componentSize_[newComponent] = 0;
  markComponent(boost::source(e, g_), newComponent);
}

void VoxelCachedLazyPRM::renumberIndices() {
  // Need to update the index map here, because nodes may have been removed and
  // the numbering will not be 0 .. N-1 otherwise.
  timers_["reindex"].time([this]() {
        size_type idx = 0;
        for (const Vertex v : boost::vertices(g_)) {
          indexProperty_[v] = idx++;
        }
      });
}

VoxelCachedLazyPRM::PredecessorType
VoxelCachedLazyPRM::astarSearch(const Vertex start, const Vertex goal) {
  boost::property_map<Graph, boost::vertex_predecessor_t>::type prev;
  timers_["astar"].time([this, &start, &goal, &prev]() {
        try {
          // Consider using a persistent distance_map if it's slow
          boost::astar_search(
              g_, start,
              [this, goal](Vertex v) { return costHeuristic(v, goal); },
              boost::predecessor_map(prev)
                  .distance_compare([this](ob::Cost c1, ob::Cost c2)
                      { return opt_->isCostBetterThan(c1, c2); })
                  .distance_combine([this](ob::Cost c1, ob::Cost c2)
                      { return opt_->combineCosts(c1, c2); })
                  .distance_inf(opt_->infiniteCost())
                  .distance_zero(opt_->identityCost())
                  .visitor(AStarGoalVisitor<Vertex>(goal)));
        }
        catch (AStarFoundGoal &) { }
      });

  if (prev[goal] == goal) {
    throw ompl::Exception(name_, "Could not find solution path");
  }

  return prev;
}

ob::PlannerStatus VoxelCachedLazyPRM::solvePrep(
    const ob::PlannerTerminationCondition &ptc)
{
  checkValidity();

  if (!setup_) {
    OMPL_ERROR("%s: Could not complete setup before solve()", getName().c_str());
    return ob::PlannerStatus::ABORT;
  }

  auto goalRegion = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

  if (goalRegion == nullptr) {
    OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  // Add the valid start states as milestones
  while (const ob::State *st = pis_.nextStart()) {
    startM_.push_back(addMilestone(si_->cloneState(st)));
  }

  if (startM_.empty()) {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return ob::PlannerStatus::INVALID_START;
  }

  if (!goalRegion->couldSample()) {
    OMPL_ERROR("%s: Insufficient states in sampleable goal region",
               getName().c_str());
    return ob::PlannerStatus::INVALID_GOAL;
  }

  // Ensure there is at least one valid goal state
  if (goalRegion->maxSampleCount() > goalM_.size() || goalM_.empty()) {
    const ob::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
    if (st != nullptr) {
      goalM_.push_back(addMilestone(si_->cloneState(st)));
    }

    if (goalM_.empty()) {
      OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
      return ob::PlannerStatus::INVALID_GOAL;
    }
  }

  return ob::PlannerStatus::APPROXIMATE_SOLUTION;
}

void VoxelCachedLazyPRM::setDefaultIkController(int max_iters,
                                                double stop_threshold,
                                                double mu_init)
{
  if (!si_->isSetup()) { si_->setup(); } // just to be safe :)
  if (!voxelStateChecker_) {
    voxelStateChecker_ = std::dynamic_pointer_cast<AbstractVoxelValidityChecker>(
        si_->getStateValidityChecker());
    if (!voxelStateChecker_) {
      OMPL_INFORM("%s: state validity checker is not set, cannot set default IK"
                  " controller", getName().c_str());
      return;
    }
  }

  const auto &robot = voxelStateChecker_->robot();
  // define the segment length for IK in terms of just the tension space.
  // The IK does not utilize the weights between spaces for distance (although
  // it could be added).
  auto seglen = si_->getStateSpace()
                   ->as<ob::CompoundStateSpace>()
                   ->getSubspace(0)
                   ->getLongestValidSegmentLength();

  tip_control::FKFunc fk_func = [this](const auto &state) {
    auto fk_result = this->voxelStateChecker_->fk(state);
    return fk_result.first.p.back();
  };

  this->setIkController(
    [this, fk_func=std::move(fk_func), robot, max_iters, stop_threshold,
     mu_init, seglen]
    (const std::vector<double> &start_control, const E::Vector3d &goal) {
      // TODO: are these good default values?
      const double stop_threshold_JT_err_inf = seglen / 10.0; // gradient step
      const double stop_threshold_Dp         = seglen / 20.0; // delta p
      const double stop_threshold_err        = stop_threshold;
      const double finite_difference_delta   = seglen / 4.0;
      const bool   verbose    = ompl::msg::getLogLevel() <= ompl::msg::LOG_DEBUG;

      // run the full Levenberg Marquardt controller
      auto result = tip_control::inverse_kinematics(
          fk_func,
          robot,
          start_control,
          goal,
          max_iters,
          mu_init,
          stop_threshold_JT_err_inf,
          stop_threshold_Dp,
          stop_threshold_err,
          finite_difference_delta,
          verbose
      );

      // issue a warning if it ends because max_iters was reached
      //if (result.iters == max_iters) {
      //  auto tip_pos = controller.robot().forward_kinematics(result.state).back();
      //  OMPL_DEBUG("%s: IK exited due to max iterations for (%lg, %lg, %lg)"
      //             " reaching (%lg, %lg, %lg)",
      //             this->getName().c_str(),
      //             goal[0], goal[1], goal[2],
      //             tip_pos[0], tip_pos[1], tip_pos[2]);
      //}
      return result.state;
    });
}

std::optional<VoxelCachedLazyPRM::IKResult> VoxelCachedLazyPRM::roadmapIk(
    const E::Vector3d &request, double tolerance, size_type k,
    VoxelCachedLazyPRM::RoadmapIkOpts opt)
{
  const bool auto_add    = bool(opt & RMAP_IK_AUTO_ADD);
  const bool do_accurate = bool(opt & RMAP_IK_ACCURATE);
  const bool do_lazy_add = bool(opt & RMAP_IK_LAZY_ADD);

  // TODO: implement use of do_accurate
  // TODO: implement use of do_lazy

  // make sure we have voxelStateChecker_ and ikController_
  checkValidity();
  if (!setup_) {
    OMPL_ERROR("%s: could not run setup before roadmapIk(), continuing anyway",
               getName().c_str());
  }

  // get k VALID nearest neighbors
  std::vector<VertexAndTip> neighbors;
  while (neighbors.empty()) {
    nnTip_->nearestK(VertexAndTip{{}, request}, k, neighbors);

    if (neighbors.empty()) {
      OMPL_ERROR("%s::roadmapIk(): no neighbors were able to be found",
                 getName().c_str());
      throw ompl::Exception("roadmapIk(): No neighbors were able to be found");
    }

    std::set<Vertex> vertices_to_remove;
    for (auto neighbor : neighbors) {
      auto v = neighbor.vertex;
      if (!computeVertexValidity(v)) {
        vertices_to_remove.insert(v);
      }
    }
    // if there were some to remove, redo nearest neighbor query
    if (vertices_to_remove.size() > 0) {
      removeVertices(vertices_to_remove);
      neighbors.clear(); // clear it to trigger another query
      OMPL_INFORM("%s::roadmapIk(): Removed %u invalid nearest neighbor states",
                  getName().c_str(), vertices_to_remove.size());
    }
  }

  struct LocalResult {
    VertexAndTip neighbor;
    IKResult result;
    bool is_valid;
    std::shared_ptr<collision::VoxelOctree> voxels;
    std::vector<Vertex> nearest;
    // partial edges to nearest
    std::vector<VoxelEnvironment::PartialVoxelization> partial_edges;
  };

  auto space = si_->getStateSpace();

  // pseudo-code:
  //   neighbors = nearest_neighbors(request, k)
  //   iks = []
  //   for neighbor in neighbors:
  //       state, error = IK(neighbor.state, request)
  //       valid = is_valid(state)
  //       if error < tolerance and valid:
  //           if auto_add:
  //               add_to_roadmap(state)
  //           return state
  //       iks.append((neighbor, state, error, valid))
  std::vector<LocalResult> local_results(neighbors.size());
  for (size_t i = 0; i < neighbors.size(); ++i) {
    auto &neighbor   = neighbors[i];
    auto &res        = local_results[i];
    auto st          = stateProperty_[neighbor.vertex];
    auto start_state = voxelStateChecker_->extract_from_state(st);
    decltype(start_state) final_state;
    res.neighbor = neighbor;
    try {
      final_state = timers_["ik_controller"].time(
          [this, &start_state, &request]() {
            return ikController_(start_state, request);
          });
    } catch(std::invalid_argument &ex) {
      using util::operator<<;
      auto precision = std::numeric_limits<double>::max_digits10;
      std::ostringstream start_str, request_str;
      start_str   << std::setprecision(precision) << start_state;
      request_str << std::setprecision(precision) << request.transpose();
      OMPL_ERROR("%s: error in ikController \"%s\" with start=[%s], request=[%s]",
                 getName().c_str(), ex.what(),
                 start_str.str().c_str(), request_str.str().c_str());
      throw;
    }
    auto [fk_shape, home_shape] = voxelStateChecker_->fk(final_state);
    bool is_valid_shape = voxelStateChecker_->is_valid_shape(
                                    fk_shape, home_shape);
    auto voxels         = std::make_shared<collision::VoxelOctree>(
                                    voxelStateChecker_->voxelize(fk_shape));
    bool collides       = voxelStateChecker_->collides(*voxels);
    double error        = (fk_shape.p.back() - request).norm();
    res.result = IKResult{final_state, fk_shape.p.back(),
                          start_state, error};
    res.is_valid = is_valid_shape && !collides;
    res.voxels = voxels;

    // we've found a good one!
    if (is_valid_shape && !collides && error < tolerance && !auto_add) {
      OMPL_DEBUG("%s::roadmapIk(): IK #%d accepted, and not added to roadmap,"
                 " error: %lf (tol: %lf)",
                 this->getName().c_str(), i+1, error, tolerance);
      return res.result;
    }

    if (error >= tolerance) {
      OMPL_DEBUG("%s::roadmapIk(): IK #%d rejected,"
                 " error: %lf (tol: %lf)",
                 this->getName().c_str(), i+1, error, tolerance);
    }

    // if it's a good IK solution, even if not valid, try to connect anyway
    if (error < tolerance && auto_add) {
      OMPL_DEBUG("%s::roadmapIk(): IK #%d within tolerance,"
                 " trying to connect to roadmap,"
                 " error: %lf (tol: %lf)",
                 this->getName().c_str(), i+1, error, tolerance);
      // try to add it to the roadmap
      ob::State *state = si_->allocState();
      space->copyFromReals(state, final_state);
      space->enforceBounds(state);
      bool was_added;
      Vertex vertex = addMilestone(state, false, &was_added);

      // if there are already edges, then we're good because it was already in
      // the roadmap
      if (boost::out_degree(vertex, g_) > 0) {
        OMPL_DEBUG("%s::roadmapIk(): IK result already part of the roadmap"
                   " with %d edges",
                   this->getName().c_str(), boost::out_degree(vertex, g_));
        return res.result;
      }

      // go through the would-be edges and try to connect
      decltype(res.nearest) nearest;
      if (do_accurate) {
        nearest = connectionStrategy_(vertex);
        if (!util::contains(nearest, res.neighbor.vertex)) {
          nearest.emplace_back(res.neighbor.vertex);
        }
      } else {
        nearest = {res.neighbor.vertex}; // only to IK neighbor
      }
      res.partial_edges.reserve(nearest.size());
      OMPL_DEBUG("%s::roadmapIk(): IK #%d, calculating up to %d edges",
                 this->getName().c_str(), i+1, nearest.size());

      for (size_t j = 0; j < nearest.size(); j++) {
        Vertex source = nearest[j];
        if (!computeVertexValidity(source)) {
          if (source != vertex) {
            removeVertices({source});
          }
          OMPL_DEBUG("%s::roadmapIk() (@1): neighbor %d is invalid",
                     getName().c_str(), j+1);
          continue;
        }
        if (source == vertex) {
          OMPL_DEBUG("%s::roadmapIk(): State already in the roadmap, returning it.",
                     this->getName().c_str());
          tipPositionProperty_[vertex] = res.result.tip_position;
          vertexVoxelsProperty_[vertex] = res.voxels;
          vertexValidityProperty_[vertex] = VALIDITY_TRUE;
          return res.result;
        }
        res.nearest.emplace_back(source);
        auto source_state  = stateProperty_[source];
        auto source_rstate = voxelStateChecker_->extract_from_state(source_state);
        OMPL_DEBUG("voxelize_until_invalid # 1");
        res.partial_edges.emplace_back(
            voxelMotionValidator_->voxelize_until_invalid(
                source_rstate, final_state));
        auto &pe = res.partial_edges.back();;
        if (pe.is_fully_valid) {
          auto e = connectVertices(source, vertex);
          if (e) {
            OMPL_DEBUG("%s::roadmapIk(): Successfully connected neighbor %d of"
                       " %d with collision-free edge",
                       this->getName().c_str(), j+1, nearest.size());
            edgeValidityProperty_[*e] = VALIDITY_TRUE;
            edgeVoxelsProperty_[*e] =
                std::make_shared<collision::VoxelOctree>(pe.voxels);
            tipPositionProperty_[vertex] = res.result.tip_position;
            vertexVoxelsProperty_[vertex] = res.voxels;
            vertexValidityProperty_[vertex] = VALIDITY_TRUE;
            return res.result;
          } else {
            OMPL_ERROR("%s::roadmapIk(): found a collision-free edge, but could"
                       " not add to the roadmap", getName().c_str());
            throw std::runtime_error("Shouldn't happen...");
          }
        }
        // else potential edge was not fully valid
      }

      // remove the vertex since we couldn't reach it
      if (was_added) {
        this->removeVertices({vertex});
      }
      OMPL_DEBUG("%s::roadmapIk(): Could not reach IK #%d soln collision-free",
                 this->getName().c_str(), i+1);
    }
  }

  // if we're doing laziness, then return the closest valid one, or step
  // backwards until valid
  if (!auto_add) {
    double min_error = 0.0;
    size_t min_idx = local_results.size() + 1; // none
    for (size_t i = 0; i < local_results.size(); i++) {
      auto &res = local_results[i];
      if (res.is_valid &&
          (min_idx > local_results.size() || res.result.error < min_error))
      {
        min_error = res.result.error;
        min_idx = i;
      }
    }
    if (min_idx < local_results.size()) {
      OMPL_DEBUG("%s::roadmapIk(): All IKs rejected, returning closest valid one #%d,"
                 " tip error: %lf (tol: %lf)",
                 this->getName().c_str(), min_idx+1, min_error, tolerance);
      return local_results[min_idx].result;
    }

    OMPL_DEBUG("%s::roadmapIk(): All IKs are in collision, stepping them"
               " backwards", this->getName().c_str());
    double smallest_error = 0.0;
    size_t smallest_idx = local_results.size() + 1;
    size_t smallest_j = 10000;
    for (size_t i = 0; i < local_results.size(); i++) {
      auto &res = local_results[i];

      // temporarily add it to the roadmap
      ob::State *state = si_->allocState();
      space->copyFromReals(state, res.result.controls);
      space->enforceBounds(state);
      bool was_added;
      Vertex vertex = addMilestone(state, false, &was_added);

      decltype(res.nearest) nearest;
      // try to connect to nearest neighbors
      if (do_accurate) {
        // get nearest neighbors
        nearest = connectionStrategy_(vertex);
        if (!util::contains(nearest, res.neighbor.vertex)) {
          nearest.emplace_back(res.neighbor.vertex);
        }
      } else {
        nearest = {res.neighbor.vertex}; // only to IK neighbor
      }
      res.partial_edges.reserve(nearest.size());
      OMPL_DEBUG("%s::roadmapIk(): IK #%d, calculating up to %d edges",
                 this->getName().c_str(), i+1, nearest.size());


      for (size_t j = 0; j < nearest.size(); j++) {
        Vertex source = nearest[j];
        // only evaluate this edge if the source vertex is valid
        if (!computeVertexValidity(source)) {
          if (source != vertex) {
            removeVertices({source});
          }
          OMPL_DEBUG("%s::roadmapIk() (@2): neighbor %d is invalid",
                     getName().c_str(), j+1);
          continue;
        }
        res.nearest.emplace_back(source);
        auto n_state = stateProperty_[source];
        auto rn_state = voxelStateChecker_->extract_from_state(n_state);
        res.partial_edges.emplace_back(
            voxelMotionValidator_->voxelize_until_invalid(
                rn_state, res.result.controls));
        auto &pe = res.partial_edges.back();
        auto error = (pe.last_backbone.back() - request).norm();

        if (smallest_idx > local_results.size() || error < smallest_error) {
          smallest_error = error;
          smallest_idx = i;
          smallest_j = res.partial_edges.size()-1;
        }
      }

      // remove the vertex after going through nearest neighbors
      if (was_added) {
        this->removeVertices({vertex});
      }
    }

    OMPL_DEBUG("%s::roadmapIk(): returning closest one after stepping back, #%d.%d,"
               " error: %lf, tol: %lf", this->getName().c_str(),
               smallest_idx+1, smallest_j+1, smallest_error, tolerance);

    auto &res_close = local_results[smallest_idx];
    auto &pe_close = res_close.partial_edges[smallest_j];

    // double check
    {
      ob::ScopedState<> local_state(si_);
      local_state = pe_close.last_valid;
      if (!voxelStateChecker_->isValid(local_state.get())){
        OMPL_ERROR("%s::roadmapIk(): closest one is invalid!, #%d,"
                   " error: %lf, tol: %lf", this->getName().c_str(),
                   smallest_idx+1, smallest_error, tolerance);
        auto tip = res_close.neighbor.vec;
        OMPL_DEBUG("%s::roadmapIk(): res_close.neighbor: (v: %lu, tip: [%lf, %lf, %lf])",
            getName().c_str(), res_close.neighbor.vertex, tip[0], tip[1], tip[2]);
        tip = res_close.result.tip_position;
        OMPL_DEBUG("%s::roadmapIk(): res_close.tip_position: [%lf, %lf, %lf]",
            getName().c_str(), tip[0], tip[1], tip[2]);
        OMPL_DEBUG("%s::roadmapIk(): res_close.is_valid: %d",
            getName().c_str(), res_close.is_valid);
        OMPL_DEBUG("%s::roadmapIk(): &voxels: %x, nearest.size(): %lu",
            getName().c_str(), res_close.voxels.get(), res_close.nearest.size());
        OMPL_DEBUG("%s::roadmapIk(): &voxels: %x, pe_close.size(): %lu",
            getName().c_str(), res_close.voxels.get(), res_close.nearest.size());
        throw std::runtime_error("roadmapIk(): closest value is invalid");
      }
    }

    return IKResult{
      pe_close.last_valid,
      pe_close.last_backbone.back(),
      voxelStateChecker_->extract_from_state(
          stateProperty_[res_close.neighbor.vertex]),
      smallest_error
    };
  }

  //   # none of the IK is both within tolerance and valid
  //   # fix invalid ones with closest valid
  //   for i, ik in enumerate(iks):
  //       if not ik.valid:
  //           (ik.state, ik.error) = closest_valid(ik.neighbor.state, ik.state)
  //           iks[i] = ik # update in the list

  //   # return the one with the smallest error after correcting
  //   iks.sort(lambda x: x.error)
  //   closest = iks[0]
  //   if auto_add:
  //       add_to_roadmap(closest.state)
  //   return closest.state

  OMPL_DEBUG("%s::roadmapIk(): All IKs rejected, finding closest"
             " collision-free connection",
             this->getName().c_str());

  double smallest_error = -1;
  size_t closest_i = 0;
  size_t closest_j = 0;
  for (size_t i = 0; i < local_results.size(); ++i) {
    auto &res = local_results[i];
    // fill in ones that weren't calculated in the above for loop
    if (res.nearest.empty()) {
      // try to add it to the roadmap
      ob::State *state = si_->allocState();
      space->copyFromReals(state, res.result.controls);
      space->enforceBounds(state);
      bool was_added = false;
      Vertex vertex = addMilestone(state, false, &was_added);

      // already part of the roadmap, no need to connect it
      if (!was_added) {
        res.nearest.emplace_back(vertex);
        continue;
      }

      // go through the would-be edges and try to connect
      decltype(res.nearest) nearest;
      if (do_accurate) {
        nearest = connectionStrategy_(vertex);
        if (!util::contains(nearest, res.neighbor.vertex)) {
          nearest.emplace_back(res.neighbor.vertex);
        }
      } else {
        nearest = {res.neighbor.vertex}; // only to IK neighbor
      }
      res.partial_edges.reserve(nearest.size());
      OMPL_DEBUG("%s::roadmapIk(): IK #%d, calculating all %d edges",
                 this->getName().c_str(), i+1, nearest.size());

      for (size_t j = 0; j < nearest.size(); j++) {
        Vertex source = nearest[j];
        // only evaluate this edge if the source vertex is valid
        if (!computeVertexValidity(source)) {
          if (source != vertex) {
            removeVertices({source});
          }
          OMPL_DEBUG("%s::roadmapIk() (@3): neighbor %d is invalid",
                     getName().c_str(), j+1);
          continue;
        }
        res.nearest.emplace_back(source);
        auto source_state  = stateProperty_[source];
        auto source_rstate = voxelStateChecker_->extract_from_state(source_state);
        res.partial_edges.emplace_back(
            voxelMotionValidator_->voxelize_until_invalid(
                source_rstate, res.result.controls));
        // even if it's fully valid, it wasn't computed before because the IK
        // error was too big
      }

      // remove the vertex for now, we may add it back
      if (was_added) {
        this->removeVertices({vertex});
      }
    }

    // search over these neighbors to see if they're the closest
    for (size_t j = 0; j < res.nearest.size(); j++) {
      //auto neighbor = res.nearest[j];
      auto &pe = res.partial_edges[j];
      auto error = (pe.last_backbone.back() - request).norm();
      if (smallest_error < 0 || error < smallest_error) {
        smallest_error = error;
        closest_i = i;
        closest_j = j;
      }
    }
  }

  // connect and return the one that is the closest
  auto &res_close = local_results[closest_i];
  auto &nbr_close = res_close.nearest[closest_j];
  auto &pe_close  = res_close.partial_edges[closest_j];
  ob::State *state = si_->allocState();
  space->copyFromReals(state, pe_close.last_valid);
  //space->enforceBounds(state);

  // We connect the vertex to all neighbors (without cache) according to the
  // connection strategy, regardless of whether nbr_close is within that
  // connection strategy
  Vertex vertex = addMilestone(state, true); // connect to all neighbors lazily

  // set vertex cache
  tipPositionProperty_[vertex] = res_close.result.tip_position;
  vertexVoxelsProperty_[vertex] = res_close.voxels;
  vertexValidityProperty_[vertex] = VALIDITY_TRUE;

  if (vertex == nbr_close) {
    OMPL_INFORM("%s::roadmapIk(): Solution is already in the roadmap", getName().c_str());
  } else {
    // connect to the one I found if not already there
    auto e = getEdge(nbr_close, vertex);
    if (!e) { e = connectVertices(nbr_close, vertex); }

    if (e) {
      // populate this edge cache
      edgeValidityProperty_[*e] = VALIDITY_TRUE;
      edgeVoxelsProperty_[*e] = std::make_shared<collision::VoxelOctree>(
          pe_close.voxels);
    } else {
      OMPL_ERROR("%s::roadmapIk(): found a collision-free edge, but could"
                 " not add to the roadmap", this->getName().c_str());
    }

    if (!do_lazy_add) {
      for (auto e : boost::out_edges(vertex, g_)) {
        if (!computeEdgeValidity(e)) {
          removeEdge(e);
        }
      }
    }
  }

  OMPL_DEBUG("%s::roadmapIk(): Closest IK connection #%d,"
             " error: %lf (tol: %lf)",
             this->getName().c_str(), closest_i+1, smallest_error, tolerance);

  return IKResult{
    pe_close.last_valid,
    pe_close.last_backbone.back(),
    voxelStateChecker_->extract_from_state(stateProperty_[nbr_close]),
    smallest_error
  };
}

void VoxelCachedLazyPRM::getPlannerData(ob::PlannerData &data) const {
  Planner::getPlannerData(data);

  // Explicitly add start and goal states. Tag all states known to be valid as 1.
  // Unchecked states are tagged as 0.
  for (auto i : startM_) {
    data.addStartVertex(ob::PlannerDataVertex(stateProperty_[i], 1));
  }

  for (auto i : goalM_) {
    data.addGoalVertex(ob::PlannerDataVertex(stateProperty_[i], 1));
  }

  // Adding edges and all other vertices simultaneously
  for (const Edge e : boost::edges(g_)) {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]),
                 ob::PlannerDataVertex(stateProperty_[v2]));

    // Add the reverse edge, since we're constructing an undirected roadmap
    data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]),
                 ob::PlannerDataVertex(stateProperty_[v1]));

    // Add tags for the newly added vertices
    data.tagState(stateProperty_[v1],
                  (vertexValidityProperty_[v1] & VALIDITY_TRUE) == 0 ? 0 : 1);
    data.tagState(stateProperty_[v2],
                  (vertexValidityProperty_[v2] & VALIDITY_TRUE) == 0 ? 0 : 1);
  }
}

} // end of namespace motion_planning
