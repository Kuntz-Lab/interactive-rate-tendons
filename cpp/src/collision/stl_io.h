/*
 Copyright (c) 2018, Sebastian Reiter (s.b.reiter@gmail.com)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * This file has been modified by Michael Bentley.  The original version came
 * from
 *
 *   https://github.com/sreiter/stl_reader.git
 *
 * specifically from commit 67a6b78.  The original intention was a single
 * header file only for reading STL mesh files.  I have removed the pieces I am
 * not using and added an ASCII STL writing function as well.
 *
 * - Michael Bentley (April 2020)
 */


/** \file
 * \brief Provides functions to read **stl files** into user provided arrays
 *
 * The central function of this file is `read_stl_file(...)`. It automatically
 * recognizes whether an *ASCII* or a *Binary* file is to be read. It
 * identifies matching corner coordinates of triangles with each other, so that
 * the resulting coordinate array does not contain the same coordinate-triple
 * multiple times.
 *
 * The function operates on template container types. Those containers should
 * have similar interfaces as `std::vector` and operate on `float` or `double`
 * types (`TNumberContainer`) or on `int` or `size_t` types
 * (`TIndexContainer`).
 *
 * ### Usage example (using raw data arrays)
 *
 * \code
 *   std::vector<float> coords, normals;
 *   std::vector<unsigned int> tris, solids;
 *
 *   try {
 *     collision::read_stl_file("geometry.stl", coords, normals, tris, solids);
 *     const size_t numTris = tris.size() / 3;
 *     for(size_t itri = 0; itri < numTris; ++itri) {
 *       std::cout << "coordinates of triangle " << itri << ": ";
 *       for(size_t icorner = 0; icorner < 3; ++icorner) {
 *         float* c = &coords[3 * tris [3 * itri + icorner]];
 *         std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
 *       }
 *       std::cout << std::endl;
 *
 *       float* n = &normals [3 * itri];
 *       std::cout << "normal of triangle " << itri << ": "
 *                 << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
 *     }
 *   } catch (std::exception& e) {
 *     std::cout << e.what() << std::endl;
 *   }
 * \endcode
 */

#ifndef STL_IO_H
#define STL_IO_H

#include <fcl/data_types.h>
#include <fcl/math/vec_3f.h>
#include <fcl/BVH/BVH_model.h>

#include <algorithm>
#include <exception>
#include <fstream>
#include <limits>     // for std::numeric_limits<>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>      // for std::sqrt()

// Throws an std::runtime_error with the given message.
#define STL_READER_THROW(msg) \
  { \
    std::stringstream ss; \
    ss << msg; \
    throw std::runtime_error(ss.str()); \
  }

// Throws an std::runtime_error with the given message, if the given condition
// evaluates to true.
#define STL_READER_COND_THROW(cond, msg) \
  if(cond) { \
    std::stringstream ss; \
    ss << msg; \
    throw std::runtime_error(ss.str()); \
  }


namespace collision {

/// Reads an ASCII or binary stl file into several arrays
/** Reads a stl file and writes its coordinates, normals and
 * triangle-corner-indices to the provided containers. It also fills a
 * container solidRangesOut, which provides the triangle ranges for individual
 * solids.
 *
 * Double vertex entries are removed on the fly, so that triangle corners with
 * equal coordinates are represented by a single coordinate entry in coordsOut.
 *
 *
 * \param filename        [in] The name of the file which shall be read
 *
 * \param coordsOut       [out] Coordinates are written to this container. On
 *                        termination, it has size numVertices * 3. Each triple
 *                        of entries forms a 3d coordinate. The type
 *                        TNumberContainer should have the same interface as
 *                        std::vector<float>.
 *
 * \param normalsOut      [out] Face normals are written to this container. On
 *                        termination, it has size numFaces * 3. Each triple of
 *                        entries forms a 3d normal. The type TNumberContainer
 *                        should have the same interface as std::vector<float>.
 *
 * \param trisOut         [out] Triangle corner indices are written to this
 *                        container.  On termination, it has size numFaces * 3.
 *                        Each triple of entries defines a triangle. The type
 *                        TIndexContainer should have the same interface as
 *                        std::vector<size_t>.  Multiply corner indices from
 *                        trisOut by 3 to obtain the index of the first
 *                        coordinate of that corner in coordsOut.
 *
 * \param solidRangesOut  [out] On termination, it holds the ranges of triangle
 *                        indices for each solid. It has the size numSolids +
 *                        1. Each entry can be interpreted as a end/begin
 *                        triangle index for the previous/next solid. E.g., if
 *                        there are 3 solids, the returned array would look
 *                        like this:
 *                        \code
 *                          {
 *                            sol1Begin,
 *                            sol1End/sol2Begin,
 *                            sol2End/sol3Begin,
 *                            sol3End
 *                          }
 *                        \endcode
 *                        The type TIndexContainer should have the same
 *                        interface as std::vector<size_t>.
 *
 * \returns               true if the file was successfully read into the
 *                        provided container.
 */
template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1,  class TIndexContainer2>
bool read_stl_file(const char* filename,
                   TNumberContainer1& coordsOut,
                   TNumberContainer2& normalsOut,
                   TIndexContainer1& trisOut,
                   TIndexContainer2& solidRangesOut);


/// Reads an ASCII stl file into several arrays
/** \copydetails read_stl_file
 * \sa read_stl_file, read_stl_file_ascii
 */
template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1,  class TIndexContainer2>
bool read_stl_file_ascii(const char* filename,
                         TNumberContainer1& coordsOut,
                         TNumberContainer2& normalsOut,
                         TIndexContainer1& trisOut,
                         TIndexContainer2& solidRangesOut);

/// Reads a binary stl file into several arrays
/** \copydetails read_stl_file
 * \todo    support systems with big endianess
 * \sa         read_stl_file, read_stl_file_binary
 */
template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1,  class TIndexContainer2>
bool read_stl_file_binary(const char* filename,
                          TNumberContainer1& coordsOut,
                          TNumberContainer2& normalsOut,
                          TIndexContainer1& trisOut,
                          TIndexContainer2& solidRangesOut);

/// Determines whether a stl file has ASCII format
/** The underlying mechanism is simply checks whether the provided file starts
 * with the keyword solid. This should work for many stl files, but may fail,
 * of course.
 */
inline bool stl_file_is_in_ascii(const char* filename);


template <class TNumberContainer, class TIndexContainer>
void write_stl_file_binary(const std::string &filename,
                           const TNumberContainer& coords,
                           const TIndexContainer& tris);

inline void write_stl_file_binary(const std::string &filename,
                           const std::vector<fcl::Vec3f> &coords,
                           const std::vector<fcl::Triangle> &tris);

template <typename BV>
void write_stl_file_binary(const std::string &filename,
                           const fcl::BVHModel<BV> &mesh);


template <class TNumberContainer, class TIndexContainer>
void write_stl_file_ascii(const std::string &filename,
                          const TNumberContainer& coords,
                          const TIndexContainer& tris);

inline void write_stl_file_ascii(const std::string &filename,
                          const std::vector<fcl::Vec3f> &coords,
                          const std::vector<fcl::Triangle> &tris);

template <typename BV>
void write_stl_file_ascii(const std::string &filename,
                          const fcl::BVHModel<BV> &mesh);


////////////////////////////////////////////////////////////////////////////////
//    IMPLEMENTATION
////////////////////////////////////////////////////////////////////////////////


namespace stl_io_impl {

  // a coordinate triple with an additional index. The index is required
  // for remove_doubles, so that triangles can be reindexed properly.
  template <typename number_t, typename index_t>
  struct CoordWithIndex {
    number_t data[3];
    index_t index;

    bool operator == (const CoordWithIndex& c) const {
      return (c[0] == data[0]) && (c[1] == data[1]) && (c[2] == data[2]);
    }

    bool operator != (const CoordWithIndex& c) const {
      return (c[0] != data[0]) || (c[1] != data[1]) || (c[2] != data[2]);
    }

    bool operator < (const CoordWithIndex& c) const {
      return     (data[0] < c[0])
              || (data[0] == c[0] && data[1] < c[1])
              || (data[0] == c[0] && data[1] == c[1] && data[2] < c[2]);
    }

    inline number_t& operator [] (const size_t i)       { return data[i]; }
    inline number_t  operator [] (const size_t i) const { return data[i]; }
  };

  // sorts the array coordsWithIndexInOut and copies unique indices to
  // coordsOut.
  // Triangle-corners are re-indexed on the fly and degenerated triangles are
  // removed.
  template <class TNumberContainer, class TIndexContainer>
  void remove_doubles (TNumberContainer& uniqueCoordsOut,
                       TIndexContainer& trisInOut,
                       std::vector <CoordWithIndex<
                         typename TNumberContainer::value_type,
                         typename TIndexContainer::value_type>>
                           &coordsWithIndexInOut)
  {
    using namespace std;

    typedef typename TNumberContainer::value_type    number_t;
    typedef typename TIndexContainer::value_type    index_t;

    sort (coordsWithIndexInOut.begin(), coordsWithIndexInOut.end());

    // first count unique indices
    index_t numUnique = 1;
    for(size_t i = 1; i < coordsWithIndexInOut.size(); ++i) {
      if(coordsWithIndexInOut[i] != coordsWithIndexInOut[i - 1]) {
        ++numUnique;
      }
    }

    uniqueCoordsOut.resize (numUnique * 3);
    vector<index_t> newIndex (coordsWithIndexInOut.size());

    // copy unique coordinates to 'uniqueCoordsOut' and create an index-map
    // 'newIndex', which allows to re-index triangles later on.
    index_t curInd = 0;
    newIndex[0] = 0;
    for(index_t i = 0; i < 3; ++i) {
      uniqueCoordsOut[i] = coordsWithIndexInOut[0][i];
    }

    for(size_t i = 1; i < coordsWithIndexInOut.size(); ++i) {
      const CoordWithIndex <number_t, index_t> c = coordsWithIndexInOut[i];
      if(c != coordsWithIndexInOut[i - 1]) {
        ++curInd;
        for(index_t j = 0; j < 3; ++j) {
          uniqueCoordsOut[curInd * 3 + j] = coordsWithIndexInOut[i][j];
        }
      }
      newIndex[c.index] = static_cast<index_t> (curInd);
    }

    // re-index triangles, so that they refer to 'uniqueCoordsOut'
    // make sure to only add triangles which refer to three different indices
    index_t numUniqueTriInds = 0;
    for(index_t i = 0; i < trisInOut.size(); i+=3) {
      int ni[3];
      for(int j = 0; j < 3; ++j) {
        ni[j] = newIndex[trisInOut[i+j]];
      }

      if((ni[0] != ni[1]) && (ni[0] != ni[2]) && (ni[1] != ni[2])) {
        for(int j = 0; j < 3; ++j) {
          trisInOut[numUniqueTriInds + j] = ni[j];
        }
        numUniqueTriInds += 3;
      }
    }

    if(numUniqueTriInds < trisInOut.size()) {
      trisInOut.resize (numUniqueTriInds);
    }
  }

  inline void ofopen(std::ofstream &out, const std::string &filename) {
    out.exceptions(std::ios::failbit); // turn on exceptions on failure
    out.open(filename);                // open file with exceptions on
    out.exceptions(std::ios::goodbit); // back to default behavior

    // enough digits for exact floats
    out.precision(std::numeric_limits<float>::max_digits10);
  }

  inline std::ostream& print_normal(std::ostream &out,
                                    const fcl::Vec3f &n)
  {
    return out << "  facet normal " << n[0] << " " << n[1] << " " << n[2]
               << "\n";
  }

  inline std::ostream& print_vertex(std::ostream &out,
                                    const fcl::Vec3f &v)
  {
    return out << "      vertex " << v[0] << " " << v[1] << " " << v[2] << "\n";
  }

  inline fcl::Vec3f compute_normal(
      const fcl::Vec3f &a, const fcl::Vec3f &b, const fcl::Vec3f &c)
  {
    auto normal = (b - a).cross(c - b);
    normal /= normal.norm();
    return normal;
  }

  inline std::ostream& print_triangle(std::ostream &out,
                                      const fcl::Vec3f &a,
                                      const fcl::Vec3f &b,
                                      const fcl::Vec3f &c)
  {
    auto normal = compute_normal(a, b, c);
    stl_io_impl::print_normal(out, normal);
    out << "    outer loop\n";
    stl_io_impl::print_vertex(out, a);
    stl_io_impl::print_vertex(out, b);
    stl_io_impl::print_vertex(out, c);
    out << "    endloop\n"
           "  endfacet\n";
    return out;
  }

  inline void print_header_binary(std::FILE* out) {
    const char header[100] =
      " Converted by code written by Michael Be"
      "ntley.                                  ";
    std::fwrite(header, sizeof(header[0]), 80, out);
  }

  inline void print_vec_binary(std::FILE* out, const fcl::Vec3f &v) {
    float val;
    val = static_cast<float>(v[0]);
    std::fwrite(&val, sizeof(float), 1, out);
    val = static_cast<float>(v[1]);
    std::fwrite(&val, sizeof(float), 1, out);
    val = static_cast<float>(v[2]);
    std::fwrite(&val, sizeof(float), 1, out);
  }

  inline void print_triangle_binary(std::FILE* out,
                                    const fcl::Vec3f &a,
                                    const fcl::Vec3f &b,
                                    const fcl::Vec3f &c)
  {
    auto normal = compute_normal(a, b, c);
    print_vec_binary(out, normal);
    print_vec_binary(out, a);
    print_vec_binary(out, b);
    print_vec_binary(out, c);
    const uint16_t attribute_byte_count = 0;
    std::fwrite(&attribute_byte_count, sizeof(attribute_byte_count), 1, out);
  }

  /// RAII structure for closing a file automatically
  struct FileCloser {
    std::FILE* file = nullptr;

    FileCloser(std::FILE* _file) : file(_file) {}
    ~FileCloser() {
      std::fclose(file);
      file = nullptr;
    }
  };
}// end of namespace stl_io_impl


template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1, class TIndexContainer2>
bool read_stl_file(const char* filename,
                   TNumberContainer1 &coordsOut,
                   TNumberContainer2 &normalsOut,
                   TIndexContainer1 &trisOut,
                   TIndexContainer2 &solidRangesOut)
{
  if(stl_file_is_in_ascii(filename)) {
    return read_stl_file_ascii(filename, coordsOut, normalsOut, trisOut,
                               solidRangesOut);
  } else {
    return read_stl_file_binary(filename, coordsOut, normalsOut, trisOut,
                                solidRangesOut);
  }
}


template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1, class TIndexContainer2>
bool read_stl_file_ascii(const char* filename,
                         TNumberContainer1& coordsOut,
                         TNumberContainer2& normalsOut,
                         TIndexContainer1& trisOut,
                         TIndexContainer2& solidRangesOut)
{
  using namespace std;
  using namespace stl_io_impl;

  typedef typename TNumberContainer1::value_type number_t;
  typedef typename TIndexContainer1::value_type  index_t;

  coordsOut.clear();
  normalsOut.clear();
  trisOut.clear();
  solidRangesOut.clear();

  ifstream in(filename);
  STL_READER_COND_THROW(!in, "Couldn't open file " << filename);

  vector<CoordWithIndex <number_t, index_t> > coordsWithIndex;

  string buffer;
  vector<string> tokens;
  int lineCount = 1;
  int maxNumTokens = 0;
  size_t numFaceVrts = 0;

  while(!(in.eof() || in.fail())) {
    // read the line and tokenize.
    // In order to reuse memory in between lines, 'tokens' won't be cleared.
    // Instead we count the number of tokens using 'tokenCount'.
    getline(in, buffer);

    istringstream line(buffer);
    int tokenCount = 0;
    while(!(line.eof() || line.fail())){
      if(tokenCount >= maxNumTokens){
        maxNumTokens = tokenCount + 1;
        tokens.resize(maxNumTokens);
      }
      line >> tokens[tokenCount];
      ++tokenCount;
    }

    if(tokenCount > 0)
    {
      string& tok = tokens[0];
      if(tok.compare("vertex") == 0){
        if(tokenCount < 4){
          STL_READER_THROW("ERROR while reading from " << filename <<
            ": vertex not specified correctly in line " << lineCount);
        }

        // read the position
        CoordWithIndex <number_t, index_t> c;
        for(size_t i = 0; i < 3; ++i) {
          c[i] = static_cast<number_t> (atof(tokens[i+1].c_str()));
        }
        c.index = static_cast<index_t>(coordsWithIndex.size());
        coordsWithIndex.push_back(c);
        ++numFaceVrts;
      } else if(tok.compare("facet") == 0) {
        STL_READER_COND_THROW(tokenCount < 5,
          "ERROR while reading from " << filename <<
          ": triangle not specified correctly in line " << lineCount);

        STL_READER_COND_THROW(tokens[1].compare("normal") != 0,
          "ERROR while reading from " << filename <<
          ": Missing normal specifier in line " << lineCount);

        // read the normal
        for(size_t i = 0; i < 3; ++i) {
          normalsOut.push_back(
            static_cast<number_t>(atof(tokens[i+2].c_str())));
        }

        numFaceVrts = 0;
      } else if(tok.compare("outer") == 0) {
        STL_READER_COND_THROW(
          (tokenCount < 2) || (tokens[1].compare("loop") != 0),
          "ERROR while reading from " << filename <<
          ": expecting outer loop in line " << lineCount);
      } else if(tok.compare("endfacet") == 0) {
        STL_READER_COND_THROW(numFaceVrts != 3,
          "ERROR while reading from " << filename <<
          ": bad number of vertices specified for face in line " << lineCount);

        trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 3));
        trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 2));
        trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 1));
      } else if(tok.compare("solid") == 0) {
        solidRangesOut.push_back(static_cast<index_t> (trisOut.size() / 3));
      }
    }
    lineCount++;
  }

  solidRangesOut.push_back(static_cast<index_t> (trisOut.size() / 3));

  remove_doubles (coordsOut, trisOut, coordsWithIndex);

  return true;
}


template <class TNumberContainer1, class TNumberContainer2,
          class TIndexContainer1, class TIndexContainer2>
bool read_stl_file_binary(const char* filename,
                          TNumberContainer1& coordsOut,
                          TNumberContainer2& normalsOut,
                          TIndexContainer1& trisOut,
                          TIndexContainer2& solidRangesOut)
{
  using namespace std;
  using namespace stl_io_impl;

  typedef typename TNumberContainer1::value_type    number_t;
  typedef typename TIndexContainer1::value_type    index_t;

  coordsOut.clear();
  normalsOut.clear();
  trisOut.clear();
  solidRangesOut.clear();

  ifstream in(filename, ios::binary);
  STL_READER_COND_THROW(!in, "Couldnt open file " << filename);

  char stl_header[80];
  in.read(stl_header, 80);
  STL_READER_COND_THROW(!in,
    "Error while parsing binary stl header in file " << filename);

  unsigned int numTris = 0;
  in.read((char*)&numTris, 4);
  STL_READER_COND_THROW(!in,
    "Couldnt determine number of triangles in binary stl file " << filename);

  vector<CoordWithIndex<number_t, index_t>> coordsWithIndex;

  for(unsigned int tri = 0; tri < numTris; ++tri) {
    float d[12];
    in.read((char*)d, 12 * 4);
    STL_READER_COND_THROW(!in,
      "Error while parsing triangle in binary stl file " << filename);

    for(int i = 0; i < 3; ++i) {
      normalsOut.push_back (d[i]);
    }

    for(size_t ivrt = 1; ivrt < 4; ++ivrt) {
      CoordWithIndex <number_t, index_t> c;
      for(size_t i = 0; i < 3; ++i) {
        c[i] = d[ivrt * 3 + i];
      }
      c.index = static_cast<index_t>(coordsWithIndex.size());
      coordsWithIndex.push_back(c);
    }

    trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 3));
    trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 2));
    trisOut.push_back(static_cast<index_t> (coordsWithIndex.size() - 1));

    char addData[2];
    in.read(addData, 2);
    STL_READER_COND_THROW(!in,
      "Error while parsing additional triangle data in binary stl file "
        << filename);
  }

  solidRangesOut.push_back(0);
  solidRangesOut.push_back(static_cast<index_t> (trisOut.size() / 3));

  remove_doubles (coordsOut, trisOut, coordsWithIndex);

  return true;
}


inline bool stl_file_is_in_ascii(const char* filename)
{
  using namespace std;
  ifstream in(filename);
  STL_READER_COND_THROW(!in, "Couldn't open file " << filename);

  string firstWord;
  in >> firstWord;
  transform(firstWord.begin(), firstWord.end(), firstWord.begin(),
            ::tolower);

  return firstWord.compare("solid") == 0;
}


template <class TNumberContainer, class TIndexContainer>
void write_stl_file_binary(const std::string &filename,
                           const TNumberContainer& coords,
                           const TIndexContainer& tris) {
  // Binary StL format:
  //   taken from http://www.fabbers.com/tech/STL_Format
  //
  // | Bytes | Data type            | Description                        |
  // |-------|----------------------|------------------------------------|
  // |  80   | ASCII                | Header.  No data significance.     |
  // |   4   | unsigned long int    | Number of facets in file           |
  //   -- repeat for each triangle --
  // |   4   | float                | i for normal                       |
  // |   4   | float                | j                                  |
  // |   4   | float                | k                                  |
  // |   4   | float                | x for vertex 1                     |
  // |   4   | float                | y                                  |
  // |   4   | float                | z                                  |
  // |   4   | float                | x for vertex 2                     |
  // |   4   | float                | y                                  |
  // |   4   | float                | z                                  |
  // |   4   | float                | x for vertex 3                     |
  // |   4   | float                | y                                  |
  // |   4   | float                | z                                  |
  // |   2   | unsigned int         | Attribute byte count (set to zero) |
  //
  // Restrictions:
  // 1. Normal must be of unit length
  // 2. Normal must adhere to the right-hand rule with vertex ordering
  //    (facet orientation)
  // 3. All numbers are in single precision
  // 4. A vertex must not be on the edge of another triangle
  //    (vertex-to-vertex rule)
  //
  // Sorting the triangles in ascending z-value order is recommended

  stl_io_impl::FileCloser out(fopen(filename.c_str(), "wb"));
  stl_io_impl::print_header_binary(out.file);

  // write the number of triangles
  uint32_t n_triangles = tris.size() / 3;
  std::fwrite(&n_triangles, sizeof(n_triangles), 1, out.file);

  // write each triangle
  for (uint32_t itri = 0; itri < n_triangles; itri++) {
    auto a = &coords[3 * tris[3 * itri + 0]]; // corner 0
    auto b = &coords[3 * tris[3 * itri + 1]]; // corner 1
    auto c = &coords[3 * tris[3 * itri + 2]]; // corner 2
    stl_io_impl::print_triangle_binary(out.file,
                                       fcl::Vec3f(a[0], a[1], a[2]),
                                       fcl::Vec3f(b[0], b[1], b[2]),
                                       fcl::Vec3f(c[0], c[1], c[2]));
  }
}

inline void write_stl_file_binary(const std::string &filename,
                           const std::vector<fcl::Vec3f> &coords,
                           const std::vector<fcl::Triangle> &tris)
{
  using stl_io_impl::print_triangle_binary;

  stl_io_impl::FileCloser out(fopen(filename.c_str(), "wb"));
  stl_io_impl::print_header_binary(out.file);

  // write the number of triangles
  uint32_t n_triangles = static_cast<uint32_t>(tris.size());
  std::fwrite(&n_triangles, sizeof(n_triangles), 1, out.file);

  // write each triangle
  for (auto &t : tris) {
    print_triangle_binary(out.file, coords[t[0]], coords[t[1]], coords[t[2]]);
  }
}

template <typename BV>
void write_stl_file_binary(const std::string &filename,
                           const fcl::BVHModel<BV> &mesh)
{
  using stl_io_impl::print_triangle_binary;

  stl_io_impl::FileCloser out(fopen(filename.c_str(), "wb"));
  stl_io_impl::print_header_binary(out.file);

  // write the number of triangles
  uint32_t n_triangles = static_cast<uint32_t>(mesh.num_tris);
  std::fwrite(&n_triangles, sizeof(n_triangles), 1, out.file);

  // write each triangle
  auto &v = mesh.vertices;
  for (int i = 0; i < mesh.num_tris; i++) {
    auto &t = mesh.tri_indices[i];
    print_triangle_binary(out.file, v[t[0]], v[t[1]], v[t[2]]);
  }
}

template <class TNumberContainer, class TIndexContainer>
void write_stl_file_ascii(const std::string &filename,
                          const TNumberContainer& coords,
                          const TIndexContainer& tris)
{
  // ASCII StL format:
  //   taken from http://www.fabbers.com/tech/STL_Format
  //
  // solid <name>
  //   facet normal <n_x> <n_y> <n_z>
  //     outer loop
  //       vertex <v1_x> <v1_y> <v1_z>
  //       vertex <v2_x> <v2_y> <v2_z>
  //       vertex <v3_x> <v3_y> <v3_z>
  //     endloop
  //   endfacet
  //   ...         // repeat as many facets as you have
  // endsolid <name>
  //
  // Restrictions:
  // 1. Normal must be of unit length
  // 2. Normal must adhere to the right-hand rule with vertex ordering
  //    (facet orientation)
  // 3. All numbers are in single precision
  // 4. A vertex must not be on the edge of another triangle
  //    (vertex-to-vertex rule)
  //
  // Sorting the triangles in ascending z-value order is recommended

  using stl_io_impl::print_triangle;

  std::ofstream out;
  stl_io_impl::ofopen(out, filename);

  out << "solid ASCII\n";

  // TODO: sort the triangles by their minimum z-value (or max? or avg?)
  // TODO: check the vertex-to-vertex rule

  const size_t numTris = tris.size() / 3;
  for (size_t itri = 0; itri < numTris; itri++) {
    auto a = &coords[3 * tris[3 * itri + 0]]; // corner 0
    auto b = &coords[3 * tris[3 * itri + 1]]; // corner 1
    auto c = &coords[3 * tris[3 * itri + 2]]; // corner 2
    print_triangle(out,
                   fcl::Vec3f(a[0], a[1], a[2]),
                   fcl::Vec3f(b[0], b[1], b[2]),
                   fcl::Vec3f(c[0], c[1], c[2]));
  }

  out << "endsolid ASCII\n";
}

inline void write_stl_file_ascii(const std::string &filename,
                                 const std::vector<fcl::Vec3f> &coords,
                                 const std::vector<fcl::Triangle> &tris)
{
  using stl_io_impl::print_triangle;

  std::ofstream out;
  stl_io_impl::ofopen(out, filename);

  out << "solid ASCII\n";

  // TODO: sort the triangles somehow

  for (auto &t : tris) {
    print_triangle(out, coords[t[0]], coords[t[1]], coords[t[2]]);
  }

  out << "endsolid ASCII\n";
}

template <typename BV>
void write_stl_file_ascii(const std::string &filename,
                          const fcl::BVHModel<BV> &mesh)
{
  using stl_io_impl::print_triangle;

  std::ofstream out;
  stl_io_impl::ofopen(out, filename);

  out << "solid ASCII\n";

  // TODO: sort the triangles somehow

  auto &v = mesh.vertices;
  for (int i = 0; i < mesh.num_tris; i++) {
    auto &t = mesh.tri_indices[i];
    print_triangle(out, v[t[0]], v[t[1]], v[t[2]]);
  }

  out << "endsolid ASCII\n";
}


} // end of namespace collision

#endif // STL_IO_H
