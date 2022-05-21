#include "submodule_collision.h"

#include <collision/collision.h>
#include <collision/Capsule.h>
#include <collision/CapsuleSequence.h>
#include <collision/Mesh.h>
#include <collision/OctomapWrap.h>
#include <collision/Sphere.h>
#include <collision/VoxelOctree.h>
#include <cpptoml/toml_conversions.h>
#include <util/json_io.h>
#include <util/macros.h>

#include <Eigen/Core>

#include <pybind11/eigen.h>    // auto convert between python and eigen types
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>      // auto convert between python and STL types

#include <sstream>

namespace py = pybind11;

// abbreviations for collision types
using Pt     = collision::Point;
using Sph    = collision::Sphere;
using Cap    = collision::Capsule;
using CapSeq = collision::CapsuleSequence;
using Mesh   = collision::Mesh;
using Vert   = collision::Mesh::Vertex;
using Tri    = collision::Mesh::Triangle;
using VoxOct = collision::VoxelOctree;
using OctMap = collision::OctomapWrap;

using collision::collides;
using collision::collides_self;

// other useful shorthands
using stdstr = std::string;
template<typename T> using stdvec = std::vector<T>;

namespace {

void def_class_Point(py::module &m) {
  UNUSED_VAR(m); // do nothing
}

void def_class_Sphere(py::module &m) {
  py::class_<Sph>(m, "Sphere")
    .def(py::init<const Pt&, double>(),
        py::arg("c") = Pt{0.0, 0.0, 0.0},
        py::arg("r") = 0.0)

    // public attributes
    .def_readwrite("c", &Sph::c, "center")
    .def_readwrite("r", &Sph::r, "radius")

    // methods
    .def("to_toml", [](const Sph &s, const stdstr &fname) {
          cpptoml::to_file(s, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const stdstr &fname) {
          return cpptoml::from_file<Sph>(fname);
        }, py::arg("filepath"),
        "load a Sphere object from the given toml file")

    // python-specific added methods
    .def("__eq__", [](const Sph &a, const Sph &b) {
          return a == b;
        })
    .def("__repr__", [](const Sph &s) {
          using collision::operator<<;
          std::ostringstream builder;
          builder << s;
          return builder.str();
        })
    .def("__str__", [](const Sph &s) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, s.to_toml());
          return builder.str();
        })
    ;
}

void def_class_Capsule(py::module &m) {
  py::class_<Cap>(m, "Capsule",
      "Capsule collision object (cylinder with hemisphere ends)\n"
      "contains all points a radius away from a line segment")
    .def(py::init<const Pt&, const Pt&, double>(),
        py::arg("a") = Pt{0.0, 0.0, 0.0},
        py::arg("b") = Pt{0.0, 0.0, 0.0},
        py::arg("r") = 0.0)

    // public attributes
    .def_readwrite("a", &Cap::a, "first capsule endpoint")
    .def_readwrite("b", &Cap::b, "second capsule endpoint")
    .def_readwrite("r", &Cap::r, "radius")

    // methods
    .def("interpolate", &Cap::interpolate,
        py::arg("t"),
        "interpolate between a and b (0 maps to a, 1 maps to b)")
    .def("closest_t", &Cap::closest_t,
        py::arg("point"),
        "return the interpolation value t from the interval [a, b] that is\n"
        "closest to the given point")
    .def("to_toml", [](const Cap &c, const stdstr &fname) {
          cpptoml::to_file(c, fname);
        }, py::arg("filepath"), "save this object to a toml file")

    // static methods
    .def_static("from_toml", [](const stdstr &fname) {
          return cpptoml::from_file<Cap>(fname);
        }, py::arg("filepath"),
        "load a Capsule object from a given toml file")

    // python-specific added methods
    .def("__eq__",
        [](const Cap &a, const Cap &b) {
          return a == b;
        })
    .def("__repr__", [](const Cap &c) {
          using collision::operator<<;
          std::ostringstream builder;
          builder << c;
          return builder.str();
        })
    .def("__str__", [](const Cap &c) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, c.to_toml());
          return builder.str();
        })
    ;
}

void def_class_CapsuleSequence(py::module &m) {
  py::class_<CapSeq>(m, "CapsuleSequence",
      "CapsuleSequence collision object, which is a sequence of Capsules\n"
      "contains all points a radius away from a piecewise linear path")
    .def(py::init<const stdvec<Pt>&, double>(),
        py::arg("points") = stdvec<Pt>{},
        py::arg("r") = 0.0)

    // public attributes
    .def_readwrite("points", &CapSeq::points,
        "points describing the piecewise linear path of the capsule sequence")
    .def_readwrite("r", &CapSeq::r, "radius")

    // methods
    .def("size", &CapSeq::size,
        "size (i.e., number of capsules)")
    .def("at", &CapSeq::at, py::arg("i"),
        "get the i'th capsule (with bounds checkin - throws)")
    .def("interpolate", &CapSeq::interpolate,
        py::arg("i"),
        "interpolate along the centerline\n"
        "interpolation is from 0.0 <= t <= len(self)\n"
        "with three points, you have two capsules, and 0.0 <= t <= 2.0\n"
        "\n"
        "Note, you can extrapolate using this function by giving numbers\n"
        "outside of the range.  If you give a negative number, it will\n"
        "extrapolate from the first capsule.  If you give a number larger\n"
        "than len(self), then the last capsule is used.")
    .def("closest_t", &CapSeq::closest_t,
        py::arg("point"),
        "Gives the closest t value to the point\n"
        "\n"
        "Note: this is not efficient as it will check each capsule for\n"
        "distance and return the smallest one.  If you just want to know if\n"
        "it collides, use collides() function instead.\n"
        "\n"
        "Note: does not extend to extrapolation range, will keep within the\n"
        "interpolation range.")
    .def("to_toml",
        [](const CapSeq &c, const stdstr &fname) {
          cpptoml::to_file(c, fname);
        }, py::arg("filepath"), "save this object to a toml file")
    .def("to_mesh", &CapSeq::to_mesh,
        py::arg("n_circle_pts") = 16,
        py::arg("add_spherical_caps") = true,
        "Convert this CapsuleSequence to a mesh object.\n"
        "\n"
        "@param n_circle_pts (int): discretization count of circles\n"
        "@param add_spherical_caps (bool): True means to put spherical caps\n"
        "    but False makes the ends like a cylinder (i.e., a flat circle)")

    // static methods
    .def_static("from_toml", [](const stdstr &fname) {
          return cpptoml::from_file<CapSeq>(fname);
        }, py::arg("filepath"),
        "load a CapsuleSequence object from a given toml file")

    // python-specific added methods
    .def("__len__", &CapSeq::size,
        "length (i.e., number of capsules)")
    .def("__getitem__", &CapSeq::operator[],
        py::arg("i"),
        "get the i'th capsule (no bounds checking)")
    .def("__eq__", &CapSeq::operator==)
    .def("__repr__", [](const CapSeq &c) {
          using collision::operator<<;
          std::ostringstream builder;
          builder << c;
          return builder.str();
        })
    .def("__str__", [](const CapSeq &c) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, c.to_toml());
          return builder.str();
        })
    ;
}

void def_class_Mesh(py::module &m) {
  // Triangle class
  py::class_<Tri>(m, "Triangle",
      "Three vertex indices that constitute a triangle in a mesh")
    .def(py::init<size_t, size_t, size_t>(),
        py::arg("p1") = 0, py::arg("p2") = 0,
        py::arg("p3") = 0,
        "Construct from triangle indices")

    // methods
    .def("set", &Tri::set,
        py::arg("p1"), py::arg("p2"), py::arg("p3"),
        "set the vertex indices of the triangle")

    // python-specific added methods
    .def("__getitem__",
        py::overload_cast<int>(
          &Tri::operator[], py::const_),
        py::arg("i"),
        "get vertex index at index i")
    .def("__setitem__",
        [](Tri &t, int key, size_t value) {
          t[key] = value;
        }, py::arg("i"), py::arg("value"))
    .def("__len__", [](const Tri &t) {
          UNUSED_VAR(t);
          return 3;
        })
    .def("__repr__", [](const Tri &t) {
          std::ostringstream builder;
          builder << "Triangle(" << t[0] << ", " << t[1] << ", " << t[2] << ")";
          return builder.str();
        })
    ;

  // Vertex Class
  // Note: this doesn't have all implemented methods, but this is enough
  py::class_<Vert>(m, "Vertex", "Point in space")
    .def(py::init<double, double, double>(),
        py::arg("x") = 0.0,
        py::arg("y") = 0.0,
        py::arg("z") = 0.0)

    // methods
    .def("set", [](Vert &v, double x, double y, double z) {
          v.setValue(x, y, z);
        },
        py::arg("x"), py::arg("y"), py::arg("z"),
        "set all values of the vertex")

    // python-specific added methods
    .def("__getitem__",
        py::overload_cast<size_t>(
          &Vert::operator[], py::const_),
        py::arg("i"),
        "get this dimensional value")
    .def("__setitem__",
        [](Vert &v, size_t key, size_t value) {
          v[key] = value;
        }, py::arg("i"), py::arg("value"))
    .def("__len__", [](const Vert &v) {
          UNUSED_VAR(v);
          return 3;
        })
    .def("__repr__", [](const Vert &v) {
          std::ostringstream builder;
          builder << "Vertex(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
          return builder.str();
        })
    .def("__eq__", &Vert::operator==)
    ;

  // Finally, the Mesh class
  py::class_<Mesh>(m, "Mesh")
    .def(py::init<const stdvec<Vert>&,
                  const stdvec<Tri>&,
                  const stdstr&>(),
        py::arg("vertices") = stdvec<Vert>{},
        py::arg("triangles") = stdvec<Tri>{},
        py::arg("filename") = stdstr())

    // public attributes
    .def_readwrite("vertices", &Mesh::vertices, "vertices")
    .def_readwrite("triangles", &Mesh::triangles, "triangles")
    .def_readonly("filename", &Mesh::filename,
        "cache of where the mesh is stored on disk - is set if loaded from\n"
        "a file or if saved to a file")

    // methods
    .def("empty", &Mesh::empty,
        "empty means no vertices or triangles")
    .def("to_toml", [](const Mesh &m, const stdstr &fname) {
          cpptoml::to_file(m, fname);
        }, py::arg("filename"), "save this object to a toml file")
    .def("to_stl",
        [](const Mesh &m, const stdstr &fname,
           bool binary = true)
        {
          m.to_stl(fname, binary);
        }, py::arg("filename"), py::arg("binary") = true,
        "save this object to an STL file (binary STL or ASCII STL)")
    .def("equal", &Mesh::equal, py::arg("other"),
        "checks equality against vertices and triangles, not filename")

    // static methods
    .def_static("from_toml", [](const stdstr &fname) {
          return cpptoml::from_file<Mesh>(fname);
        }, py::arg("filepath"),
        "load a Mesh object from the given toml file")
    .def_static("from_stl", [](const stdstr &fname) {
          return Mesh::from_stl(fname);
        }, py::arg("filepath"),
        "load a Mesh object from the given STL file\n"
        "supports both binary and ASCII STL files")

    // python-specific added methods
    .def("__bool__", [](const Mesh &m) { return !m.empty(); },
        "a nonempty mesh evaluates to True")
    .def("__eq__", &Mesh::operator==,
        "checks equality for all fields including filename")
    .def("__repr__", [](const Mesh &m) {
          using collision::operator<<;
          std::ostringstream builder;
          builder << m;
          return builder.str();
        })
    .def("__str__", [](const Mesh &m) {
          std::ostringstream builder;
          cpptoml::to_stream(builder, m.to_toml());
          return builder.str();
        })
    ;
}

void def_class_VoxelOctree(py::module &m) {

  py::class_<VoxOct>(m, "VoxelOctree",
      "Occupancy octree for voxelization of a collision space.\n"
      "\n"
      "This is a sparse representation of a voxelization.  The space is\n"
      "separated into individual voxels which can be either empty or\n"
      "occupied.  This is condensed into blocks of 4x4x4.  Each block is\n"
      "represented by a single 64-bit unsigned integer, one bit per voxel to\n"
      "indicate occupied or not.\n"
      "\n"
      "Only the blocks that have occupied voxel cells are stored.  Those\n"
      "blocks that are occupied are stored into a tree structure.  At each\n"
      "level, the space is subdivided in each of the three dimensions,\n"
      "making eight subdivisions.  That is where the name Octree comes from.\n"
      "For collision checking against a particular point, we recurse down\n"
      "the octant child until we either get to a null pointer (meaning that\n"
      "whole subtree is empty space) or a leaf node containing a single\n"
      "64-bit unsigned integer.\n"
      "\n"
      "You can specify the limits of the space.  The bottom-left of each\n"
      "voxel (i.e., toward the origin) is the coordinate of that voxel.\n"
      "This is different than many other voxel implementations where the\n"
      "center of the voxel is the voxel coordinate.  However, having the\n"
      "center be the voxel coordinate causes geometric changes when scaling\n"
      "up or down to a different voxelization resultion.  No such problem\n"
      "occurs when using the bottom-left corner as the voxel coordinate.\n"
      "Instead the center of a voxel at (ix, iy, iz) is\n"
      "\n"
      "  voxel_center = (\n"
      "      voxels.xlim[0] + (ix + 0.5) * voxels.dx,\n"
      "      voxels.ylim[0] + (iy + 0.5) * voxels.dy,\n"
      "      voxels.zlim[0] + (iz + 0.5) * voxels.dz,\n"
      "      )\n"
      "\n"
      "Note: discretizations need to match for all dimensions and it needs\n"
      "to be a power of 2 bigger than 4.  We may be able to remove this\n"
      "limitation later, but for now, it seems to work well.\n")

    .def(py::init<size_t>(), py::arg("Ndim") = 4,
        "create a VoxelOctree with the given discretization in all dimensions")

    //
    // properties
    //
    .def_property_readonly("Nx", &VoxOct::Nx,
        "voxel discretization amount in the x-direction")
    .def_property_readonly("Ny", &VoxOct::Ny,
        "voxel discretization amount in the y-direction")
    .def_property_readonly("Nz", &VoxOct::Nz,
        "voxel discretization amount in the z-direction")
    .def_property_readonly("N", &VoxOct::N,
        "total number of voxels, both empty and nonempty")

    .def_property_readonly("Nbx", &VoxOct::Nbx,
        "block discretization amount in the x-direction")
    .def_property_readonly("Nby", &VoxOct::Nby,
        "block discretization amount in the y-direction")
    .def_property_readonly("Nbz", &VoxOct::Nbz,
        "block discretization amount in the z-direction")
    .def_property_readonly("Nb", &VoxOct::Nb,
        "total number of blocks, both empty and nonempty")

    .def_property("xlim", &VoxOct::xlim,
        py::overload_cast<const std::pair<double, double>&>(
          &VoxOct::set_xlim),
        "space dimensions covered in the x-direction")
    .def_property("ylim", &VoxOct::ylim,
        py::overload_cast<const std::pair<double, double>&>(
          &VoxOct::set_ylim),
        "space dimensions covered in the y-direction")
    .def_property("zlim", &VoxOct::zlim,
        py::overload_cast<const std::pair<double, double>&>(
          &VoxOct::set_zlim),
        "space dimensions covered in the z-direction")
    .def_property_readonly("lower_left", &VoxOct::lower_left,
        "lower-left corner of voxel space")
    .def_property_readonly("upper_right", &VoxOct::upper_right,
        "upper-right corner of voxel space")

    .def_property_readonly("dx", &VoxOct::dx,
        "size of voxels in the x-direction")
    .def_property_readonly("dy", &VoxOct::dy,
        "size of voxels in the y-direction")
    .def_property_readonly("dz", &VoxOct::dz,
        "size of voxels in the z-direction")
    .def_property_readonly("dbx", &VoxOct::dbx,
        "size of blocks in the x-direction")
    .def_property_readonly("dby", &VoxOct::dby,
        "size of blocks in the y-direction")
    .def_property_readonly("dbz", &VoxOct::dbz,
        "size of blocks in the z-direction")

    //
    // methods
    //
    .def("copy", [](const VoxOct &orig) {
          return VoxOct(orig);
        }, "make a deep copy of the current VoxelOctree")
    .def("empty_copy", [](const VoxOct &orig) {
          return orig.empty_copy();
        }, "make an empty copy that retains the limits")
    .def("copy_limits", &VoxOct::copy_limits, py::arg("other"),
        "copy bounding limits from another VoxelOctree")

    .def("nblocks", &VoxOct::nblocks, "number of occupied blocks")
    .def("ncells", &VoxOct::ncells, "number of occupied voxels")

    .def("is_in_domain", &VoxOct::is_in_domain,
        py::arg("x"), py::arg("y"), py::arg("z"),
        "True if x, y, and z are within limits")
    .def("is_in_domain", [](const VoxOct &v, const Pt &p) {
          return v.is_in_domain(p[0], p[1], p[2]);
        }, py::arg("point"), "True if point is within all limits")

    .def("block_center", &VoxOct::block_center,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        "return the workspace point of this block's center")
    .def("voxel_center", &VoxOct::voxel_center,
        py::arg("ix"), py::arg("iy"), py::arg("iz"),
        "return the workspace point of this voxel's center")

    .def("block", &VoxOct::block,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        "return the raw block value at this index (no bounds checking)")
    .def("set_block", &VoxOct::set_block,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        py::arg("values"),
        "set the raw block value at this index (no bounds checking)")
    .def("union_block", &VoxOct::union_block,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        py::arg("value"),
        "union the value against the raw block at this index (no bounds\n"
        "checking)")
    .def("intersect_block", &VoxOct::intersect_block,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        py::arg("value"),
        "intersect the value against the raw block at this index (no bounds\n"
        "checking)")
    .def("subtract_block", &VoxOct::subtract_block,
        py::arg("bx"), py::arg("by"), py::arg("bz"),
        py::arg("value"),
        "set subtract the value against the raw block at this index (no\n"
        "bounds checking)")

    .def("cell", &VoxOct::cell,
        py::arg("ix"), py::arg("iy"), py::arg("iz"),
        "return True if this voxel is occupied (no bounds checking)")
    .def("set_cell", &VoxOct::set_cell,
        py::arg("ix"), py::arg("iy"), py::arg("iz"),
        py::arg("value") = true,
        "sets the voxel's value.  Returns True if the cell's value changed")

    .def("find_block",
        py::overload_cast<const Pt &>(
          &VoxOct::find_block, py::const_),
        py::arg("point"),
        "returns the raw block value that contains this point coordinate")
    .def("nearest_block_idx",
        py::overload_cast<const Pt&>(
          &VoxOct::nearest_block_idx, py::const_),
        py::arg("point"),
        "returns a tuple of block indices storing this point coordinate\n"
        "truncates to the limiting bounds.")
    .def("find_block_idx",
        py::overload_cast<const Pt&>(
          &VoxOct::find_block_idx, py::const_),
        py::arg("point"),
        "returns a tuple of block indices storing this point coordinate\n"
        "checks bounds and throws if out of bounds")
    .def("find_block_idx",
        py::overload_cast<const Pt&>(
          &VoxOct::find_block_idx, py::const_),
        py::arg("point"),
        "returns a tuple of block indices storing this point coordinate\n"
        "checks bounds and throws if out of bounds")

    .def("nearest_cell",
        py::overload_cast<const Pt&>(
          &VoxOct::nearest_cell, py::const_),
        py::arg("point"),
        "returns a tuple of voxel indices storing this point coordinate\n"
        "truncates to the limiting bounds.")
    .def("find_cell",
        py::overload_cast<const Pt&>(
          &VoxOct::find_cell, py::const_),
        py::arg("point"),
        "returns a tuple of voxel indices storing this point coordinate\n"
        "checks bounds and throws if out of bounds")

    .def("add_point",
        py::overload_cast<const Pt&>(&VoxOct::add_point),
        py::arg("point"),
        "mark the voxel containing this point as occupied")
    .def("add_line", &VoxOct::add_line, py::arg("a"), py::arg("b"),
        "mark the voxels touching the line segment from a to b")
    .def("add_piecewise_line", &VoxOct::add_piecewise_line, py::arg("line"),
        "like add_line() but for a sequence of connected line segments")
    .def("add_sphere", &VoxOct::add_sphere, py::arg("sphere"),
        "mark all voxels whos centers go through the sphere are occupied")
    .def("add_capsule", &VoxOct::add_capsule, py::arg("capsule"),
        "mark all voxels whos centers go through the capsule are occupied")
    .def("add_voxels", &VoxOct::add_voxels, py::arg("other"),
        "add the other marked voxels to this object, assumes same bounds")

    .def("remove_interior_6neighbor", &VoxOct::remove_interior_6neighbor,
        "remove interior points according to a 6-neighbor criteria")
    .def("remove_interior_27neighbor", &VoxOct::remove_interior_27neighbor,
        "remove interior points according to a 27-neighbor criteria,\n"
        "essentially keeping diagonals.")
    .def("remove_interior", &VoxOct::remove_interior,
        py::arg("keep_diagonal") = true, "remove interior points.")

    .def("dilate_6neighbor", &VoxOct::dilate_6neighbor, py::arg("num") = 1,
        "dilate num voxels with 6 neighbors")
    .def("dilate_27neighbor", &VoxOct::dilate_27neighbor,
        "dilate num voxels with 27 neighbors")
    .def("dilate", &VoxOct::dilate,
        py::arg("num") = 1, py::arg("use_diagonal") = false,
        "dilate num voxels using diagonal (27 neighbors) or not (6 neighbors)")
    .def("dilate_sphere", &VoxOct::dilate_sphere, py::arg("r"),
        "dilate to union(Sphere(r, p) for p in voxels) (only approximately)")

    .def("erode_6neighbor", &VoxOct::erode_6neighbor,
        "erode one voxel with 6 neighbors")
    .def("erode_27neighbor", &VoxOct::erode_27neighbor,
        "erode one voxel with 27 neighbors")
    .def("erode", &VoxOct::erode, py::arg("use_diagonal") = false,
        "erode one voxel using diagonal (27 neighbors) or not (6 neighbors)")
    .def("erode_sphere", &VoxOct::erode_sphere, py::arg("r"),
        "erode to {p : Sphere(r, p) in voxels}")

    .def("collides",
        py::overload_cast<const Pt &>(&VoxOct::collides, py::const_),
        py::arg("point"),
        "return True if the point is in an occupied voxel")
    .def("collides",
        py::overload_cast<const VoxOct&>(&VoxOct::collides, py::const_),
        py::arg("other"),
        "return True if at least one voxel is in common between these two\n"
        "VoxelOctrees.  It is assumed they span the same limits - just\n"
        "indices are checked.")

    .def("to_mesh", &VoxOct::to_mesh,
        "convert voxel object to a mesh\n"
        "\n"
        "this converts each voxel into triangles that represent the box shape\n"
        "of the voxel.  It is not the typical idea of what it would mean to\n"
        "convert a voxelized object to a surface mesh.")
    .def("to_file", &VoxOct::to_file,
        py::arg("filepath"),
        "save this voxel object to a file, based on the file extension.\n"
        "Supported file extensions are nrrd, toml, toml.gz, json, bson,\n"
        "cbor, msgpack, and ubjson.")

    //
    // static methods
    //
    .def_static("to_supported_size", &VoxOct::to_supported_size,
        py::arg("Ndim"),
        "upscale the argument to the nearest multiple of two.  if Ndim is\n"
        "larger than the largest supported size, then an exception is thrown.")
    .def_static("largest_supported_size", &VoxOct::largest_supported_size,
        "largest discretization supported in this implementation")
    .def_static("from_file", &VoxOct::from_file, py::arg("filepath"),
        "load a voxel object from file by extension")

    //
    // python-specific added methods
    //
    .def("__eq__", &VoxOct::operator==)
    ;
}

void def_class_OctomapWrap(py::module &m) {

  py::class_<OctMap>(m, "OctomapWrap",
      "Wrapper around Octomap's OcTree class")

    .def(py::init<double>(), py::arg("resolution"),
        "Make empty one from smallest voxel dimension")
    .def(py::init<const VoxOct&>(), py::arg("vtree"),
        "Convert from VoxelOctree to OctomapWrap")

    //
    // properties
    //
    .def_property_readonly("Nx", &OctMap::Nx,
        "voxel discretization amount in the x-direction")
    .def_property_readonly("Ny", &OctMap::Ny,
        "voxel discretization amount in the y-direction")
    .def_property_readonly("Nz", &OctMap::Nz,
        "voxel discretization amount in the z-direction")

    .def_property("xlim", &OctMap::xlim,
        [](OctMap &m, const std::pair<double, double> &lim) {
          m.set_xlim(lim.first, lim.second);
        },
        "space dimensions covered in the x-direction")
    .def_property("ylim", &OctMap::ylim,
        [](OctMap &m, const std::pair<double, double> &lim) {
          m.set_ylim(lim.first, lim.second);
        },
        "space dimensions covered in the y-direction")
    .def_property("zlim", &OctMap::zlim,
        [](OctMap &m, const std::pair<double, double> &lim) {
          m.set_zlim(lim.first, lim.second);
        },
        "space dimensions covered in the z-direction")

    .def_property_readonly("dx", &OctMap::dx,
        "size of voxels in the x-direction")
    .def_property_readonly("dy", &OctMap::dy,
        "size of voxels in the y-direction")
    .def_property_readonly("dz", &OctMap::dz,
        "size of voxels in the z-direction")

    //
    // methods
    //
    .def("copy", [](const OctMap &orig) {
          return OctMap(orig);
        }, "make a deep copy of the current VoxelOctree")

    .def("memoryUsage", &OctMap::memoryUsage, "memory usage")
    .def("nblocks", &OctMap::nblocks, "number of represented voxels")
    .def("ncells", &OctMap::nblocks, "number of represented voxels")

    .def("add_point", &OctMap::add_point,
        py::arg("x"), py::arg("y"), py::arg("z"),
        py::arg("occupied") = true,
        py::arg("lazy") = true,
        "mark the occupancy of the voxel containing this point\n"
        "\n"
        "Parameters:\n"
        "    x, y, z:   float\n"
        "        Point in the voxel space to set\n"
        "    occupied:  bool = True\n"
        "        Value to give this voxel (if within the domain)\n"
        "    lazy:      bool = False\n"
        "        True means to not update internal cache.  This speeds up\n"
        "        updates if many are performed, but you should call update()\n"
        "        afterward all additions if you use laziness here.")
    .def("update", &OctMap::update,
        "Update internal cache.  Use if you called one or more methods that\n"
        "update voxel values and you used laziness.")

    .def("collides",
        py::overload_cast<const OctMap&>(&OctMap::collides, py::const_),
        py::arg("other"),
        "return True if at least one voxel is in common between these two\n"
        "VoxelOctrees.")

    ;
}

template <typename A, typename B>
using collides_cast = bool (*)(const A&, const B&);

void def_function_collides(py::module &m) {
  // Point
  m.def("collides", collides_cast<Pt    , Pt    >(&collides));

  // Sphere
  m.def("collides", collides_cast<Sph   , Pt    >(&collides));
  m.def("collides", collides_cast<Pt    , Sph   >(&collides));
  m.def("collides", collides_cast<Sph   , Sph   >(&collides));

  // Capsule
  m.def("collides", collides_cast<Cap   , Pt    >(&collides));
  m.def("collides", collides_cast<Pt    , Cap   >(&collides));
  m.def("collides", collides_cast<Cap   , Sph   >(&collides));
  m.def("collides", collides_cast<Sph   , Cap   >(&collides));
  m.def("collides", collides_cast<Cap   , Cap   >(&collides));

  // CapsuleSequence
  m.def("collides", collides_cast<CapSeq, Pt    >(&collides));
  m.def("collides", collides_cast<Pt    , CapSeq>(&collides));
  m.def("collides", collides_cast<CapSeq, Sph   >(&collides));
  m.def("collides", collides_cast<Sph   , CapSeq>(&collides));
  m.def("collides", collides_cast<CapSeq, Cap   >(&collides));
  m.def("collides", collides_cast<Cap   , CapSeq>(&collides));
  m.def("collides", collides_cast<CapSeq, CapSeq>(&collides));

  // Mesh
  m.def("collides", collides_cast<Sph   , Mesh  >(&collides));
  m.def("collides", collides_cast<Mesh  , Sph   >(&collides));
  m.def("collides", collides_cast<Cap   , Mesh  >(&collides));
  m.def("collides", collides_cast<Mesh  , Cap   >(&collides));
  m.def("collides", collides_cast<Mesh  , Mesh  >(&collides));

  // VoxelOctree
  m.def("collides", collides_cast<VoxOct, VoxOct>(&collides));
}

template <typename T>
using collides_self_cast = bool (*)(const T&);

void def_function_collides_self(py::module &m) {
  // CapsuleSequence
  m.def("collides_self", collides_self_cast<CapSeq>(&collides_self));
}

} // end of unnamed namespace

py::module def_submodule_collision(py::module &m) {
  auto submodule = m.def_submodule("collision", "A simple collision library");
  def_class_Point(submodule);
  def_class_Sphere(submodule);
  def_class_Capsule(submodule);
  def_class_CapsuleSequence(submodule);
  def_class_Mesh(submodule);
  def_class_VoxelOctree(submodule);
  def_class_OctomapWrap(submodule);
  def_function_collides(submodule);
  def_function_collides_self(submodule);
  return submodule;
}
