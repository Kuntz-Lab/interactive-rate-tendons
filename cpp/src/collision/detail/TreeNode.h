#ifndef TREE_NODE_H
#define TREE_NODE_H

#include <array>
#include <functional>
#include <memory>

#include <cstdint>

namespace collision {
namespace detail {

template <size_t _Nt> class TreeNode {
  static_assert(_Nt % 2 == 0, "TreeNode: number of nodes must be even");

public:
  static constexpr size_t Nt()  { return _Nt;         }   // voxel dimension size
  static constexpr size_t N()   { return _Nt*_Nt*_Nt; }   // total voxels
  static constexpr size_t Nbt() { return _Nt / 4;     }   // block dimension size
  static constexpr size_t Nb()  { return N() / 64;    }   // total blocks
  static constexpr size_t child_Nt()  { return _Nt / 2; } // each child node voxel dimension size
  static constexpr size_t child_Nbt() { return _Nt / 8; } // each child node block dimension size

  using Child = TreeNode<child_Nt()>;
  using ChildPtr = std::unique_ptr<Child>;

  using ConstVisitor = std::function<void(size_t, size_t, size_t, uint64_t)>;
  using ModVisitor = std::function<uint64_t(size_t, size_t, size_t, uint64_t)>;

  TreeNode() : _children() {}
  TreeNode(const TreeNode<_Nt> &other);        // copy
  TreeNode(TreeNode<_Nt> &&other) = default;   // move
  TreeNode<_Nt>& operator= (const TreeNode<_Nt> &other);      // copy
  TreeNode<_Nt>& operator= (TreeNode<_Nt> &&other) = default; // move
  bool operator ==(const TreeNode<_Nt> &other) const;
  size_t nblocks() const;
  bool is_empty() const;
  uint64_t block(size_t bx, size_t by, size_t bz) const;
  void set_block(size_t bx, size_t by, size_t bz, uint64_t value);
  void union_tree(const TreeNode &other);
  void intersect_tree(const TreeNode &other);
  void remove_tree(const TreeNode &other);
  uint64_t union_block(size_t bx, size_t by, size_t bz, uint64_t value);
  uint64_t intersect_block(size_t bx, size_t by, size_t bz, uint64_t value);
  bool collides(const TreeNode<_Nt> &other) const;

  void visit_leaves(const ConstVisitor &visitor) const {
    visit_leaves_impl(visitor, 0, 0, 0);
  }

  void visit_blocks(const ConstVisitor &visitor) const {
    visit_blocks_impl(visitor, 0, 0, 0);
  }

  void visit_modify_blocks(const ModVisitor &visitor) {
    visit_modify_blocks_impl(visitor, 0, 0, 0);
  }

  void visit_leaves_impl(const ConstVisitor &visitor,
                         size_t dx, size_t dy, size_t dz) const;
  void visit_blocks_impl(const ConstVisitor &visitor,
                         size_t dx, size_t dy, size_t dz) const;
  void visit_modify_blocks_impl(const ModVisitor &visitor,
                                size_t dx, size_t dy, size_t dz);
protected:
  size_t idx(size_t bx, size_t by, size_t bz) const {
    return (bz/child_Nbt()) + 2*(by/child_Nbt()) + 4*(bx/child_Nbt());
  }

protected:
  std::array<ChildPtr, 8> _children;
};

} // end of namespace detail
} // end of namespace collision

#include "TreeNode.hxx"

#endif // TREE_NODE_H
