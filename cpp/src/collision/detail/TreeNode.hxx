#ifndef TREE_NODE_HXX
#define TREE_NODE_HXX

#include <algorithm>
#include <functional>
#include <memory>

#include <cstdint>

namespace collision {
namespace detail {

template <size_t _Nt>
TreeNode<_Nt>::TreeNode(const TreeNode<_Nt> &other) { // copy
  for (int i = 0; i < 8; i++) {
    auto &child = other._children[i];
    if (child) {
      _children[i] = std::make_unique<Child>(*child);
    }
  }
}

template <size_t _Nt>
TreeNode<_Nt>& TreeNode<_Nt>::operator= (const TreeNode<_Nt> &other) { // copy
  for (int i = 0; i < 8; ++i) {
    auto &child = other._children[i];
    if (child) {
      _children[i] = std::make_unique<Child>(*child);
    } else {
      _children[i].reset(nullptr); // delete
    }
  }
  return *this;
}

template <size_t _Nt>
bool TreeNode<_Nt>::operator== (const TreeNode<_Nt> &other) const {
  // first check suboccupancy
  for (int i = 0; i < 8; i++) {
    if (bool(_children[i]) != bool(other._children[i])) {
      return false;
    }
  }

  // recurse down
  for (int i = 0; i < 8; i++) {
    auto &a = _children[i];
    auto &b = other._children[i];
    if (a && b && !(*a == *b)) {
      return false;
    }
  }

  return true;
}

template <size_t _Nt>
size_t TreeNode<_Nt>::nblocks() const {
  size_t child_nblocks = 0;
  for (auto &child : _children) {
    if (child) {
      child_nblocks += child->nblocks();
    }
  }
  return child_nblocks;
}

template <size_t _Nt>
bool TreeNode<_Nt>::is_empty() const {
  return std::all_of(_children.begin(), _children.end(),
                     [] (auto &x) { return x.get() == nullptr; });
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::block(size_t bx, size_t by, size_t bz) const {
  const ChildPtr &child = _children[idx(bx, by, bz)];
  if (child) {
    return child->block(bx % child_Nbt(), by % child_Nbt(), bz % child_Nbt());
  }
  return 0;
}

template <size_t _Nt>
void TreeNode<_Nt>::set_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (!child && value) {
    child = std::make_unique<Child>();
  }
  if (child) {
    child->set_block(bx % child_Nbt(), by % child_Nbt(), bz % child_Nbt(), value);
    if (!value && child->is_empty()) {
      child.reset(nullptr); // delete
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::union_tree(const TreeNode<_Nt> &other) {
  for (int i = 0; i < 8; i++) {
    ChildPtr &a = _children[i];
    const ChildPtr &b = other._children[i];
    if (a && b) {
      a->union_tree(*b);
    } else if (!a && b) {
      a = std::make_unique<Child>(*b);
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::intersect_tree(const TreeNode<_Nt> &other) {
  for (int i = 0; i < 8; i++) {
    ChildPtr &a = _children[i];
    const ChildPtr &b = other._children[i];
    if (a && b) {
      a->intersect_tree(*b);
      if (a->is_empty()) {
        a.reset(nullptr); // delete to prune this part of the tree
      }
    } else if (a) {
      a.reset(nullptr); // delete this subtree entirely
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::remove_tree(const TreeNode<_Nt> &other) {
  for (int i = 0; i < 8; i++) {
    ChildPtr &a = _children[i];
    const ChildPtr &b = other._children[i];
    if (a && b) {
      a->remove_tree(*b);
      if (a->is_empty()) {
        a.reset(nullptr); // delete to prune this part of the tree
      }
    }
  }
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::union_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (!child) {
    child = std::make_unique<Child>();
  }
  return child->union_block(
      bx % child_Nbt(), by % child_Nbt(), bz % child_Nbt(), value);
}

template <size_t _Nt>
uint64_t TreeNode<_Nt>::intersect_block(size_t bx, size_t by, size_t bz, uint64_t value) {
  ChildPtr &child = _children[idx(bx, by, bz)];
  if (child) {
    auto oldval = child->intersect_block(
        bx % child_Nbt(), by % child_Nbt(), bz % child_Nbt(), value);
    if (child->is_empty()) {
      child.reset(nullptr); // delete to prune this part of the tree
    }
    return oldval;
  }
  return 0;
}

template <size_t _Nt>
bool TreeNode<_Nt>::collides(const TreeNode<_Nt> &other) const {
  for (int i = 0; i < 8; i++) {
    if (_children[i] && other._children[i]
        && _children[i]->collides(*other._children[i]))
    {
      return true;
    }
  }
  return false;
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_leaves_impl(const TreeNode<_Nt>::ConstVisitor &visitor,
                                      size_t dx, size_t dy, size_t dz) const
{
  for (size_t bx = 0; bx < Nbt(); bx += child_Nbt()) {
    for (size_t by = 0; by < Nbt(); by += child_Nbt()) {
      for (size_t bz = 0; bz < Nbt(); bz += child_Nbt()) {
        auto &child = _children[idx(bx, by, bz)];
        if (child) {
          child->visit_leaves_impl(visitor, dx + bx, dy + by, dz + bz);
        }
      }
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_blocks_impl(const ConstVisitor &visitor,
                                      size_t dx, size_t dy, size_t dz) const
{
  for (size_t bx = 0; bx < Nbt(); bx += child_Nbt()) {
    for (size_t by = 0; by < Nbt(); by += child_Nbt()) {
      for (size_t bz = 0; bz < Nbt(); bz += child_Nbt()) {
        auto &child = _children[idx(bx, by, bz)];
        if (child) {
          child->visit_blocks_impl(visitor, dx + bx, dy + by, dz + bz);
        } else {
          visitor(dx + bx, dy + by, dz + bz, uint64_t(0));
        }
      }
    }
  }
}

template <size_t _Nt>
void TreeNode<_Nt>::visit_modify_blocks_impl(const ModVisitor &visitor,
                                             size_t dx, size_t dy, size_t dz)
{
  for (size_t bx = 0; bx < Nbt(); bx += child_Nbt()) {
    for (size_t by = 0; by < Nbt(); by += child_Nbt()) {
      for (size_t bz = 0; bz < Nbt(); bz += child_Nbt()) {
        auto &child = _children[idx(bx, by, bz)];
        if (child) {
          child->visit_blocks_impl(visitor, dx + bx, dy + by, dz + bz);
          // prune child if now empty
          if (child->is_empty()) {
            child.reset(nullptr); // delete
          }
        } else {
          auto blockval = visitor(dx + bx, dy + by, dz + bz, uint64_t(0));
          // set the block value
          if (blockval) {
            child = std::make_unique<Child>();
            child->set_block(bx, by, bz, blockval);
          }
        }
      }
    }
  }
}

// leaf node
template <> class TreeNode<4> {
public:
  using ConstVisitor = std::function<void(size_t, size_t, size_t, uint64_t)>;
  using ModVisitor = std::function<uint64_t(size_t, size_t, size_t, uint64_t)>;

  static constexpr size_t Nt() { return 4;  }
  static constexpr size_t N()  { return 64; }
  TreeNode()                                     = default; // default constructor
  TreeNode(const TreeNode<4> &other)             = default; // copy constructor
  TreeNode(TreeNode<4> &&other)                  = default; // move constructor
  TreeNode& operator= (const TreeNode<4> &other) = default; // copy assignment
  TreeNode& operator= (TreeNode<4> &&other)      = default; // move assignment
  bool operator ==(const TreeNode<4> &other) const { return _tree == other._tree; }
  size_t nblocks() const { return 1; }
  bool is_empty() const { return !_tree; }
  uint64_t block(size_t, size_t, size_t) const { return _tree; }
  void union_tree(const TreeNode<4> &other) { _tree |= other._tree; }
  void intersect_tree(const TreeNode<4> &other) { _tree &= other._tree; }
  void remove_tree(const TreeNode<4> &other) { _tree &= ~(other._tree); }
  uint64_t union_block(size_t, size_t, size_t, uint64_t value) {
    auto prev = _tree;
    _tree |= value;
    return prev;
  }
  uint64_t intersect_block(size_t, size_t, size_t, uint64_t value) {
    auto prev = _tree;
    _tree &= value;
    return prev;
  }
  void set_block(size_t, size_t, size_t, uint64_t value) { _tree = value; }
  bool collides(const TreeNode<4> &other) const { return _tree & other._tree; }
  void visit_leaves(const ConstVisitor &visitor) const { visitor(0, 0, 0, _tree); }
  void visit_blocks(const ConstVisitor &visitor) const { visitor(0, 0, 0, _tree); }
  void visit_modify_blocks(const ModVisitor &visitor) { _tree = visitor(0, 0, 0, _tree); }
  void visit_leaves_impl(const ConstVisitor &visitor,
                         size_t dx, size_t dy, size_t dz) const
  { visitor(dx, dy, dz, _tree); }
  void visit_blocks_impl(const ConstVisitor &visitor,
                         size_t dx, size_t dy, size_t dz) const
  { visitor(dx, dy, dz, _tree); }
  void visit_modify_blocks_impl(const ModVisitor &visitor,
                                size_t dx, size_t dy, size_t dz)
  { _tree = visitor(dx, dy, dz, _tree); }
protected:
  uint64_t _tree = 0;
};

} // end of namespace detail
} // end of namespace collision

#endif // TREE_NODE_HXX
