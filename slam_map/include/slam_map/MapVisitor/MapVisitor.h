// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <limits>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>

/**
 * A generic map "visitor" for use in graphs searches.
 *
 * @note Subclasses of this class are not expected to be
 * reusable. Each new search should create a new visitor.
 */
class MapVisitor {
 public:
  static const uint32_t kMaxDepth = std::numeric_limits<uint32_t>::max();
  MapVisitor() : depth_(0),
                 should_ignore_broken_(false),
                 depth_equals_count_(false),
                 has_explore_edge_(false),
                 has_explore_node_(false),
                 has_visit_(false) {}

  /**
   * Called after getting a new edge to explore, but before adding a
   * node to the queue
   */
  virtual void ExploreEdge(const SlamFramePtr& /* parent */,
                           const SlamEdgePtr& /* edge */,
                           const SlamFramePtr& /* child */) {}

  /**
   * Called before adding a node to the search queue
   *
   * @returns true if this node should be added to the visit list.
   */
  virtual bool ExploreNode(const SlamFramePtr& /* parent */,
                           const SlamEdgePtr& /* edge */,
                           const SlamFramePtr& /* child */) {
    return true;
  }

  /**
   * Called when a new node is visited
   *
   * @returns true if the children of this node should be added.
   */
  virtual bool Visit(const SlamFramePtr& /* node */) {
    return true;
  }

  /**
   * Called when the search has completed.
   *
   * Can be used to finalize search-related tasks before returning
   * from search call. Always called: no 'has_finished()' check.
   */
  virtual void Finished() {}

  /**
   * Should the search finish early?
   */
  virtual bool IsDone() {
    return false;
  }

  virtual ~MapVisitor() {}

  ReferenceFrameId root_id() const {
    return root_id_;
  }

  void set_root_id(const ReferenceFrameId& r) {
    root_id_ = r;
  }

  uint32_t depth() const {
    return depth_;
  }

  void set_depth(uint32_t d) {
    depth_ = d;
  }

  bool should_ignore_broken() const {
    return should_ignore_broken_;
  }

  void set_should_ignore_broken(bool i) {
    should_ignore_broken_ = i;
  }

  bool depth_equals_count() const {
    return depth_equals_count_;
  }

  void set_depth_equals_count(bool d) {
    depth_equals_count_ = d;
  }

  bool has_explore_edge() {
    return has_explore_edge_;
  }

  bool has_explore_node() {
    return has_explore_node_;
  }

  bool has_visit() {
    return has_visit_;
  }

 protected:
  void set_has_explore_edge(bool has) { has_explore_edge_ = has; }
  void set_has_explore_node(bool has) { has_explore_node_ = has; }
  void set_has_visit(bool has) { has_visit_ = has; }

 private:
  ReferenceFrameId root_id_;
  uint32_t depth_;
  bool should_ignore_broken_;
  bool depth_equals_count_;
  bool has_explore_edge_, has_explore_node_, has_visit_;
};
