// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <map>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <common/MathTypes.h>
#include <slam_map/slam_mapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/TransformEdgeId.h>

/**
 * A snapshot of absolute coordinates of frames and landmarks with
 * respect to the first frame added to the View.
 *
 * This class uses a GlobalMapViewUpdater internally to maintain
 * synchronization with the slam_mapProxy it's given.
 */
class GlobalMapView {
  typedef std::lock_guard<std::mutex> LockGuardT;

 public:
  GlobalMapView(const std::shared_ptr<slam_mapProxy>& map);
  virtual ~GlobalMapView();

  void Reset(const std::shared_ptr<slam_mapProxy>& map);

  /** Configure which frame is used as the global identity */
  void set_root_id(const ReferenceFrameId& id);

  ReferenceFrameId root_id() {
    return root_id_;
  }

  bool has_root_id() {
    return has_root_id_;
  }

  /**
   * Get the global pose of a frame
   *
   * @param [in]  uId ID of the frame
   * @param [out] out Global pose of the frame
   * @returns         True if the frame exists in this view
   */
  bool GetFramePose(const ReferenceFrameId& uId, Sophus::SE3t* out) const;

  /**
   * Get vertex arrays for OpenGL drawing.
   *
   * @todo chachi Move into subclass.
   */
  void LinesForGL(std::vector<TransformEdgeId>* edges,
                  Eigen::Vector3tArray* points,
                  std::vector<int>* session_ids,
                  int* num_sessions) const;

  void SetFramePose(const ReferenceFrameId& id, const Sophus::SE3t& pose);
  void AddEdge(const TransformEdgeId& id);
  void UpdateEdge(const TransformEdgeId& id);

  /**
   * Clears current contents and loads the entire map in through the proxy.
   */
  void ResetAndLoadWholeMap();

 protected:
  inline void set_root_id_inline(const ReferenceFrameId& id);
  inline void AddEdgeInline(const TransformEdgeId& id);
  inline bool GetFramePoseInline(const ReferenceFrameId& uId,
                                 Sophus::SE3t* out) const;

  /** Update all children of the dest frame using the map's edge from
   * root to dest. */
  inline void UpdateFromRoot(const ReferenceFrameId& root,
                             const ReferenceFrameId& dest);

  inline void UpdateEdgeFromMap(const ReferenceFrameId& a,
                                const ReferenceFrameId& b);
  inline void UpdateEdgeId(const TransformEdgeId& id);
  inline bool UpdateEdge(const TransformEdgeId& edge,
                         const ReferenceFrameId& a,
                         const ReferenceFrameId& b,
                         const Sophus::SE3t& t_ab);

 private:
  std::shared_ptr<slam_mapProxy> map_;
  std::shared_ptr<GlobalMapViewUpdater> updater_;
  std::unordered_map<ReferenceFrameId, Sophus::SE3t> frames_;
  std::unordered_set<TransformEdgeId> edges_;
  std::unordered_map<ReferenceFrameId,
                     std::unordered_set<TransformEdgeId> > tree_;
  ReferenceFrameId root_id_;
  bool has_root_id_;
  mutable std::mutex mutex_;
  mutable std::map<SessionId, int> maps_;
  friend class LoadWholeMapVisitor;
};
