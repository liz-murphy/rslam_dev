// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

// \file slam_map.h This is the primary SLAM data structure. A map
// holds the SLAM graph, frames, landmarks and measurements.

#pragma once

#include <algorithm>
#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>

#include <ba/Types.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/MapEvent.h>
#include <slam_map/Landmark.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/TransformEdgeId.h>
#include <slam_map/DataStore/SlamMapDataStore.h>
#include <utils/MathTypes.h>

struct HoldRequest {
  ReferenceFrameId root_id;
  unsigned int depth;
  bool with_covisible;
};

class SlamMap {
 public:
  // Is this map used for any number of sessions or only for a single
  // track. Single track operation is more optimized, but also
  // limited.
  enum SessionStorage {
    kSingleSessionStorage,
    kMultipleSessionStorage
  };

  SlamMap();
  ~SlamMap();

  ///
  /// Initialize a persistent map, backed by the given file
  ///
  void InitWithPersistence(const std::string& filename,
                           bool continue_existing);

  ///
  /// Initialize a transient, in memory map
  ///
  void InitInMemory(const SessionStorage& storage_type);

  /// Functions that MODIFY the map

  /// Allocate a new frame, but do not link it into the graph.
  SlamFramePtr AddFrame(const double time);

  /// Add an existing node to the map
  bool AddFrame(const SlamFramePtr& node);

  /// Add edge between frames
  SlamEdgePtr AddEdge(const SlamFramePtr& pA,
                      const SlamFramePtr& pB,
                      const Sophus::SE3t &Tab,
                      bool is_loop_closure = false);

  /// Add an existing edge to the map
  bool AddEdge(const SlamEdgePtr& edge);

  /// Update the relative transformation between frames already linked
  void UpdateEdgeImuPoints(const TransformEdgeId& edge_id,
                           const std::vector<ba::ImuPoseT<Scalar> > &vMeas,
                           const Eigen::Vector3t& g);

  bool SwapEdgeStartFrame(const TransformEdgeId& edge_id,
                          const ReferenceFrameId& old_start_id,
                          const ReferenceFrameId& new_start_id,
                          const Sophus::SE3t new_tab);
  size_t NumCameras() const;

  void AddCamera(const SessionId& session_id, const _CameraRigPtr& cam);
  void SetCamera(const SessionId& session_id, const _CameraRigPtr& cam);
  CameraRigPtr GetCamera(const SessionId& session_id) const;
  bool HasCamera(const SessionId& session_id) const;

  /// Sparse methods
  void AddNewMeasurementsToFrame(
      const ReferenceFrameId& frame_id,
      std::vector<MultiViewMeasurement> &measurements);
  void RemoveMeasurementsFromFrame(const ReferenceFrameId& frame_id);

  /// Clear Map
  void Clear();

  /// Save map to disk (if possible)
  void Save() const;

  /// Can this map save itself to disk?
  bool CanSave() const;

  /// Get map data/info (READ ONLY)

  /// Query whether the given frame is already loaded into memory
  bool IsFrameLoaded(const ReferenceFrameId& id) const;

  /// Query whether the given edge is already loaded into memory
  bool IsEdgeLoaded(const TransformEdgeId& id) const;

  SlamFramePtr GetFramePtr(const MeasurementId& id) const;
  SlamFramePtr GetFramePtr(const ReferenceFrameId& frame_id) const;
  SlamEdgePtr GetEdgePtr(const ReferenceFrameId& start_id,
                         const ReferenceFrameId& end_id) const;
  SlamEdgePtr GetEdgePtr(const TransformEdgeId& edge_id) const;

  /// Retrieve a frame, but only if it's currently in memory
  SlamFramePtr GetFramePtrLoaded(const ReferenceFrameId& frame_id) const;

  /// Retrieve an edge, but only if it's currently in memory
  SlamEdgePtr GetEdgePtrLoaded(const TransformEdgeId& frame_id) const;

  unsigned int NumEdges() const;
  unsigned int NumFrames() const;
  unsigned int NumSessions() const;

  bool HasFrame(const ReferenceFrameId& id) const;
  bool HasEdge(const ReferenceFrameId& start_id,
               const ReferenceFrameId& end_id) const;

  // Return reference to measuement based on measurement Id
  bool GetMeasurement(const MeasurementId& id, MultiViewMeasurement& Msr) const;

  // Return reference in Msr to measurement identified by a Frame Id and a local
  // measurement index. True if found, false otherwise.
  bool GetMeasurement(const ReferenceFrameId& frame_id,
                      const unsigned int uMsrLocalIndex,
                      MultiViewMeasurement& Msr) const;

  bool GetLandmark(const LandmarkId& Id, Landmark* lm) const;

  ///
  /// Get a unique token for specifying held frames
  ///
  uint64_t GetHoldToken() const;
  void RemoveFrameHold(uint64_t token);

  void SetHoldFrames(uint64_t token,
                     const ReferenceFrameId& root_id,
                     unsigned int depth,
                     bool with_covisible);

  /** Set the given frames as held in memory, blocking until they are loaded */
  void SetHoldFramesSync(uint64_t token,
                         const ReferenceFrameId& root_id,
                         unsigned int depth,
                         bool with_covisible);

  /** Get the IDs associated with all sessions in this map */
  void sessions(std::vector<SessionId>* sessions) const;

  /** Find the lowest indexed frame for the given track */
  ReferenceFrameId SessionStart(const SessionId& session) const;

  SessionId id() const {
    return id_;
  }

  void set_id(const SessionId& id) {
    id_ = id;
  }

  void ClearEdgeAttributes(uint32_t attribute);

  void SetEdgeAttribute(const TransformEdgeId& id, uint32_t attribute);

  uint32_t GetEdgeAttribute(const TransformEdgeId& id) const;

  /** Breadth-first search */
  void BFS(MapVisitor* visitor) const;

  /** Breadth-first search of only nodes and edges in memory*/
  void InMemoryBFS(MapVisitor* visitor) const;

  /** Traverses the map along parent edges */
  void ParentTraverse(MapVisitor* visitor) const;

  void Subscribe(const std::vector<rslam::map::MapEvent>& event_subscriptions,
                 const rslam::map::NotificationCallback& listener);

  void UpdateDebugLevel(int level){debug_level_=level;};
  void UpdateHoldDebugLevel(int level){hold_debug_level_=level;};

 protected:
  bool _FindEdgeId(const ReferenceFrameId& start_id,
                   const ReferenceFrameId& end_id,
                   TransformEdgeId& edge_id) const;

  void UpdateHeldFrames(uint64_t token);

  /**
   * Main BFS function
   *
   * @param only_loaded If true, only progresses up to the boundary of
   * loaded frames and edges
   */
  void InternalBFS(MapVisitor* visitor, bool only_loaded) const;

 private:
  std::shared_ptr<SlamMapDataStore> store_;

  mutable std::atomic<uint64_t> token_, hold_token_;
  mutable std::map<uint64_t, bool> hold_updating_;
  std::map<uint64_t, HoldRequest> holds_;
  std::map<uint64_t, std::vector<SlamFramePtr> > held_frames_;
  std::map<uint64_t, std::vector<SlamEdgePtr> > held_edges_;

  mutable std::mutex hold_mutex_, hold_update_mutex_, edge_attrib_mutex_;

  /** The last frame id issued from this map */
  std::atomic<int> last_frame_id_;

  /** Edge attribute map */
  std::unordered_map<TransformEdgeId, uint32_t> edge_attributes_;

  /** The unique identifier for this map for all time */
  SessionId id_;
  std::shared_ptr<rslam::map::NotificationCenter> notifier_;

  /** Replace CVars functionality with private member variables, accessors and dynamic reconfigure*/
  int hold_debug_level_;
  int debug_level_;
};
