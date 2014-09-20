// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <map>
#include <memory>
#include <set>
#include <miniglog/logging.h>
#include <pb_msgs/SlamServer.pb.h>
#include <slam_server/SlamServerFwd.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/SlamMapFwd.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>

/** MapVisitor which gathers frames and edges for use with SlamServer */
class ServerFetchMapVisitor : public MapVisitor {
 public:
  /**
   * Build a visitor with objects to fill out/use for parsing
   *
   * Will expand the given vector with messages as they get too large.
   *
   * @param server Server to use when a full message should be uploaded.
   * @param map Map being iterated over. Used for querying cameras.
   * @param [optional] matcher PlaceMatcher used for adding places to msg.
   * @param [optional] excluding_id_msg MapId to exclude when traversing map
   */
  ServerFetchMapVisitor(const std::shared_ptr<SlamServerInterface>& server,
                        const SlamMap* const map,
                        const pb::SessionIdMsg* const excluding_id_msg);

  /**
   * Build a visitor with objects to fill out/use for parsing
   *
   * Will only collect as many frames as it can without exceeding the
   * max encoded size for the output protobuf.
   *
   * @param map Map being iterated over. Used for querying cameras.
   * @param [optional] matcher PlaceMatcher used for adding places to msg.
   * @param [optional] excluding_id_msg MapId to exclude when traversing map
   * @param [out] msg Message that gets written to during BFS.
   */
  ServerFetchMapVisitor(const SlamMap* const map,
                        const pb::SessionIdMsg* const excluding_id_msg,
                        pb::PlaceMapMsg* out);
  ~ServerFetchMapVisitor();

  void ExploreEdge(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override;

  bool Visit(const SlamFramePtr& cur_node) override;
  void Finished() override;

  bool IsDone() override {
    return finished_;
  }

  /**
   * Configure the earliest time at which items will be retrieved.
   *
   * This is used to bound a search on the modification time of edges
   * and frames. If the last modified time of an edge or frame is less
   * than to the last_map_fetch_time_, it will not be retrieved and
   * the search will halt there.
   */
  void set_last_map_fetch_time(double t) {
    last_map_fetch_time_ = t;
  }

  void set_template_matcher(const TemplateMatcher* tm) {
    template_matcher_ = tm;
    UpdateMsgPlaceType();
  }

  void set_dbow_matcher(const DBoWMatcher* dm) {
    dbow_matcher_ = dm;
    UpdateMsgPlaceType();
  }

  void set_multidbow_matcher(const MultiDBoWMatcher* mdm) {
    multidbow_matcher_ = mdm;
    UpdateMsgPlaceType();
  }

 protected:
  /** Delegated constructor without output type */
  ServerFetchMapVisitor(const SlamMap* const map,
                        const pb::SessionIdMsg* const excluding_id_msg);

  /** Deal with messages that are too large */
  void CheckMessageSize();

  /**
   * Checks the size of the current message.
   *
   * @returns true if the message is large enough to warrant starting
   *          a new one
   */
  bool CurrentMessageTooLarge() const;
  void UpdateMsgPlaceType();

 private:
  std::shared_ptr<SlamServerInterface> server_;
  const SlamMap* const map_;
  const TemplateMatcher* template_matcher_;
  const DBoWMatcher* dbow_matcher_;
  const MultiDBoWMatcher* multidbow_matcher_;
  pb::PlaceMapMsg* current_msg_;
  std::set<SessionId> session_ids_;
  SessionId excluding_id_;
  bool has_excluding_id_;
  double last_map_fetch_time_;
  bool finished_;
  size_t num_frames_;

  // The set of nodes which have been explored, but not yet
  // visited. This is for tracking leaves of the graph.
  std::set<ReferenceFrameId> explored_;
};
