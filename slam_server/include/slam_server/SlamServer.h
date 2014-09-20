// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <stdint.h>
#include <map>
#include <mutex>
#include <place_matching/PlaceMatcherFactory.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_server/SlamServerInterface.h>
#include <pb_msgs/rslam.pb.h>

class TemplateMatcher;
class BackEnd;

/**
 * Concrete implementation of SlamServerInterface
 *
 * This object will actually act upon the calls it receives, not pass
 * them along.
 */
class SlamServer : public SlamServerInterface {
 public:
  SlamServer(const std::shared_ptr<PlaceMatcher>& matcher,
             const std::shared_ptr<SlamMap>& map);
  virtual ~SlamServer();

  void UploadMap(const pb::PlaceMapMsg& map) override;

  void UploadMap(const pb::ServerUploadRequest& request,
                 pb::ServerUploadResponse* response) override;

  void QueryPlace(const pb::ServerQueryRequest& request,
                  pb::ServerQueryResponse* response) override;

  void DownloadMap(const pb::ServerDownloadRequest& request,
                   pb::ServerDownloadResponse* response) const override;

  void Save() const;

  bool QueryPlace(const pb::CameraMsg& images,
                  const pb::CameraRigMsg& rig_msg,
                  const pb::ReferenceFrameMsg& frame_msg,
                  pb::TransformEdgeMsg* to_match,
                  std::vector<pb::MultiViewMeasurementMsg>* meas);

  double FetchToDepth(const pb::ReferenceFrameIdMsg& id,
                      size_t depth,
                      double last_fetch_time,
                      const pb::SessionIdMsg* const excluding,
                      pb::PlaceMapMsg* to_fetch) const;

  /** Remove ALL map and place data from server */
  void Reset();

  void PrintStats() const;

 protected:
  inline bool UsingTemplateMatcher() const;
  inline bool UsingDBoWMatcher() const;
  inline bool UsingMultiDBoWMatcher() const;

 private:
  std::shared_ptr<SlamMap> map_;
  std::shared_ptr<PlaceMatcher> place_matcher_;
  std::unique_ptr<BackEnd> backend_;
  mutable std::mutex mutex_;
};
