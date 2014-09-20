// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <atomic>
#include <map>
#include <vector>
#include <opencv2/core/core.hpp>
#include <slam_server/SlamServerFwd.h>
#include <slam_map/SlamMapFwd.h>

class PlaceMatcher;
class TemplateMatcher;

/**
 * Proxy through which a SlamServer can be accessed.
 *
 * Provides an interface that accepts regular rslam types and handles
 * the conversion to Protobufs then sends to the interface given in
 * the constructor.
 */
class SlamServerProxy {
 public:
  explicit SlamServerProxy(const std::shared_ptr<SlamServerInterface>& server);
  virtual ~SlamServerProxy();

  /** Upload entire map with this map's MapId to server */
  void UploadMap(const SlamMap& map,
                 const std::shared_ptr<PlaceMatcher>& place_matcher,
                 const ReferenceFrameId& root_id);

  /**
   * Query given images against server
   *
   * @param [in] images Query images
   * @param [in] rig Camera rig which captured images
   * @param [in] frame Frame corresponds to query images
   * @param [out] Edge between query frame and matched node OR null if no
   * @param [out] Measurements made between query image and matched map
   * match made
   */
  bool QueryPlace(const std::vector<cv::Mat>& images,
                  const CameraRigPtr& rig,
                  const ReferenceFrame& frame,
                  SlamEdgePtr* edge,
                  std::vector<MultiViewMeasurement>* measurements) const;

  /**
   * Fetch a section of map rooted at the given ID
   *
   * @param [in] id Root of search
   * @param [in] depth Depth of search radius in # of edges
   * @param [in] excluding Skip nodes from the map with this id
   * @param [out] map Fetched nodes are added to the given map
   * @param [out] matcher PlaceMatcher where places are added
   * @param [out] place_to_frame Mapping between SlamMap and PlaceMatcher
   */
  void FetchToDepth(const ReferenceFrameId& id,
                    size_t depth,
                    const SessionId* const excluding,
                    SlamMap* map,
                    PlaceMatcher* matcher) const;

  void set_server_interface(const std::shared_ptr<SlamServerInterface>& server);

 private:
  std::shared_ptr<SlamServerInterface> server_;
  double last_upload_time_;
  mutable std::atomic<double> last_fetch_time_;
};
