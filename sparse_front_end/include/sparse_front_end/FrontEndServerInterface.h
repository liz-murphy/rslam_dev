// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <future>
#include <mutex>
#include <slam_server/SlamServerFwd.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <opencv2/core/core.hpp>

class PlaceMatcher;

namespace rslam {
namespace sparse {

class FrontEndServerInterface {
 public:
  FrontEndServerInterface() = default;
  FrontEndServerInterface(const FrontEndServerInterface&) = default;
  virtual ~FrontEndServerInterface();

  /**
   * Query a server with a place and fetch any matched map.
   *
   * Function takes parameters by value because it is designed to be
   * called asynchronously and needs to be robust to the pointers being
   * reset while it runs.
   */
  bool QueryServerWithPlace(std::shared_ptr<SlamMap> map,
                            std::shared_ptr<PlaceMatcher> place_matcher,
                            ReferenceFrameId id,
                            cv::Mat image);

  void AsyncQueryServerWithPlace(
      const std::shared_ptr<SlamMap>& map,
      const std::shared_ptr<PlaceMatcher>& place_matcher,
      const ReferenceFrameId& id,
      const cv::Mat& image);

  /**
   * Upload the map and places to the server
   *
   * Function takes parameters by value because it is designed to be
   * called asynchronously and needs to be robust to the pointers being
   * reset while it runs.
   */
  void UploadMapToServer(std::shared_ptr<SlamMap> map,
                         std::shared_ptr<PlaceMatcher> place_matcher,
                         ReferenceFrameId root_id);

  void AsyncUploadMapToServer(
      const std::shared_ptr<SlamMap>& map,
      const std::shared_ptr<PlaceMatcher>& place_matcher,
      const ReferenceFrameId& root_id);

  void set_server_proxy(const std::shared_ptr<SlamServerProxy>& proxy) {
    server_proxy_ = proxy;
  }

  double last_download_time() const {
    return last_download_time_;
  }

 protected:
  bool IsServerQueryReady() const;
  bool IsServerUploadReady() const;

 private:
  double last_download_time_ = 0.0;
  std::future<bool> server_query_future_;
  std::future<void> server_upload_future_;
  std::shared_ptr<SlamServerProxy> server_proxy_;
};
}  // namespace rslam
}  // namespace sparse
