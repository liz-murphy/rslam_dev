// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <sparse_front_end/FrontEndServerInterface.h>

#include <future>
#include <utils/TicToc.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <slam_server/SlamServerProxy.h>
#include <slam_map/SlamMap.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>

#include <ros/ros.h>

namespace rslam {
namespace sparse {

FrontEndServerInterface::~FrontEndServerInterface() {
  int n_tries = 0;
  while (!IsServerQueryReady() && n_tries++ < 10) {
    ROS_INFO("Waiting for server interface to finish");
    usleep(1000);
  }
}

void FrontEndServerInterface::AsyncQueryServerWithPlace(
    const std::shared_ptr<SlamMap>& map,
    const std::shared_ptr<PlaceMatcher>& place_matcher,
    const ReferenceFrameId& id,
    const cv::Mat& image) {
  if (!IsServerQueryReady()) return;

  server_query_future_ = std::async(
      std::launch::async,
      &FrontEndServerInterface::QueryServerWithPlace, this,
      map, place_matcher, id, image.clone());
}

bool FrontEndServerInterface::QueryServerWithPlace(
    std::shared_ptr<SlamMap> map,
    std::shared_ptr<PlaceMatcher> place_matcher,
    ReferenceFrameId id,
    cv::Mat image) {
  if (id.session_id != map->id()) {
    ROS_WARN("Querying server with a frame which is not ours: %d", id.id);
    return false;
  }

  CameraRigPtr rig = map->GetCamera(id.session_id);
  SlamFramePtr frame = map->GetFramePtr(id);
  if (!rig) {
    ROS_DEBUG_NAMED("sparse_front_end::Server", "No rig available for frame %d while querying server", id.id);
    return false;
  } else if (!frame) {
    ROS_DEBUG_NAMED("sparse_front_end::Server", "No frame available for frame %d while querying server", id.id);
    return false;
  }

  SlamEdgePtr edge;
  std::vector<MultiViewMeasurement> measurements;
  if (!server_proxy_->QueryPlace({image}, rig, *frame, &edge, &measurements)) {
    return false;
  }

  ROS_DEBUG_NAMED("sparse_front_end::Server","Match found on server for frame %d! Fetching map to depth of %d", id.id, server_download_map_size_);

  TransformEdgeId edge_id = edge->id();
  ReferenceFrameId outgoing_frame_id =
      (edge_id.start == id) ? edge_id.end : edge_id.start;
  SessionId current_session_id = map->id();

  ROS_DEBUG_NAMED("sparse_front_end::Server", "After query, adding edge %d", edge_id.id);
  map->AddEdge(edge);

  for (MultiViewMeasurement& z : measurements) {
    frame->AddMeasurement(z);
  }

  last_download_time_ = hal::Tic();
  server_proxy_->FetchToDepth(
      outgoing_frame_id, server_download_map_size_,
      &current_session_id, map.get(),
      place_matcher.get());
  return true;
}

void FrontEndServerInterface::AsyncUploadMapToServer(
    const std::shared_ptr<SlamMap>& map,
    const std::shared_ptr<PlaceMatcher>& place_matcher,
    const ReferenceFrameId& root_id) {
  if ((map->NumFrames() % server_upload_map_size_) != 0 ||
      !IsServerUploadReady()) {
    return;
  }

  server_upload_future_ = std::async(
      std::launch::async,
      &FrontEndServerInterface::UploadMapToServer, this,
      map, place_matcher, root_id);
}

void FrontEndServerInterface::UploadMapToServer(
    std::shared_ptr<SlamMap> map,
    std::shared_ptr<PlaceMatcher> place_matcher,
    ReferenceFrameId root_id) {
  ROS_DEBUG_NAMED("sparse_front_end::Server","Uploading map with %d frames.", map->NumFrames());
  server_proxy_->UploadMap(*map, place_matcher, root_id);
  ROS_DEBUG_NAMED("sparse_front_end::Server","Done uploading map");
}

template<typename FutureT>
inline bool future_ready(const FutureT& future) {
  static const std::chrono::milliseconds wait_time(0);
  return (!future.valid() ||
          (future.wait_for(wait_time) == std::future_status::ready));
}

bool FrontEndServerInterface::IsServerQueryReady() const {
  try {
    double since_download = hal::Toc(last_download_time());
    return (server_proxy_ &&
            future_ready(server_query_future_) &&
            since_download > server_query_spread_);
  } catch(const std::future_error& fe) {
    ROS_ERROR("Server query threw an exception: %s", fe.what());
  } catch(...) {
    ROS_ERROR("Server query threw an unknown exception");
  }

  // It failed, but that means it's ready again
  return true;
}

bool FrontEndServerInterface::IsServerUploadReady() const {
  return (server_proxy_ &&
          (!server_upload_future_.valid() ||
           future_ready(server_upload_future_)));
}

void FrontEndServerInterface::configCallback(sparse_front_end::FrontEndServerConfig &config, uint32_t level)
{
  server_upload_map_size_ = config.server_upload_map_size;
  server_download_map_size_ = config.server_download_map_size;
  server_query_spread_ = config.server_query_spread;
}
}  // namespace rslam
}  // namespace sparse
