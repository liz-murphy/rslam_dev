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
//#include <sparse_FrontEnd/FrontEndCVars.h>
#include <sparse_front_end/FrontEndConfig.h>

namespace rslam {
namespace sparse {

FrontEndServerInterface::~FrontEndServerInterface() {
  int n_tries = 0;
  while (!IsServerQueryReady() && n_tries++ < 10) {
    LOG(INFO) << "Waiting for server interface to finish";
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
    LOG(WARNING) << "Querying server with a frame which is not ours: "
                 << id;
    return false;
  }

  CameraRigPtr rig = map->GetCamera(id.session_id);
  SlamFramePtr frame = map->GetFramePtr(id);
  if (!rig) {
    LOG(FrontEndConfig::getConfig()->server_debug_level)
        << "No rig available for frame " << id
        << "while querying server";
    return false;
  } else if (!frame) {
    LOG(FrontEndConfig::getConfig()->server_debug_level)
        << "No frame available for frame " << id
        << "while querying server";
    return false;
  }

  SlamEdgePtr edge;
  std::vector<MultiViewMeasurement> measurements;
  if (!server_proxy_->QueryPlace({image}, rig, *frame, &edge, &measurements)) {
    return false;
  }

  LOG(FrontEndConfig::getConfig()->server_debug_level)
      << "Match found on server for frame " << id
      << "! Fetching map to depth of "
      << FrontEndConfig::getConfig()->server_download_map_size;

  TransformEdgeId edge_id = edge->id();
  ReferenceFrameId outgoing_frame_id =
      (edge_id.start == id) ? edge_id.end : edge_id.start;
  SessionId current_session_id = map->id();

  LOG(FrontEndConfig::getConfig()->server_debug_level)
      << "After query, adding edge " << edge_id;
  map->AddEdge(edge);

  for (MultiViewMeasurement& z : measurements) {
    frame->AddMeasurement(z);
  }

  last_download_time_ = hal::Tic();
  server_proxy_->FetchToDepth(
      outgoing_frame_id, FrontEndConfig::getConfig()->server_download_map_size,
      &current_session_id, map.get(),
      place_matcher.get());
  return true;
}

void FrontEndServerInterface::AsyncUploadMapToServer(
    const std::shared_ptr<SlamMap>& map,
    const std::shared_ptr<PlaceMatcher>& place_matcher,
    const ReferenceFrameId& root_id) {
  if ((map->NumFrames() % FrontEndConfig::getConfig()->server_upload_map_size) != 0 ||
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
  LOG(FrontEndConfig::getConfig()->server_debug_level)
      << "Uploading map with " << map->NumFrames() << " frames.";
  server_proxy_->UploadMap(*map, place_matcher, root_id);
  LOG(FrontEndConfig::getConfig()->server_debug_level) << "Done uploading map";
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
            since_download > FrontEndConfig::getConfig()->server_query_spread);
  } catch(const std::future_error& fe) {
    LOG(ERROR) << "Server query threw an exception: " << fe.what();
  } catch(...) {
    LOG(ERROR) << "Server query threw an unknown exception";
  }

  // It failed, but that means it's ready again
  return true;
}

bool FrontEndServerInterface::IsServerUploadReady() const {
  return (server_proxy_ &&
          (!server_upload_future_.valid() ||
           future_ready(server_upload_future_)));
}
}  // namespace rslam
}  // namespace sparse
