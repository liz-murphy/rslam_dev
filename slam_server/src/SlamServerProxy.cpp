// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#include <slam_server/SlamServerProxy.h>

#include <thread>
#include <limits>
#include <map>
#include <queue>
#include <vector>

#include <utils/TicToc.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
//#include <PlaceMatching/DBoWMatcher/DBoWMatcher.h>
#include <slam_map/ProtobufIO.h>
#include <slam_map/SlamMap.h>
#include <slam_server/LoadPlaceMap.h>
#include <slam_server/ServerCVars.h>
#include <slam_server/ServerFetchMapVisitor.h>
#include <slam_server/SlamServerInterface.h>
#include <utils/PrintMessage.h>

SlamServerProxy::SlamServerProxy(
    const std::shared_ptr<SlamServerInterface>& server)
    : server_(server),
      last_upload_time_(0.),
      last_fetch_time_(0.) {
}

SlamServerProxy::~SlamServerProxy() {}

void SlamServerProxy::UploadMap(
    const SlamMap& map,
    const std::shared_ptr<PlaceMatcher>& place_matcher,
    const ReferenceFrameId& root_id) {
  LOG(g_server_cvars.debug_level) << "Uploading map from " << root_id
                                  << " since " << last_upload_time_;

  double upload_time = hal::Tic();
  ServerFetchMapVisitor visitor(server_, &map, nullptr);

  if (TemplateMatcher* tm =
      dynamic_cast<TemplateMatcher*>(place_matcher.get())) {
    visitor.set_template_matcher(tm);
  } else if (DBoWMatcher* dm =
             dynamic_cast<DBoWMatcher*>(place_matcher.get())) {
    visitor.set_dbow_matcher(dm);
  } else if (MultiDBoWMatcher* mdm =
             dynamic_cast<MultiDBoWMatcher*>(place_matcher.get())) {
    visitor.set_multidbow_matcher(mdm);
  } else {
    LOG(FATAL) << "Unknown template matcher in server proxy";
  }
  visitor.set_last_map_fetch_time(last_upload_time_);
  visitor.set_root_id(root_id);
  visitor.set_depth(MapVisitor::kMaxDepth);
  map.BFS(&visitor);
  last_upload_time_ = upload_time;
}

bool
SlamServerProxy::QueryPlace(const std::vector<cv::Mat>& images,
                            const CameraRigPtr& rig,
                            const ReferenceFrame& frame,
                            SlamEdgePtr* edge,
                            std::vector<MultiViewMeasurement>* meas) const {
  pb::ServerQueryRequest request;
  for (const cv::Mat& img : images) {
    pb::ReadCvMat(img, request.mutable_images()->add_image());
  }
  pb::fill_message(*rig, request.mutable_rig());
  pb::fill_message(frame, request.mutable_frame());

  pb::ServerQueryResponse response;
  server_->QueryPlace(request, &response);

  if (response.success()) {
    edge->reset(new TransformEdge);
    pb::parse_message(response.edge(), edge->get());

    meas->clear();
    for (const pb::MultiViewMeasurementMsg& z : response.measurements()) {
      meas->emplace_back();
      pb::parse_message(z, &meas->back());
    }

    LOG(g_server_cvars.debug_level)
        << "Query successful with edge " << (*edge)->id()
        << " and " << meas->size() << " new measurements";
    return true;
  }

  return false;
}

void SlamServerProxy::FetchToDepth(
    const ReferenceFrameId& id,
    size_t depth,
    const SessionId* const excluding,
    SlamMap* map,
    PlaceMatcher* matcher) const {
  pb::ServerDownloadRequest request;
  pb::ServerDownloadResponse response;
  size_t num_fetched = 0;

  std::queue<ReferenceFrameId> to_fetch;
  to_fetch.push(id);

  do {
    response.Clear();
    request.Clear();

    pb::fill_message(to_fetch.front(), request.mutable_frame_id());
    request.set_depth(depth);
    if (excluding) {
      pb::fill_message(*excluding, request.mutable_excluding_map());
    }
    LOG(g_server_cvars.debug_level) << "Fetching map to depth of " << depth
                                    << " from root " << to_fetch.front()
                                    << ", IsExcluding " << (!!excluding);
    to_fetch.pop();

    server_->DownloadMap(request, &response);

    if (!response.success()) {
      LOG(g_server_cvars.debug_level) << "Map download failed.\n";
      return;
    }

    last_fetch_time_ = response.timestamp();
    LOG(g_server_cvars.debug_level)
        << "Done fetching map. Loading." << std::endl;

    const pb::PlaceMapMsg& place_map = response.map();
    if (place_map.has_map()) {
      LOG(g_server_cvars.debug_level)
          << "Fetched map stats: "
          << "\n\t" << response.map().map().nodes_size() << " nodes, "
          << "\n\t" << response.map().map().edges_size() << " edges,  "
          << "\n\t" << response.map().leaf_size() << " leaves,  "
          << "\n\t" << response.map().dbow_places_size() << " dbow places, and"
          << "\n\t" << response.map().templates_size() << " templates.";
      num_fetched += response.map().map().nodes_size();

      LoadSlamMapMsg(place_map.map(), map, true);
      LOG(g_server_cvars.debug_level) << "Done loading SlamMap.";
    }

    LOG(g_server_cvars.debug_level) << " Loading places.";
    LoadPlacesFromMsg(place_map, matcher);

    ReferenceFrameId leaf_id;
    for (const pb::ReferenceFrameIdMsg& id_msg : response.map().leaf()) {
      pb::parse_message(id_msg, &leaf_id);
      if (map->GetFramePtr(leaf_id)) continue;
      to_fetch.push(leaf_id);
      LOG(g_server_cvars.debug_level) << "Adding leaf to download " << leaf_id;
    }

  } while (num_fetched < depth && !to_fetch.empty());

  LOG(g_server_cvars.debug_level)
      << "Done loading fetched map." << std::endl;
}

void SlamServerProxy::set_server_interface(
    const std::shared_ptr<SlamServerInterface>& server) {
  server_ = server;
}
