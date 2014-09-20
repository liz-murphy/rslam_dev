// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <slam_server/SlamServer.h>

#include <string>
#include <vector>
#include <pb_msgs/SlamServer.pb.h>
#include <pb_msgs/rslam.pb.h>

#include <back_end/BackEnd.h>
//#include <common_front_end/CommonFrontEndCVars.h>
#include <common_front_end/TrackingStats.h>
#include <utils/TicToc.h>
#include <miniglog/logging.h>
#include <place_matching/PlaceMatcherFactory.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <slam_map/ProtobufIO.h>
#include <slam_map/SlamMap.h>
#include <slam_server/LoadPlaceMap.h>
//#include <slam_server/ServerCVars.h>
#include <slam_server/ServerFetchMapVisitor.h>
#include <sparse_tracking/EstimateRelativePose.h>
//#include <sparse_tracking/TrackingCVars.h>
#include <slam_server/ServerConfig.h>

static const std::string places_filename = "places.pb";
static const std::string map_filename = "SlamServer.db";

SlamServer::SlamServer(const std::shared_ptr<PlaceMatcher>& matcher,
                       const std::shared_ptr<SlamMap>& map)
    : map_(map), place_matcher_(matcher), backend_(new BackEnd) {
  // Always continue so we can restart without worrying
  map_->InitWithPersistence(map_filename, true);
  place_matcher_->Load(places_filename);
  backend_->Init(map_);
  PrintStats();
}

SlamServer::~SlamServer() {}

void SlamServer::PrintStats() const {
  LOG(INFO) << "**SlamServer Stats**";
  LOG(INFO) << "Map stats:";
  LOG(INFO) << "\t" << map_->NumFrames() << " frames";
  LOG(INFO) << "\t" << map_->NumEdges() << " edges";
  LOG(INFO) << "\t" << map_->NumCameras() << " cameras";
  LOG(INFO) << std::endl;
  LOG(INFO) << "PlaceMatcher stats:";
  LOG(INFO) << "\t" << place_matcher_->NumPlaces() << " places";
  LOG(INFO) << "****";
}

void SlamServer::UploadMap(const pb::PlaceMapMsg& place_map) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!place_map.has_map()) {
    LOG(ERROR) << "Uploaded map does not have a SlamMap.";
    return;
  } else if (place_map.map().session_ids_size() != place_map.map().rigs_size()) {
    LOG(ERROR) << "Uploaded map has unequal number of rigs and map ids";
    return;
  }

  LoadSlamMapMsg(place_map.map(), map_.get(), false);
  LoadPlacesFromMsg(place_map, place_matcher_.get());
  Save();
}

double SlamServer::FetchToDepth(const pb::ReferenceFrameIdMsg& id_msg,
                                size_t depth,
                                double last_fetch_time,
                                const pb::SessionIdMsg* const excluding,
                                pb::PlaceMapMsg* to_fetch) const {
  CHECK_NOTNULL(to_fetch);
  std::lock_guard<std::mutex> lock(mutex_);

  ReferenceFrameId id;
  pb::parse_message(id_msg, &id);

  LOG(ServerConfig::getConfig()->debug_level) << "Fetching map at " << id
                                  << " to depth of " << depth
                                  << " since " << last_fetch_time;


  ServerFetchMapVisitor visitor(map_.get(),
                                excluding,
                                to_fetch);

  if (UsingTemplateMatcher()) {
    visitor.set_template_matcher(
        dynamic_cast<TemplateMatcher*>(place_matcher_.get()));
  } else if (UsingDBoWMatcher()) {
    visitor.set_dbow_matcher(dynamic_cast<DBoWMatcher*>(place_matcher_.get()));
  } else if (UsingMultiDBoWMatcher()) {
    visitor.set_multidbow_matcher(dynamic_cast<MultiDBoWMatcher*>
                                  (place_matcher_.get()));
  }

  visitor.set_last_map_fetch_time(last_fetch_time);
  visitor.set_root_id(id);
  visitor.set_depth(depth);
  map_->BFS(&visitor);

  LOG(ServerConfig::getConfig()->debug_level)
      << "Fetched map stats: "
      << "\n\t" << to_fetch->map().nodes_size() << " nodes, "
      << "\n\t" << to_fetch->map().edges_size() << " edges,  "
      << "\n\t" << to_fetch->leaf_size() << " leaves,  "
      << "\n\t" << to_fetch->dbow_places_size() << " dbow places, and"
      << "\n\t" << to_fetch->templates_size() << " templates.";

  return hal::Tic();
}

bool SlamServer::UsingTemplateMatcher() const {
  return dynamic_cast<TemplateMatcher*>(place_matcher_.get());
}

bool SlamServer::UsingDBoWMatcher() const {
  return dynamic_cast<DBoWMatcher*>(place_matcher_.get());
}

bool SlamServer::UsingMultiDBoWMatcher() const {
  return dynamic_cast<MultiDBoWMatcher*>(place_matcher_.get());
}

struct RelocalizationCandidate {
  ReferenceFrameId frame;
  Scalar score;
  Sophus::SE3t t_ab;
  std::vector<MultiViewMeasurement> new_measurements;
};

bool SlamServer::QueryPlace(const pb::CameraMsg& cam,
                            const pb::CameraRigMsg& rig_msg,
                            const pb::ReferenceFrameMsg& frame_msg,
                            pb::TransformEdgeMsg* to_match,
                            std::vector<pb::MultiViewMeasurementMsg>* meas) {
  CHECK_NOTNULL(to_match);
  CHECK_NOTNULL(meas);
  std::lock_guard<std::mutex> lock(mutex_);

  // Find match
  std::vector<PlaceMatchCandidate> matches;

  SlamFramePtr query_frame(new ReferenceFrame);
  pb::parse_message(frame_msg, query_frame.get());
  map_->AddFrame(query_frame);

  _CameraRigPtr rig(new calibu::CameraRigT<Scalar>);
  pb::parse_message(rig_msg, rig.get());
  map_->AddCamera(query_frame->id().session_id, rig);

  cv::Mat query_image = WriteCvMat(cam.image(0));
  place_matcher_->GetPotentialPlaceMatchesOrAddPlace(
      query_frame->id(), query_image, matches);

  if (matches.size() != 1) return false;

  LOG(ServerConfig::getConfig()->debug_level)
      << "Found " << matches.size() << " match candidates";

  const PlaceMatchCandidate& match = matches[0];

  meas->clear();
  // For all the keypoints in the candidate
  MultiViewMeasurement z(1);
  MeasurementId zid(query_frame->id(), LandmarkId());
  LOG(ServerConfig::getConfig()->debug_level)
      << "Creating " << match.query.size() << " matching measurements";
  for (size_t i = 0; i < match.query.size(); ++i) {
    // Set its landmark_id, frame_id accordingly
    zid.landmark_id = match.match_landmarks[i];
    z.set_id(zid);

    // Set the pixel location from the keypoint
    z.SetPixelInCam(0, match.query[i].pt.x, match.query[i].pt.y);

    // Set Flag to GoodMatch
    z.SetFlag(0, GoodMatch);

    // Set patch homography with two keypoints and a scale of 1.0
    PatchHomography<CANONICAL_PATCH_SIZE> H(
        match.query[i].pt.x, match.query[i].pt.y, 1.0);
    z.SetPatchHomography(0, H);

    // Extract the patch from the image
    LoadWarpedPatch<CANONICAL_PATCH_SIZE>(
        query_image.data, query_image.cols, query_image.rows,
        H.matrix(), &z.PatchVector(0)[0]);
    meas->emplace_back();
    pb::fill_message(z, &meas->back());
  }
  LOG(ServerConfig::getConfig()->debug_level) << "Checking matches";

  SlamFramePtr matched_frame = map_->GetFramePtr(match.getFrameId());
  CHECK(matched_frame) << match.getFrameId();

  SlamEdgePtr edge = map_->AddEdge(query_frame, matched_frame, match.Tqm);
  pb::fill_message(*edge, to_match);
  return true;
}

void SlamServer::UploadMap(const pb::ServerUploadRequest& request,
                           pb::ServerUploadResponse* response) {
  UploadMap(request.map());
  response->set_success(true);
}

void SlamServer::QueryPlace(const pb::ServerQueryRequest& request,
                            pb::ServerQueryResponse* response) {
  pb::TransformEdgeMsg* edge = new pb::TransformEdgeMsg;
  std::vector<pb::MultiViewMeasurementMsg> meas;
  bool success = QueryPlace(
      request.images(), request.rig(), request.frame(), edge, &meas);
  response->set_success(success);
  for (const pb::MultiViewMeasurementMsg& z : meas) {
    *response->add_measurements() = z;
  }
  if (success) {
    response->set_allocated_edge(edge);
  } else {
    delete edge;
  }
}

void SlamServer::DownloadMap(const pb::ServerDownloadRequest& request,
                             pb::ServerDownloadResponse* response) const {

  pb::SessionIdMsg excluding;
  if (request.has_excluding_map()) {
    excluding = request.excluding_map();
  }
  double timestamp = FetchToDepth(
      request.frame_id(), request.depth(),
      request.has_last_download_time() ? request.last_download_time() : 0,
      request.has_excluding_map() ? &excluding : nullptr,
      response->mutable_map());
  response->set_timestamp(timestamp);
  response->set_success(true);
}

void SlamServer::Save() const {
  map_->Save();
  place_matcher_->Save(places_filename);
}

void SlamServer::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  map_->Clear();
  backend_.reset(new BackEnd);
  /** @todo Reset PlaceMatcher! */
}
