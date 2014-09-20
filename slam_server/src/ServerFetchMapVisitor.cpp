// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <slam_server/ServerFetchMapVisitor.h>

#include <map>
#include <set>
#include <miniglog/logging.h>
#include <place_matching/DBoWMatcher/dbow_protobuf.h>
#include <pb_msgs/SlamServer.pb.h>
#include <slam_map/MapVisitor/MapVisitor.h>
#include <slam_map/ProtobufIO.h>
#include <slam_map/SlamMap.h>
#include <slam_server/ServerCVars.h>
#include <slam_server/SlamServerInterface.h>

ServerFetchMapVisitor::ServerFetchMapVisitor(
    const SlamMap* const map,
    const pb::SessionIdMsg* const excluding_id_msg)
    : map_(map),
      template_matcher_(nullptr),
      dbow_matcher_(nullptr),
      current_msg_(nullptr),
      excluding_id_(),
      has_excluding_id_(excluding_id_msg),
      last_map_fetch_time_(0.),
      finished_(false),
      num_frames_(0) {
  if (has_excluding_id_) {
    pb::parse_message(*excluding_id_msg, &excluding_id_);
  }

  set_has_explore_edge(true);
  set_has_visit(true);
}

ServerFetchMapVisitor::ServerFetchMapVisitor(
    const std::shared_ptr<SlamServerInterface>& server,
    const SlamMap* const map,
    const pb::SessionIdMsg* const excluding_id_msg)
    : ServerFetchMapVisitor(map, excluding_id_msg) {
  server_ = server;
  current_msg_ = new pb::PlaceMapMsg;
  CHECK(server.get());
}

ServerFetchMapVisitor::ServerFetchMapVisitor(
    const SlamMap* const map,
    const pb::SessionIdMsg* const excluding_id_msg,
    pb::PlaceMapMsg* out)
    : ServerFetchMapVisitor(map, excluding_id_msg) {
  CHECK(out);
  current_msg_ = out;
  UpdateMsgPlaceType();
}

ServerFetchMapVisitor::~ServerFetchMapVisitor() {
  if (server_) {
    delete current_msg_;
  }
}

void ServerFetchMapVisitor::CheckMessageSize() {
  if (num_frames_ % g_server_cvars.fetch_check_msg_size_skip != 0 ||
      !CurrentMessageTooLarge()) {
    return;
  }

  if (server_) {
    LOG(g_server_cvars.debug_level)
        << "Uploading map with " << current_msg_->map().nodes_size()
        << " nodes, " << current_msg_->map().edges_size() << " edges, "
        << current_msg_->dbow_places_size() << "dbow places and "
        << current_msg_->templates_size() << " templates.";
    server_->UploadMap(*current_msg_);
    current_msg_->Clear();
  } else {
    finished_ = true;
    for (const ReferenceFrameId& leaf : explored_) {
      pb::fill_message(leaf, current_msg_->add_leaf());
    }
  }
}

bool ServerFetchMapVisitor::CurrentMessageTooLarge() const {
  // Copied from coded_stream.h (it's a private value by default)
  static const int kDefaultTotalBytesLimit = 64 << 20;  // 64MB

  // To be conservative, we'll just say if the message is over half
  // the byte size limit, we'll start a new one
  return current_msg_->ByteSize() > kDefaultTotalBytesLimit / 2;
}

void ServerFetchMapVisitor::UpdateMsgPlaceType() {
  if (template_matcher_) {
    current_msg_->set_place_type(pb::TEMPLATE);
  } else if (dbow_matcher_) {
    current_msg_->set_place_type(pb::DBOW);
  } else if (multidbow_matcher_) {
    current_msg_->set_place_type(pb::MULTI_DBOW);
  }
}

void ServerFetchMapVisitor::ExploreEdge(const SlamFramePtr& parent,
                                        const SlamEdgePtr& edge,
                                        const SlamFramePtr& child) {
  if ((has_excluding_id_ && edge->id().session_id == excluding_id_) ||
      edge->last_modified_time() < last_map_fetch_time_) {
    LOG(g_server_cvars.debug_level)
        << "Ignoring edge " << edge->id()
        << " last modified at " << edge->last_modified_time()
        << " which was before last fetch at " << last_map_fetch_time_;
    return;
  }

  LOG(g_server_cvars.debug_level) << "Adding edge " << edge->id();
  pb::fill_message(*edge, current_msg_->mutable_map()->add_edges());

  explored_.insert(child->id());
  CheckMessageSize();
}

bool ServerFetchMapVisitor::Visit(const SlamFramePtr& cur_node) {
  explored_.erase(cur_node->id());
  if ((has_excluding_id_ && cur_node->id().session_id == excluding_id_)) {
    LOG(g_server_cvars.debug_level)
        << "Ignoring node " << cur_node->id()
        << " last modified at "
        << cur_node->last_modified_time()
        << " which was before last fetch at " << last_map_fetch_time_;
    return true;
  } else if (cur_node->last_modified_time() < last_map_fetch_time_) {
    LOG(g_server_cvars.debug_level)
        << "Node " << cur_node->id() << " modification time @ "
        << cur_node->last_modified_time() << " before last fetch @ "
        << last_map_fetch_time_;
    return false;
  }
  LOG(g_server_cvars.debug_level) << "Adding node " << cur_node->id();
  auto* node = current_msg_->mutable_map()->add_nodes();
  pb::fill_message(*cur_node, node);

  const SessionId& mid = cur_node->id().session_id;
  if (!session_ids_.count(mid)) {
    CameraRigPtr rig_ptr = map_->GetCamera(mid);

    // If we don't have a rig pointer for this, someone screwed up.
    LOG_IF(FATAL, !rig_ptr) << "Rig not found for ID " << mid;

    pb::fill_message(mid, current_msg_->mutable_map()->add_session_ids());
    pb::fill_message(*rig_ptr, current_msg_->mutable_map()->add_rigs());
    session_ids_.insert(mid);
  }

  ReferenceFrameId frame_id = cur_node->id();

  if (template_matcher_) {
    cv::Mat img;
    if (template_matcher_->GetPlace(frame_id, &img)) {
      pb::TemplateMsg* place_msg = current_msg_->add_templates();
      pb::fill_message(frame_id, place_msg->mutable_id());
      pb::ReadCvMat(img, place_msg->mutable_image());
    }
  } else if (dbow_matcher_) {
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors;
    unsigned int place_id = dbow_matcher_->
        GetExistingInternalPlaceId(frame_id);
    dbow_matcher_->GetPlace(place_id, keypoints, descriptors);

    pb::DBoWPlaceMsg* msg = current_msg_->add_dbow_places();
    pb::fill_message(frame_id, msg->mutable_index());
    pb::fill_message(keypoints, msg->mutable_keypoint_vector());
    pb::fill_message(descriptors, msg->mutable_descriptor_vector());

  } else if (multidbow_matcher_) {
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Mat> descriptors;
    std::vector<LandmarkId> landmarks;
    multidbow_matcher_->GetPlace(frame_id, keypoints, descriptors,
                                 &landmarks);

    // we reuse the dbow structure
    pb::DBoWPlaceMsg* msg = current_msg_->add_dbow_places();
    pb::fill_message(keypoints, msg->mutable_keypoint_vector());
    pb::fill_message(descriptors, msg->mutable_descriptor_vector());
    pb::fill_message(frame_id, msg->mutable_index());
    pb::fill_message(landmarks, msg->mutable_landmarks_vector());
  }
  ++num_frames_;
  return true;
}

void ServerFetchMapVisitor::Finished() {
  if (server_ && current_msg_) {
    LOG(g_server_cvars.debug_level)
        << "Uploading map with " << current_msg_->map().nodes_size()
        << " nodes, " << current_msg_->map().edges_size() << " edges, "
        << current_msg_->dbow_places_size() << "dbow places and "
        << current_msg_->templates_size() << " templates.";
    server_->UploadMap(*current_msg_);
    current_msg_->Clear();
  } else if (!finished_) {
    finished_ = true;
    for (const ReferenceFrameId& leaf : explored_) {
      pb::fill_message(leaf, current_msg_->add_leaf());
    }
  }
}
