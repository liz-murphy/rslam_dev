// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <slam_server/LoadPlaceMap.h>

#include <map>
#include <pb_msgs/SlamServer.pb.h>
#include <pb_msgs/rslam.pb.h>
#include <pb_msgs/template.pb.h>
#include <pb_msgs/Image.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>
#include <place_matching/DBoWMatcher/dbow_protobuf.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <slam_map/ProtobufIO.h>
#include <slam_map/SlamMap.h>
//#include <slam_server/ServerCVars.h>
#include <slam_server/ServerConfig.h>
void LoadSlamMapMsg(const pb::SlamMapMsg& map_msg,
                    SlamMap* slam_map,
                    bool zero_modified_time) {

  LOG(ServerConfig::getConfig()->debug_level) << "Server: Loading rigs...";
  SessionId session_id;
  for (int i = 0; i < map_msg.session_ids_size(); ++i) {
    pb::parse_message(map_msg.session_ids(i), &session_id);
    _CameraRigPtr rig = std::make_shared<calibu::CameraRigT<Scalar> >();
    pb::parse_message(map_msg.rigs(i), rig.get());
    LOG(ServerConfig::getConfig()->debug_level) << "Loading rig for " << session_id;
    slam_map->AddCamera(session_id, rig);
  }

  LOG(ServerConfig::getConfig()->debug_level) << "Server: Loading nodes...";
  for (const pb::ReferenceFrameMsg& fmsg : map_msg.nodes()) {
    SlamFramePtr frame = std::make_shared<ReferenceFrame>();
    pb::parse_message(fmsg, frame.get());
    slam_map->AddFrame(frame);
    LOG(ServerConfig::getConfig()->debug_level) << "Loading " << frame->id();
    if (zero_modified_time) {
      frame->set_last_modified_time(0.0);
    }
  }

  LOG(ServerConfig::getConfig()->debug_level) << "Server: Loading edges...";
  for (const pb::TransformEdgeMsg& emsg : map_msg.edges()) {
    SlamEdgePtr edge = std::make_shared<TransformEdge>();
    pb::parse_message(emsg, edge.get());
    slam_map->AddEdge(edge);
    LOG(ServerConfig::getConfig()->debug_level) << "Loading " << edge->id();
    if (zero_modified_time) {
      edge->set_last_modified_time(0.0);
    }
  }
}

void LoadPlacesFromMsg(
    const pb::PlaceMapMsg& place_map,
    PlaceMatcher* matcher) {
  if (!place_map.has_place_type()) return;

  if (place_map.place_type() == pb::TEMPLATE) {
    TemplateMatcher* tm = dynamic_cast<TemplateMatcher*>(matcher);
    if (!tm) {
      LOG(ERROR) << "Message has incorrect place type: Template.";
      return;
    }
    LoadTemplateMsg(place_map, tm);
  } else if (place_map.place_type() == pb::DBOW) {
    DBoWMatcher* dbow = dynamic_cast<DBoWMatcher*>(matcher);
    if (!dbow) {
      LOG(ERROR) << "Message has incorrect place type: DBoW.";
      return;
    }
    LoadDBoWMsg(place_map, dbow);
  } else if (place_map.place_type() == pb::MULTI_DBOW) {
    MultiDBoWMatcher* multidbow = dynamic_cast<MultiDBoWMatcher*>(matcher);
    if (!multidbow) {
      LOG(ERROR) << "Message has incorrect place type: MultiDBoW.";
      return;
    }
    LoadMultiDBoWMsg(place_map, multidbow);
  }
}

void LoadTemplateMsg(const pb::PlaceMapMsg& place_map,
                     TemplateMatcher* matcher) {
  CHECK(matcher);
  CHECK_EQ(place_map.place_type(), pb::TEMPLATE);

  ReferenceFrameId frame_id;
  auto place = place_map.templates().begin();
  for(; place != place_map.templates().end() ; ++place) {
    cv::Mat img = pb::WriteCvMat(place->image()).clone();
    pb::parse_message(place->id(), &frame_id);
    matcher->AddTemplate(frame_id, img);
  }
}

void LoadDBoWMsg(
    const pb::PlaceMapMsg& place_map,
    DBoWMatcher* matcher) {
  CHECK(matcher);
  CHECK_EQ(place_map.place_type(), pb::DBOW);

  LOG(ServerConfig::getConfig()->debug_level) << "Loading DBoW places...";

  // @todo: check if this is correct
  // ReferenceFrameId frame_id;
  // std::vector<cv::KeyPoint> image_keys;
  // std::vector<cv::Mat> image_descriptors;

  // auto place = place_map.dbow_places().begin();
  // for(; place != place_map.dbow_places().end() ; ++place) {
  //   pb::parse_message(place->keypoint_vector(), &image_keys);
  //   pb::parse_message(place->descriptor_vector(), &image_descriptors);
  //   pb::parse_message(mapping->frame_id(), &frame_id);

  //   matcher->AddPlace(frame_id, image_keys, image_descriptors);
  // }

  /*
  DBoWMatcher::InternalData dbow_data;
  dbow_data.frame_indices.reserve(place_map.dbow_places_size());

  std::map<uint32_t, ReferenceFrameId> mappings;
  ReferenceFrameId frame_id;
  for (const pb::PlaceFrameMappingMsg& mapping : place_map.place_ids()) {
    pb::parse_message(mapping.frame_id(), &frame_id);
    mappings.insert({mapping.place_id(), frame_id});
  }

  std::vector<cv::KeyPoint> image_keys;
  std::vector<cv::Mat> image_descriptors;
  for (const pb::DBoWPlaceMsg& place : place_map.dbow_places()) {
    // Don't reinsert places we've already gathered
    if (frame_to_place &&
        frame_to_place->count(mappings[place.index()])) {
      LOG(ServerConfig::getConfig()->debug_level)
          << "Skipping " << mappings[place.index()]
          << " since we have a place for it already";
      continue;
    }
    uint32_t new_place_id = (place_to_frame->empty() ?
                             0 : place_to_frame->rbegin()->first) + 1;

    image_keys.emplace_back();
    image_descriptors.emplace_back();

    image_keys.clear();
    image_descriptors.clear();
    pb::parse_message(place.keypoint_vector(), &image_keys);
    pb::parse_message(place.descriptor_vector(), &image_descriptors);
    matcher->AddPlace(new_place_id, image_keys, image_descriptors);
    LOG(ServerConfig::getConfig()->debug_level) << "Adding DBoW place #" << new_place_id;

    ReferenceFrameId frame_id = mappings[place.index()];
    if (place_to_frame) {
      place_to_frame->insert({new_place_id, frame_id});
    }

    if (frame_to_place) {
      frame_to_place->insert({frame_id, new_place_id});
      LOG(ServerConfig::getConfig()->debug_level) << "Added " << frame_id
                                      << " as place #" << new_place_id;
    }
  }
  */
  LOG(ServerConfig::getConfig()->debug_level) << "Done loading DBoW places.";
}

void LoadMultiDBoWMsg(const pb::PlaceMapMsg& place_map,
                      MultiDBoWMatcher* matcher)
{
  CHECK(matcher);
  CHECK_EQ(place_map.place_type(), pb::MULTI_DBOW);

  LOG(ServerConfig::getConfig()->debug_level)
      << "Loading " << place_map.dbow_places_size() << " DBoW places...";

  ReferenceFrameId frame_id;
  std::vector<cv::KeyPoint> image_keys;
  std::vector<cv::Mat> image_descriptors;
  std::vector<LandmarkId> landmarks;
  for(auto place = place_map.dbow_places().begin();
      place != place_map.dbow_places().end(); ++place) {
    pb::parse_message(place->keypoint_vector(), &image_keys);
    pb::parse_message(place->descriptor_vector(), &image_descriptors);
    pb::parse_message(place->index(), &frame_id);
    pb::parse_message(place->landmarks_vector(), &landmarks);

    matcher->AddPlace(frame_id, image_keys, image_descriptors, landmarks);
  }
}
