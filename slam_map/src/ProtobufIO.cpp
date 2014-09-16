#include <slam_map/ProtobufIO.h>
#include <slam_map/FrameObjectProtobufIO.h>

void pb::fill_message(const SessionId& id, SessionIdMsg* msg) {
  msg->set_uuid(rslam::uuid::uuid_begin(id.uuid),
                rslam::uuid::uuid_size(id.uuid));
}

void pb::fill_message(const ReferenceFrameId& id, ReferenceFrameIdMsg* msg) {
  msg->set_frame_id(id.id);
  pb::fill_message(id.session_id, msg->mutable_session_id());
}

void pb::fill_message(const TransformEdgeId& id, TransformEdgeIdMsg* msg) {
  msg->set_edge_id(id.id);
  pb::fill_message(id.session_id, msg->mutable_session_id());
  pb::fill_message(id.start, msg->mutable_start());
  pb::fill_message(id.end, msg->mutable_end());
}

void pb::fill_message(const LandmarkId& id, LandmarkIdMsg* msg) {
  msg->set_landmark_id(id.landmark_index);
  pb::fill_message(id.ref_frame_id, msg->mutable_ref_frame_id());
}

void pb::fill_message(const MeasurementId& id, MeasurementIdMsg* msg) {
  pb::fill_message(id.frame_id, msg->mutable_ref_frame_id());
  pb::fill_message(id.landmark_id, msg->mutable_landmark_id());
}

void pb::fill_message(const Landmark& landmark, LandmarkMsg* msg) {
  pb::fill_message(landmark.id(), msg->mutable_id());

  msg->set_base_camera(landmark.base_camera());
  msg->set_landmark_state(landmark.state());
  msg->set_extent(landmark.extent());
  msg->set_active(landmark.is_active());

  pb::fill_message(landmark.orientation().matrix(), msg->mutable_orientation());

  pb::fill_message(landmark.patch_vector(), msg->mutable_patch());
  msg->set_feature_descriptor(landmark.descriptor(), landmark.desc_size());

  pb::fill_message(landmark.xrp(), msg->mutable_relative_pos());

  for (const MeasurementId& id : landmark.GetFeatureTrackRef()) {
    pb::fill_message(id, msg->add_feature_track());
  }
}

void pb::fill_message(const MultiViewMeasurement& z,
                      MultiViewMeasurementMsg* msg) {
  msg->clear_measurements();
  for (size_t i = 0; i < z.NumCameras(); ++i) {
    MeasurementMsg* z_msg = msg->add_measurements();
    pb::fill_message(z.id(), z_msg->mutable_id());

    z_msg->mutable_pixel()->set_u(z.Pixel(i)[0]);
    z_msg->mutable_pixel()->set_v(z.Pixel(i)[1]);
    z_msg->set_match_flag(z.Flag(i));
    z_msg->set_matching_error(z.MatchingError(i));
    z_msg->set_reprojection_error(z.ReprojectionError(i));
    z_msg->set_scale(z.Scale(i));
    z_msg->set_orientation(z.Orientation(i));

    pb::fill_message(z.GetPatchHomography(i),
                     z_msg->mutable_patch_homography());
    pb::fill_message(z.PatchVector(i), z_msg->mutable_patch_vector());
  }
}

void pb::fill_message(const ReferenceFrame& frame, ReferenceFrameMsg* msg) {
  pb::fill_message(frame.id(), msg->mutable_id());
  pb::fill_message(frame.parent_edge_id(), msg->mutable_parent_edge_id());

  for (unsigned int i = 0; i < frame.NumNeighbors(); ++i) {
    pb::fill_message(frame.GetNeighborEdgeId(i), msg->add_neighbor_edge_ids());
  }
  msg->set_sensor_time(frame.time());
  msg->set_is_isolated(frame.is_isolated());

  pb::fill_message(frame.v_r(), msg->mutable_velocity());

  msg->clear_landmarks();
  Landmark lm;
  for (size_t i = 0; i < frame.NumLandmarks(); ++i) {
    if (!frame.GetLandmark(i, &lm)) continue;
    pb::fill_message(lm, msg->add_landmarks());
  }

  msg->clear_measurements();
  MultiViewMeasurement z;
  for (size_t i = 0; i < frame.NumMeasurements(); ++i) {
    frame.GetMeasurement(i, &z);
    pb::fill_message(z, msg->add_measurements());
  }

  std::shared_ptr<FrameObject> object;
  for (size_t i = 0; i < frame.NumObjects(); ++i) {
    frame.GetObject(i, &object);
    fill_message(object, msg->add_objects());
  }
  msg->set_last_modified_time(frame.last_modified_time());
  pb::fill_message(frame.g_r(), msg->mutable_gravity());
  pb::fill_message(frame.b(), msg->mutable_biases());
}

void pb::fill_message(const TransformEdge& edge, TransformEdgeMsg* msg) {
  pb::fill_message(edge.id(), msg->mutable_id());

  Sophus::SE3t Tab;
  edge.transform(edge.start_id(), edge.end_id(), Tab);
  msg->set_last_modified_time(edge.last_modified_time());
  pb::fill_message(Tab.matrix(), msg->mutable_transform());

  pb::fill_message(edge.g(), msg->mutable_g());
  msg->set_is_broken(edge.is_broken());
  msg->set_is_loop_closure(edge.is_loop_closure());
}

void pb::fill_message(
    const calibu::CameraModelAndTransformT<Scalar>& model_transform,
    CameraModelMsg* msg) {
  WritePoseSE3(model_transform.T_wc, msg->mutable_pose_local_camera());

  const auto& cam = model_transform.camera;
  msg->set_type(model_transform.camera.Type());
  msg->set_version(cam.Version());
  msg->set_serial_number(cam.SerialNumber());
  msg->set_index(cam.Index());
  msg->set_width(cam.Width());
  msg->set_height(cam.Height());
  WriteVector(cam.GenericParams().cast<double>(), msg->mutable_params());
}

void pb::fill_message(const calibu::CameraRigT<Scalar>& rig,
                      CameraRigMsg* msg) {
  for (const calibu::CameraModelAndTransformT<Scalar>& cam : rig.cameras) {
    pb::fill_message(cam, msg->add_cameras());
  }
}

void pb::parse_message(const SessionIdMsg& msg, SessionId* id) {
  std::copy(msg.uuid().begin(), msg.uuid().end(),
            rslam::uuid::uuid_begin(id->uuid));
}

void pb::parse_message(const ReferenceFrameIdMsg& msg, ReferenceFrameId* id) {
  id->id = msg.frame_id();
  pb::parse_message(msg.session_id(), &id->session_id);
}

void pb::parse_message(const TransformEdgeIdMsg& msg, TransformEdgeId* id) {
  id->id = msg.edge_id();
  pb::parse_message(msg.session_id(), &id->session_id);
  pb::parse_message(msg.start(), &id->start);
  pb::parse_message(msg.end(), &id->end);
}

void pb::parse_message(const LandmarkIdMsg& msg, LandmarkId* id) {
  id->landmark_index = msg.landmark_id();
  pb::parse_message(msg.ref_frame_id(), &id->ref_frame_id);
}

void pb::parse_message(const MeasurementIdMsg& msg, MeasurementId* id) {
  pb::parse_message(msg.ref_frame_id(), &id->frame_id);
  pb::parse_message(msg.landmark_id(), &id->landmark_id);
}

void pb::parse_message(const MultiViewMeasurementMsg& msg,
                       MultiViewMeasurement* z) {
  // If the size isn't correct, we need to recreate it
  if ((size_t)msg.measurements_size() != z->NumCameras()) {
    *z = MultiViewMeasurement(msg.measurements_size());
  }

  for (int i = 0; i < msg.measurements_size(); ++i) {
    const MeasurementMsg& mm = msg.measurements(i);

    MeasurementId zId;
    pb::parse_message(mm.id(), &zId);
    z->set_id(zId);

    z->SetPixelInCam(i, mm.pixel().u(), mm.pixel().v());
    z->SetFlag(i, (MatchFlag)mm.match_flag());
    z->SetMatchingError(i, mm.matching_error());
    z->SetReprojectionError(i, mm.reprojection_error());
    z->SetScale(i, mm.scale());
    z->SetOrientation(i, mm.orientation());

    PatchHomography<CANONICAL_PATCH_SIZE> patch;
    pb::parse_message(mm.patch_homography(), &patch);
    z->SetPatchHomography(i, patch);

    parse_message(mm.patch_vector(), &z->PatchVector(i));
  }
}

void pb::parse_message(const LandmarkMsg& msg, Landmark* landmark) {
  LandmarkId landmark_id;
  pb::parse_message(msg.id(), &landmark_id);

  Sophus::Matrix<Scalar, 3, 3> orientation;
  pb::parse_message(msg.orientation(), &orientation);

  MeasurementId mId;
  pb::parse_message(msg.feature_track(0), &mId);

  Eigen::Matrix<Scalar, 4, 1> relativePos;
  pb::parse_message(msg.relative_pos(), &relativePos);

  Landmark::PatchVectorT patch;
  pb::parse_message(msg.patch(), &patch);
  landmark->Init(mId,
                 msg.active(),
                 landmark_id,
                 msg.base_camera(),
                 relativePos,
                 Sophus::SO3t(orientation),
                 msg.extent(),
                 (const unsigned char*)msg.feature_descriptor().data(),
                 msg.feature_descriptor().size(),
                 patch);

  landmark->set_state((LandmarkState)msg.landmark_state());

  for (int i = 1; i < msg.feature_track_size(); ++i) {
    MeasurementId mId;
    pb::parse_message(msg.feature_track(i), &mId);
    landmark->AddToFeatureTrack(mId);
  }
}

void pb::parse_message(const ReferenceFrameMsg& msg, ReferenceFrame* frame) {
  frame->Clear();

  ReferenceFrameId id;
  pb::parse_message(msg.id(), &id);
  frame->set_id(id);

  TransformEdgeId parent_id;
  pb::parse_message(msg.parent_edge_id(), &parent_id);
  frame->set_parent_edge_id(parent_id);

  Eigen::Matrix<Scalar, 3, 1> V;
  pb::parse_message(msg.velocity(), &V);
  frame->set_v_r(V);

  TransformEdgeId edge_id;
  for (const auto& id_msg : msg.neighbor_edge_ids()) {
    pb::parse_message(id_msg, &edge_id);
    frame->AddNeighbor(edge_id);
  }

  frame->set_time(msg.sensor_time());
  if (!msg.is_isolated()) {
    frame->set_linked();
  }

  for (const MultiViewMeasurementMsg& z_msg : msg.measurements()) {
    MultiViewMeasurement mvm(z_msg.measurements_size());
    pb::parse_message(z_msg, &mvm);
    frame->AddMeasurement(mvm);
  }

  for (const LandmarkMsg& lMsg : msg.landmarks()) {
    Landmark lm;
    pb::parse_message(lMsg, &lm);
    frame->AddLandmark(lm);
  }

  for (const FrameObjectMsg& obj_msg : msg.objects()) {
    std::shared_ptr<FrameObject> object;
    parse_message(obj_msg, &object);
    frame->AddObject(object);
  }

  frame->set_last_modified_time(msg.last_modified_time());

  Eigen::Vector3t g_r;
  Eigen::Vector6t b;
  pb::parse_message(msg.gravity(), &g_r);
  pb::parse_message(msg.biases(), &b);
  frame->set_g_r(g_r);
  frame->set_b(b);
}

void pb::parse_message(const TransformEdgeMsg& msg, TransformEdge* edge) {
  TransformEdgeId edge_id;
  pb::parse_message(msg.id(), &edge_id);
  edge->set_id(edge_id);

  typename Sophus::SE3t::Transformation mat;
  pb::parse_message(msg.transform(), &mat);
  edge->set_transform(Sophus::SE3t(mat));
  edge->set_last_modified_time(msg.last_modified_time());

  Eigen::Vector3t g;
  pb::parse_message(msg.g(), &g);
  edge->set_g(g);
  edge->set_is_broken(msg.is_broken());
  edge->set_is_loop_closure(msg.is_loop_closure());
}

void pb::parse_message(const CameraModelMsg& msg,
                       calibu::CameraModelAndTransformT<Scalar>* model) {
  calibu::CameraModelGeneric<Scalar> cam(msg.type());
  cam.SetVersion(msg.version());
  cam.SetSerialNumber(msg.serial_number());
  cam.SetIndex(msg.index());
  cam.SetImageDimensions(msg.width(), msg.height());

  Eigen::VectorXd params;
  ReadVector(msg.params(), &params);
  cam.SetGenericParams(params.cast<Scalar>());

  Sophus::SE3t pose;
  pb::ReadPoseSE3(msg.pose_local_camera(), &pose);
  *model = calibu::CameraModelAndTransformT<Scalar>(cam, pose);
}

void pb::parse_message(const CameraRigMsg& msg,
                       calibu::CameraRigT<Scalar>* rig) {
  calibu::CameraModelAndTransformT<Scalar> cam;

  Sophus::SE3t pose;
  for (const CameraModelMsg& model : msg.cameras()) {
    pb::parse_message(model, &cam);
    rig->Add(cam);
  }
}
