#include <rslam_engine/Gui.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/MapVisitor/GatherLandmarksMapVisitor.h>

bool Gui::Update(const std::shared_ptr<pb::ImageArray> &frames, const ReferenceFrameId& id, std::vector<std::vector<cv::KeyPoint> >&current_keypoints)
{
 
  std::unique_lock<std::mutex> lock = lock_gui();

  if(!lock)
  {
    ROS_ERROR("Can't update Gui");
    return false;
  }
  
  // Reference frame
  current_frame_id_ = id;

  // Current images
  current_frames_.clear();
  if(frames)
  {
    for (int i = 0; i < frames->Size(); ++i) {
      current_frames_.push_back(frames->at(i)->Mat().clone());
    }
  }

  SlamFramePtr current_frame = map_->GetFramePtr(current_frame_id_);
  if (!current_frame) {
    return false;
  }

  size_t num_meas = current_frame->NumMeasurements();
  ROS_ERROR_COND(num_meas==0, "No measurements in frame");
  std::vector<MultiViewMeasurement> cur_meas(
      num_meas, MultiViewMeasurement(rig_.cameras.size()));
  for (size_t zi = 0; zi < num_meas; ++zi) {
    current_frame->GetMeasurement(zi, &cur_meas[zi]);
  }

  current_measurements_.swap(cur_meas);
  current_keypoints_.swap(current_keypoints);

  // Update track data structure
  TrackInfoPtr pTrackInfo;
  
  if(track_info_.size() < num_visible_time_steps_)
  {
    pTrackInfo = TrackInfoPtr(new TrackInfo());
  }
  else
  {
    pTrackInfo = track_info_.back();
    track_info_.pop_back();
  }
  track_info_.push_front(pTrackInfo);
  pTrackInfo->Update(current_measurements_, current_keypoints_, current_frame_id_);

  if(!init_)
    init_=true;

  return true;
}

void Gui::set_map(std::shared_ptr<SlamMapProxy> p) {
  map_ = p;
  global_view_.reset(new GlobalMapView(map_));
}

void Gui::SetCameraRig(const calibu::CameraRigT<Scalar>& rig) {
    rig_ = rig;
}

std::unique_lock<std::mutex> Gui::lock_gui() const {
  return std::unique_lock<std::mutex>(mutex_);
}

void Gui::GetDisplayImage(cv::Mat &image)
{
  cv::Mat out_left, out_right;
  std::unique_lock<std::mutex> lock = lock_gui();

  if(!lock)
    return;
  
  cv::drawKeypoints(current_frames_[0], current_keypoints_[0], out_left);
  cv::drawKeypoints(current_frames_[1], current_keypoints_[1], out_right);
  cv::hconcat(out_left, out_right, image);
  // Draw correspondences
  for(auto z : current_measurements_)
  {
    if(z.HasGoodMeasurement())
    {
      Eigen::MatrixXd p1 = z.Pixel(0);
      Eigen::MatrixXd p2 = z.Pixel(1);

      line(image, cv::Point(p1(0),p1(1)), cv::Point(p2(0)+current_frames_[0].size().width,p2(1)),cv::Scalar(0,255,0.,1.0));
    }
  }

  /* Draw landmark tracks */
  for(size_t row=0; row < track_info_.size(); ++row) {
    for(MultiViewMeasurement& z: track_info_[row]->measurements_){
        Landmark landmark;
        if(!map_->GetLandmark(z.id().landmark_id, &landmark)){
          ROS_INFO("Can't find landmark");
          continue;
        }
        const std::vector<MeasurementId>& msr_ids = landmark.GetFeatureTrackRef();

        MultiViewMeasurement z1,z2;
        int cam_id = 0;
        for (unsigned int ii=0; ii < msr_ids.size()-1; ++ii) {
          map_->GetMeasurement(msr_ids[ii], z1);
          map_->GetMeasurement(msr_ids[ii+1], z2);
          if (z1.HasGoodMeasurementInCam(cam_id) && z2.HasGoodMeasurementInCam(cam_id)){
            const Eigen::Vector2t& p1 = z1.Pixel(cam_id);
            const Eigen::Vector2t& p2 = z2.Pixel(cam_id);
            line(image, cv::Point(p1(0),p1(1)), cv::Point(p2(0),p2(1)),cv::Scalar(255,0.,0.,1.0));
          }
        }
    }

  }
}

bool Gui::GetLandmarks(visualization_msgs::Marker &landmark_msg)
{
  int min_observations = 4;
  landmark_msg.header.stamp = ros::Time::now();
  landmark_msg.header.frame_id = "/map";
  landmark_msg.action = visualization_msgs::Marker::ADD;
  landmark_msg.type = visualization_msgs::Marker::POINTS;

  landmark_msg.id = 0;
  landmark_msg.color.r = 1.0;
  landmark_msg.color.g = 1.0;
  landmark_msg.color.b = 1.0;
  landmark_msg.color.a = 1.0;
  landmark_msg.scale.x = 0.1;
  landmark_msg.scale.y = 0.1;
    
  std::vector<Landmark> landmarks;
  if(only_draw_active_)
  {
    SlamFramePtr current_frame = map_->GetFramePtr(current_frame_id_);
    if(!current_frame)
      return false;

    MeasurementId zid;
    for (size_t i = 0; i < current_frame->NumMeasurements(); ++i) {
      if (!current_frame->GetMeasurementId(i, &zid)) continue;
        landmarks.emplace_back();
        map_->GetLandmark(zid.landmark_id, &landmarks.back());
    }
  } else if(num_poses_to_show_ > 0 && current_frame_id_.valid())
  {
    GatherLandmarksMapVisitor visitor(&landmarks);
    visitor.set_root_id(current_frame_id_);
    visitor.set_depth(num_poses_to_show_);
    map_->BFS(&visitor);
  }

  if(landmarks.empty()) 
    return false;

  // Rotation? or draw in base_link frame ...
  Sophus::SE3t Twp;
  for(size_t ii=0; ii < landmarks.size(); ++ii) {
    const Landmark& lm = landmarks[ii];
    const LandmarkId& id = lm.id();
    if (lm.FeatureTrackLength() < min_observations ||
        lm.state() == eLmkAtInfinity ||
        lm.state() == eLmkNotReliable) {
      continue;
    }

    if (!global_view_->GetFramePose(id.ref_frame_id, &Twp)) continue;

    const Eigen::Vector4t& Xrp =  lm.xrp();
    Eigen::Vector4t Xw = Sophus::MultHomogeneous(Twp, Xrp);
    Eigen::Vector4t Xs = Sophus::MultHomogeneous( rig_.cameras[lm.base_camera()].T_wc.inverse(), Xrp);
    Xw.head(3) /= Xw(3);

    geometry_msgs::Point p;
    p.x = Xw(0);
    p.y = Xw(1);
    p.z = Xw(2);
    landmark_msg.points.push_back(p);
  }
  return true;
}

bool Gui::GetMap(visualization_msgs::Marker &map)
{
  // Display landmark points and edges in the graph

  map.header.stamp = ros::Time::now();
  map.header.frame_id = "/map";
  map.action = visualization_msgs::Marker::ADD;
  //map.type = visualization_msgs::Marker::LINE_STRIP;
  map.type = visualization_msgs::Marker::LINE_LIST;

  map.id = 0;
  map.color.r = 1.0;
  map.color.a = 1.0;
  map.scale.x = 0.2;
  
  // Get edges from slam map 
  global_view_->LinesForGL(&edge_ids_, &line_points_, &maps_, &num_maps_);

  // Set up colors for each edge
  static const Eigen::Vector4f base_colors[6] =
      {{1.0, 1.0, 0.0, 0.5},  // Yellow
       {0.8, 0.8, 0.8, 0.2},  // Gray
       {0.5, 1.0, 1.0, 1.0},  // Cyan
       {1.0, 0.5, 1.0, 1.0},  // Pink
       {1.0, 0.0, 0.0, 1.0},  // Red
       {0.93, 0.3, 0.0, 1.0}  // Orange
      };

  Eigen::Vector4d rgb; 
  std::vector<float> edge_colors;
  int path_color_size = 4;
  edge_colors.reserve(edge_ids_.size() * 2 * path_color_size);
  int cindex = 0; 
  SlamEdgePtr edge;

  for (size_t i = 0; i < edge_ids_.size(); ++i) {
    const TransformEdgeId& edge_id = edge_ids_[i];
    const uint32_t attrib = map_->GetEdgeAttribute(edge_id);
    if (attrib & EdgeAttrib_IsBeingOptimized) {
      cindex = 2;
    } else if (attrib & EdgeAttrib_AsyncIsBeingOptimized) {
      cindex = 3;
    } else if (attrib & EdgeAttrib_Broken) {
      cindex = 4;
    } else if (num_maps_ == 1) {
      cindex = (map_->IsFrameLoaded(edge_id.start) &&
                map_->IsFrameLoaded(edge_id.end)) ? 0 : 1;
      if (cindex == 0 &&
          (edge = map_->GetEdgePtr(edge_id)) &&
          edge->is_loop_closure()) {
        cindex = 5;
      }
    }/* else { 
      // If we've got more than 1 map, color them differently.
      Eigen::Vector3d hsv(1./num_maps_ * maps_[i], 1., 1.);
      hsv2rgb(hsv, &rgb);
      if (!map_->IsFrameLoaded(edge_id.start) ||
          !map_->IsFrameLoaded(edge_id.end)) {
        rgb[3] = 0.1;
      }
      for (int i = 0; i < 2; ++i) {
        edge_colors.insert(edge_colors.end(), &rgb[0], &rgb[path_color_size]);
      }
      
      continue;
    }*/   
    // THIS IS BROKEN !!!
    for (int j = 0; j < 2; ++j) {
      edge_colors.insert(edge_colors.end(),
                         &(base_colors[cindex][0]),
                         &(base_colors[cindex][path_color_size-1])+1);
    }
  }

  // Now draw
  for (size_t i = 0; i < edge_ids_.size(); ++i) {
    geometry_msgs::Point p;
    p.x = line_points_[2*i](0);
    p.y = line_points_[2*i](1);
    map.points.push_back(p);
    p.x = line_points_[2*i+1](0);
    p.y = line_points_[2*i+1](1);
    map.points.push_back(p);
    
    std_msgs::ColorRGBA c;
    c.r = edge_colors[4*i+0];
    c.g = edge_colors[4*i+1];
    c.b = edge_colors[4*i+2];
    c.a = edge_colors[4*i+3];
    map.colors.push_back(c);
    map.colors.push_back(c);
  }
  return true;

  /*if (!line_points_.empty()) {
    for (size_t i = 0; i < line_points_.size(); i++) {
      const TransformEdgeId& edge_id = edge_ids_[i];
      edge = map_->GetEdgePtr(edge_id);
      if(edge && edge->is_loop_closure())
        continue;
      geometry_msgs::Point p;
      p.x = line_points_[i](0);
      p.y = line_points_[i](1);
      ROS_INFO("x: %f, y: %f",p.x, p.y);
      map.points.push_back(p);
    }
    return true;
  }
  else
    return false;*/
}
/*bool Gui::current_t_wp(Sophus::SE3t* out) const {
  bool success = false;
  if ((success = global_view_->GetFramePose(current_frame_id_, out))) {
    *out = initial_pose_ * (*out);
  }
  return success;
}*/















