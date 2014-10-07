#include <rslam_engine/Gui.h>
#include <Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
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
  ROS_INFO("Frame has %d measurements", num_meas);
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

bool Gui::GetMap(visualization_msgs::Marker &map)
{
  // Display landmark points and edges in the graph

  map.header.stamp = ros::Time::now();
  map.header.frame_id = "/map";
  map.action = visualization_msgs::Marker::ADD;
  map.type = visualization_msgs::Marker::LINE_STRIP;

  map.color.r = 1.0;
  map.color.a = 1.0;

  // Edges
  global_view_->LinesForGL(&edge_ids_, &line_points_, &maps_, &num_maps_);

  if (!line_points_.empty()) {
    int step_size = line_points_.size() > 100 ? line_points_.size() / 50.0 : 2;
    for (size_t i = 0; i < line_points_.size(); i += step_size) {
      geometry_msgs::Point p;
      p.x = line_points_[i](0);
      p.y = line_points_[i](1);
      ROS_INFO("x: %f, y: %f",p.x, p.y);
      map.points.push_back(p);
    }
    return true;
  }
  else
    return false;
}
/*bool Gui::current_t_wp(Sophus::SE3t* out) const {
  bool success = false;
  if ((success = global_view_->GetFramePose(current_frame_id_, out))) {
    *out = initial_pose_ * (*out);
  }
  return success;
}*/















