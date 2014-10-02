#include <rslam_engine/Gui.h>

bool Update(const std::shared_ptr<pb::ImageArray> &frames, const ReferenceFrameId& id, std::vector<std::vector<cv::KeyPoint> >&current_keypoints)
{
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
    return;
  }

  size_t num_meas = current_frame->NumMeasurements();
  std::vector<MultiViewMeasurement> cur_meas(
      num_meas, MultiViewMeasurement(rig_.cameras.size()));
  for (size_t zi = 0; zi < num_meas; ++zi) {
    current_frame->GetMeasurement(zi, &cur_meas[zi]);
  }

  current__measurements_.swap(cur_meas);
  current_keypoints_.swap(*current_keypoints);
}

void Gui::set_map(std::shared_ptr<SlamMapProxy> p) {
  map_ = p;
}

void Gui::SetCameraRig(const calibu::CameraRigT<Scalar>& rig) {
    rig_ = rig;
}

bool Gui::current_t_wp(Sophus::SE3t* out) const {
  bool success = false;
  if ((success = global_view_->GetFramePose(current_frame_id_, out))) {
    *out = initial_pose_ * (*out);
  }
  return success;
}















