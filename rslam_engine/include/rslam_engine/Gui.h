#ifndef GUI_H
#define GUI_H

class Gui
{
  Gui(){};

  void set_map(std::shared_ptr<SlamMapProxy> p);
  void SetCameraRig(const calibu::CameraRigT<Scalar>& rig);
  
  // Current position of the robot in world coordinates
  bool current_t_wp(Sophus::SE3t* out) const;

  private:
  std::shared_ptr<TrackView> track_view_;
  std::shared_ptr<SlamMapProxy> map_;
  calibu::CameraRigT<Scalar> rig_;

  // for passing data through to the gui
  std::vector<cv::Mat> current_frames_;
  std::vector<MultiViewMeasurement> current_measurements_;
  std::vector<std::vector<cv::KeyPoint> > current_keypoints_;
  std::atomic<ReferenceFrameId> current_frame_id_;
};

#endif
