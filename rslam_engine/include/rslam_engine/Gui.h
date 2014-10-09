#ifndef GUI_H
#define GUI_H

#include <slam_map/PointerSlamMapProxy.h>
#include <slam_map/ReferenceFrame.h>
#include <calibu/cam/CameraRig.h>
#include <pb_msgs/ImageArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <deque>
#include <slam_map/GlobalMapView/GlobalMapView.h>
#include <visualization_msgs/Marker.h>

class TrackInfo
{
  public:
    TrackInfo(){};
    void Update(const std::vector<MultiViewMeasurement>& vMeasurements, const std::vector<std::vector<cv::KeyPoint> >& vMultiKeypts,
                const ReferenceFrameId& frame_id_in)
    {
      frame_id_ = frame_id_in;
      measurements_ = vMeasurements;
      keypoints_ = vMultiKeypts;
    }
    
    std::vector<MultiViewMeasurement> measurements_;
  private:
    ReferenceFrameId frame_id_;
    std::vector<std::vector<cv::KeyPoint> > keypoints_;

};

using TrackInfoPtr = std::shared_ptr<TrackInfo>;

class Gui
{
  public:
  
  Gui(){num_visible_time_steps_ = 5;init_=false; map_id_=0; landmarks_id_=0; num_poses_to_show_=40; only_draw_active_=false;};

  void set_map(std::shared_ptr<SlamMapProxy> p);
  void SetCameraRig(const calibu::CameraRigT<Scalar>& rig);
  bool Update(const std::shared_ptr<pb::ImageArray> &frames, const ReferenceFrameId& id, std::vector<std::vector<cv::KeyPoint> >&current_keypoints);
 
  void GetDisplayImage(cv::Mat &image);
  bool GetMap(visualization_msgs::Marker &map);
  bool GetLandmarks(visualization_msgs::Marker &landmarks);
  // Current position of the robot in world coordinates
  //bool current_t_wp(Sophus::SE3t* out) const;
  bool init(){return init_;};

  private:
  int map_id_;
  int landmarks_id_;
  std::unique_lock<std::mutex> lock_gui() const;
//  std::shared_ptr<TrackView> track_view_;
  std::shared_ptr<SlamMapProxy> map_;
  calibu::CameraRigT<Scalar> rig_;

  // for passing data through to the gui
  std::vector<cv::Mat> current_frames_;
  std::vector<MultiViewMeasurement> current_measurements_;
  std::vector<std::vector<cv::KeyPoint> > current_keypoints_;
  ReferenceFrameId current_frame_id_;
  std::deque<std::shared_ptr<TrackInfo> > track_info_;
  int num_visible_time_steps_;
  mutable std::mutex mutex_;
  bool init_;

  int num_poses_to_show_;
  bool only_draw_active_;

  std::shared_ptr<GlobalMapView> global_view_;
  std::vector<TransformEdgeId> edge_ids_;
  Eigen::Vector3tArray line_points_;
  std::vector<int> maps_;
  int num_maps_;
};

#endif
