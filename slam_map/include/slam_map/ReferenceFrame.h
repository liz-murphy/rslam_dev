// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   \file ReferenceFrame.h
//
//   This class represents a coordinate frame on the manifold, usually
//   coincident with the vehicle or primary sensor.
//

#pragma once

#include <atomic>
#include <map>
#include <unordered_map>
#include <vector>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <slam_map/Measurement.h>
#include <slam_map/Landmark.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/TransformEdgeId.h>
#include <utils/MathTypes.h>

class ReferenceFrame {
  typedef std::lock_guard<std::mutex> LockGuardT;
 public:
  ReferenceFrame();
  ReferenceFrame(const ReferenceFrame& frame);
  ~ReferenceFrame();
  void Merge(const ReferenceFrame& frame);

  void AddNeighbor(const TransformEdgeId& edge_id);
  void RemoveNeighbor(const TransformEdgeId& edge_id);
  bool HasNeighbor(const TransformEdgeId& edge_id) const;
  unsigned int NumNeighbors() const;

  // Getters
  ReferenceFrameId id() const {
    return id_;
  }

  double time() const {
    LockGuardT lock(mutex_);
    return time_;
  }

  Eigen::Vector3t g_r() const {
    LockGuardT lock(mutex_);
    return g_r_;
  }

  Sophus::SE3Group<Scalar> t_vs() const {
    LockGuardT lock(mutex_);
    return t_vs_;
  }

  Eigen::Vector3t v_r() const {
    LockGuardT lock(mutex_);
    return v_r_;
  }

  Eigen::Vector6t b() const {
    LockGuardT lock(mutex_);
    return b_;
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> cam_params() const {
    LockGuardT lock(mutex_);
    return cam_params_;
  }

  TransformEdgeId parent_edge_id() const {
    return parent_edge_id_;
  }

  bool is_isolated() const {
    return is_isolated_;
  }

  double last_modified_time() const {
    return last_modified_time_;
  }

  void set_id(const ReferenceFrameId& id);
  void set_time(const double time) { time_ = time; }
  void set_parent_edge_id(const TransformEdgeId& edge_id);
  void set_linked();

  void set_g_r(const Eigen::Vector3t& g_r) {
    LockGuardT lock(mutex_);
    g_r_ = g_r;
  }

  void set_t_vs(const Sophus::SE3t& tvs) {
    LockGuardT lock(mutex_);
    t_vs_ = tvs;
  }

  void set_v_r(const Eigen::Vector3t& V) {
    LockGuardT lock(mutex_);
    v_r_ = V;
  }

  void set_b(const Eigen::Vector6t& biases){
    LockGuardT lock(mutex_);
    b_ = biases;
  }

  void set_cam_params(const Eigen::VectorXt& params) {
    LockGuardT lock(mutex_);
    cam_params_ = params;
  }

  void set_last_modified_time(double t) {
    last_modified_time_ = t;
  }

  void set_notification_center(
      const std::shared_ptr<rslam::map::NotificationCenter>& notifier) {
    notification_center_ = notifier;
  }

  TransformEdgeId GetNeighborEdgeId(const unsigned int idx) const;

  const std::vector<TransformEdgeId>& Neighbors() const;

  /// Sparse
  void AddMeasurement(MultiViewMeasurement& z);
  void AddLandmark(Landmark &lm);
  void AddMultiViewMeasurementoLandmark(MultiViewMeasurement& z);
  void RemoveMeasurements();

  bool HasGoodMeasurement(unsigned int z_index) const {
    LockGuardT lock(mutex_);
    return (z_index < measurements_.size() &&
            measurements_[z_index].HasGoodMeasurement());
  }

  bool HasGoodMeasurement(const MeasurementId& zid) const {
    LockGuardT lock(mutex_);
    try {
      return measurements_set_.at(zid).HasGoodMeasurement();
    } catch(...) {
      return false;
    }
  }

  bool HasGoodMeasurementInCam(unsigned int z_index, unsigned int cam_id) const;

  size_t NumMeasurements() const;
  bool GetMeasurementId(unsigned int z_index,
                        MeasurementId* z_out) const;
  bool GetMeasurement(unsigned int z_index, MultiViewMeasurement *z_out) const;
  bool GetMeasurement(const MeasurementId& zId,
                      MultiViewMeasurement* z_out) const;
  bool GetMeasurement(const LandmarkId& lm_id, MultiViewMeasurement* z) const;

  size_t NumLandmarks() const;
  bool GetLandmark(const unsigned int landmark_index, Landmark* lm) const;
  bool GetLandmarkCamPlaneExtent(unsigned int landmark_index,
                                 Scalar* extent) const;
  bool GetLandmarkXr(const unsigned int landmark_index,
                     Eigen::Vector4t *x_r) const;
  bool GetLandmarkBaseCamera(const unsigned int landmark_index,
                             uint32_t *cam) const;
  bool GetLandmarkFeatureTrackRef(uint32_t landmark_index,
      const std::vector<MeasurementId> *&feature_track) const;
  bool SetLandmarkXr(const unsigned int landmark_index,
                     const Eigen::Vector4t &x_r);
  bool SetLandmarkState(const unsigned int landmark_index,
                        LandmarkState state);
  bool GetLandmarkState(const unsigned int landmark_index,
                        LandmarkState *state) const;
  bool SetLandmark(const unsigned int landmark_index, const Landmark &lm);
  bool SetLandmarkExtent(const unsigned int landmark_index, Scalar extent);

  // Absolute pose measurements
  void AddPoseMeasurement(const rslam::map::PoseMeasurement& z);
  size_t NumPoseMeasurements() const;
  bool GetPoseMeasurement(size_t index, rslam::map::PoseMeasurement* z) const;

  // Frame Objects: Attached metadata
  void AddObject(const std::shared_ptr<FrameObject>& fo);
  void RemoveObject(size_t i);
  void GetObject(size_t i, std::shared_ptr<FrameObject>* o) const;
  void GetObjects(std::vector<std::shared_ptr<FrameObject> >* objects) const;
  size_t NumObjects() const;

  /// Dense
  void SetGreyImage(const cv::Mat& GreyImage);
  void SetDepthImage(const cv::Mat& DepthImage);
  const cv::Mat& GetGreyImage() const { return grey_image_; }
  const cv::Mat& GetDepthImage() const { return depth_image_; }

  void Clear();

  /** Mark this frame as having been modified at the time this was called. */
  void MarkAsModified();

 private:
  std::atomic<bool> is_isolated_;
  Eigen::VectorXt cam_params_;
  Sophus::SE3Group<Scalar> t_vs_;
  Eigen::Vector3t v_r_;
  Eigen::Vector3t g_r_;
  Eigen::Vector6t b_;
  std::atomic<ReferenceFrameId> id_;
  std::atomic<TransformEdgeId> parent_edge_id_;

  // Time measurements were made
  double time_;

  // Timestamp of last modification to frame
  std::atomic<double> last_modified_time_;

  // for the co-vis graph
  std::vector<TransformEdgeId> neighbor_edge_ids_;

  /// Sparse
  // Vector of initialized landmarks at this frame
  std::vector<Landmark> landmarks_;

  // Vector of landmarks' image measurements
  std::vector<MultiViewMeasurement> measurements_;
  std::unordered_map<MeasurementId, MultiViewMeasurement> measurements_set_;

  /** Global position measurements */
  std::vector<rslam::map::PoseMeasurement> pose_measurements_;

  // Arbitrary renderable objects
  std::vector<std::shared_ptr<FrameObject> > objects_;

  // Dense
  cv::Mat grey_image_;
  cv::Mat depth_image_;

  std::shared_ptr<rslam::map::NotificationCenter> notification_center_;

  mutable std::mutex mutex_;
};
