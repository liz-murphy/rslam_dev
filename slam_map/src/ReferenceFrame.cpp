// Copyright(c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

//   \file ReferenceFrame.h
//
//   This class represents a coordinate frame on the manifold, usually
//   coincident with the vehicle or primary sensor.
//

#include <slam_map/ReferenceFrame.h>

#include <algorithm>
#include <limits>
#include <vector>

#include <utils/TicToc.h>
#include <slam_map/FrameObject.h>
#include <slam_map/pose_measurement.h>
#include <slam_map/NotificationCenter.h>

ReferenceFrame::ReferenceFrame() : is_isolated_(true),
                                   v_r_(Eigen::Vector3t::Zero()),
                                   g_r_(-Eigen::Vector3t::UnitZ()),
                                   b_(Eigen::Vector6t::Zero()),
                                   time_(0),
                                   last_modified_time_(hal::Tic()) {
  id_.store(ReferenceFrameId());
  parent_edge_id_.store(TransformEdgeId());
}

ReferenceFrame::ReferenceFrame(const ReferenceFrame& frame)
    : is_isolated_(frame.is_isolated_.load()),
      cam_params_(frame.cam_params_),
      t_vs_(frame.t_vs_),
      v_r_(frame.v_r_),
      g_r_(frame.g_r_),
      b_(frame.b_),
      time_(frame.time_),
      last_modified_time_(frame.last_modified_time_.load()),
      neighbor_edge_ids_(frame.neighbor_edge_ids_),
      landmarks_(frame.landmarks_),
      measurements_(frame.measurements_),
      measurements_set_(frame.measurements_set_) {
  id_.store(frame.id_.load());
  parent_edge_id_.store(frame.parent_edge_id_.load());
}

ReferenceFrame::~ReferenceFrame() {}

void ReferenceFrame::Merge(const ReferenceFrame& frame) {
  LockGuardT lock(mutex_);

  CHECK_EQ(id_.load(), frame.id());
  is_isolated_ = frame.is_isolated_.load();
  cam_params_ = frame.cam_params_;
  t_vs_ = frame.t_vs_;
  v_r_ = frame.v_r_;
  g_r_ = frame.g_r_;
  b_ = frame.b_;
  parent_edge_id_.store(frame.parent_edge_id_);
  time_ = frame.time_;

  for (const auto& m : frame.measurements_) {
    if (!measurements_set_.count(m.id())) {
      measurements_set_.insert({m.id(), m});
    }
  }
  measurements_.clear();
  for (const auto& m : measurements_set_) {
    measurements_.push_back(m.second);
  }

  size_t num_objs = objects_.size();
  for (const auto& other_obj : frame.objects_) {
    bool contained = false;
    // We only need to search up until the last original object
    for (size_t i = 0; i < num_objs; ++i) {
      if (*objects_[i] == *other_obj) {
        contained = true;
        break;
      }
    }
    if (!contained) {
      objects_.push_back(other_obj);
    }
  }

  std::set<TransformEdgeId> curr(neighbor_edge_ids_.begin(),
                                 neighbor_edge_ids_.end());
  for (const auto& o : frame.neighbor_edge_ids_) {
    if (!curr.count(o)) {
      neighbor_edge_ids_.push_back(o);
    }
  }

#define MERGE_VECTORS_BY_ID(IdT, field) {       \
    std::set<IdT> curr;                         \
    for (const auto& v : field) {               \
      curr.insert(v.id());                      \
    }                                           \
    for (const auto& o : frame.field) {         \
      if (!curr.count(o.id())) {                \
        field.push_back(o);                     \
      }                                         \
    }                                           \
  }

  MERGE_VECTORS_BY_ID(LandmarkId, landmarks_);
#undef MERGE_VECTORS_BY_ID

  MarkAsModified();
}

void ReferenceFrame::AddNeighbor(const TransformEdgeId& edge_id) {
  LockGuardT lock(mutex_);
  neighbor_edge_ids_.push_back(edge_id);
  MarkAsModified();
}

bool ReferenceFrame::HasNeighbor(const TransformEdgeId& edge_id) const {
  LockGuardT lock(mutex_);
  auto has_neighbor = [&edge_id](const TransformEdgeId& eid) {
    return eid == edge_id;
  };

  return std::any_of(neighbor_edge_ids_.begin(), neighbor_edge_ids_.end(),
                     has_neighbor);
}

void ReferenceFrame::RemoveMeasurements() {
  LockGuardT lock(mutex_);
  measurements_set_.clear();
  measurements_.clear();
  MarkAsModified();
}

// Setters
void ReferenceFrame::set_id(const ReferenceFrameId& id) {
  LockGuardT lock(mutex_);
  id_ = id;
  MarkAsModified();
}

void ReferenceFrame::set_parent_edge_id(const TransformEdgeId& edge_id) {
  parent_edge_id_ = edge_id;
  MarkAsModified();
}

void ReferenceFrame::set_linked() {
  is_isolated_ = false;
  MarkAsModified();
}

unsigned int ReferenceFrame::NumNeighbors() const {
  LockGuardT lock(mutex_);
  return neighbor_edge_ids_.size();
}

size_t ReferenceFrame::NumLandmarks() const {
  LockGuardT lock(mutex_);
  return landmarks_.size();
}

size_t ReferenceFrame::NumMeasurements() const {
  LockGuardT lock(mutex_);
  return measurements_.size();
}

const std::vector<TransformEdgeId>& ReferenceFrame::Neighbors()  const {
  LockGuardT lock(mutex_);
  return neighbor_edge_ids_;
}

void ReferenceFrame::AddMeasurement(MultiViewMeasurement& z) {
  MeasurementId zid = z.id();
  zid.frame_id = id_;
  z.set_id(zid);

  LockGuardT lock(mutex_);
  if (measurements_set_.count(z.id())) return;

  measurements_set_.insert({z.id(), z});
  measurements_.push_back(z);
  MarkAsModified();
}

void ReferenceFrame::AddLandmark(Landmark &lm) {
  LockGuardT lock(mutex_);
  landmarks_.push_back(lm);
  MarkAsModified();
}

void ReferenceFrame::AddMultiViewMeasurementoLandmark(MultiViewMeasurement& z) {
  bool bSuccessTracking = z.HasGoodMeasurement();

  LockGuardT lock(mutex_);
  landmarks_[ z.id().landmark_id.landmark_index ].AddToFeatureTrack(
      z.id(), bSuccessTracking);
  MarkAsModified();
}

void ReferenceFrame::RemoveNeighbor(const TransformEdgeId& edge_id) {
  LockGuardT lock(mutex_);
  auto it = std::find(neighbor_edge_ids_.begin(),
                      neighbor_edge_ids_.end(),
                      edge_id);
  if(it != neighbor_edge_ids_.end()){
    neighbor_edge_ids_.erase(it);
  }
  MarkAsModified();
}

TransformEdgeId
ReferenceFrame::GetNeighborEdgeId(const unsigned int idx) const {
  CHECK_LT(idx, neighbor_edge_ids_.size());
  LockGuardT lock(mutex_);
  return neighbor_edge_ids_[idx];
}

bool ReferenceFrame::HasGoodMeasurementInCam(unsigned int z_index,
                                             unsigned int cam_id) const {
  LockGuardT lock(mutex_);
  return (z_index < measurements_.size() &&
          measurements_[z_index].HasGoodMeasurementInCam(cam_id));
}

bool ReferenceFrame::GetMeasurementId(unsigned int z_index,
                                      MeasurementId* id_out) const {
  CHECK_NOTNULL(id_out);
  LockGuardT lock(mutex_);
  if (z_index < measurements_.size()) {
    *id_out = measurements_[z_index].id();
    return true;
  }

  return false;
}

bool ReferenceFrame::GetMeasurement(unsigned int z_index,
                                    MultiViewMeasurement* z_out) const {
  LockGuardT lock(mutex_);
  if (z_index < measurements_.size()) {
    *z_out = measurements_[z_index];
    return true;
  }

  return false;
}

bool ReferenceFrame::GetMeasurement(const MeasurementId& z_id,
                                    MultiViewMeasurement* z_out) const {
  LockGuardT lock(mutex_);
  auto it = measurements_set_.find(z_id);
  if (it != measurements_set_.end()) {
    *z_out = it->second;
    return true;
  }

  return false;
}

bool ReferenceFrame::GetMeasurement(const LandmarkId& lm_id,
                                    MultiViewMeasurement* z_out) const {
  LockGuardT lock(mutex_);
  for (const MultiViewMeasurement& z : measurements_) {
    if (z.id().landmark_id == lm_id) {
      *z_out = z;
      return true;
    }
  }
  return false;
}

bool ReferenceFrame::SetLandmark(const unsigned int landmark_index,
                                 const Landmark& lm) {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    landmarks_[landmark_index] = lm;
    return true;
  }

  return false;
}

bool ReferenceFrame::GetLandmark(const unsigned int landmark_index,
                                 Landmark* lm) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    *lm = landmarks_[landmark_index];
    return true;
  }
  return false;
}

bool ReferenceFrame::GetLandmarkFeatureTrackRef(
    uint32_t landmark_index,
    const std::vector<MeasurementId>*& feature_track) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    feature_track = &landmarks_[landmark_index].GetFeatureTrackRef();
    return true;
  }
  return false;
}

bool ReferenceFrame::GetLandmarkBaseCamera(const unsigned int landmark_index,
                                           uint32_t *cam) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    *cam = landmarks_[landmark_index].base_camera();
    return true;
  }
  return false;
}

bool ReferenceFrame::GetLandmarkXr(const unsigned int landmark_index,
                                   Eigen::Vector4t* x_r) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    *x_r = landmarks_[landmark_index].xrp();
    return true;
  }
  return false;
}

bool ReferenceFrame::GetLandmarkState(const unsigned int landmark_index,
                                   LandmarkState* state) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    *state = landmarks_[landmark_index].state();
    return true;
  }
  return false;
}

bool ReferenceFrame::GetLandmarkCamPlaneExtent(unsigned int landmark_index,
                                               Scalar* extent) const {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    *extent = landmarks_[landmark_index].cam_plane_extent();
    return true;
  }
  return false;
}

bool ReferenceFrame::SetLandmarkXr(const unsigned int landmark_index,
                                   const Eigen::Vector4t& x_r) {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    landmarks_[landmark_index].set_xrp(x_r);
    return true;
  }
  return false;
}

bool ReferenceFrame::SetLandmarkExtent(unsigned int landmark_index,
                                       Scalar extent) {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    landmarks_[landmark_index].set_extent(extent);
    return true;
  }
  return false;
}

bool ReferenceFrame::SetLandmarkState(const unsigned int landmark_index,
                                   LandmarkState state) {
  LockGuardT lock(mutex_);
  if (landmark_index < landmarks_.size()) {
    landmarks_[landmark_index].set_state(state);
    return true;
  }
  return false;
}

void ReferenceFrame::SetDepthImage(const cv::Mat& DepthImage) {
  LockGuardT lock(mutex_);
  depth_image_ = DepthImage.clone();
  MarkAsModified();
}

void ReferenceFrame::SetGreyImage(const cv::Mat& GreyImage) {
  LockGuardT lock(mutex_);
  grey_image_ = GreyImage.clone();
  MarkAsModified();
}

void ReferenceFrame::Clear() {
  LockGuardT lock(mutex_);
  time_ = std::numeric_limits<double>::max();

  neighbor_edge_ids_.clear();
  landmarks_.clear();
  measurements_.clear();
  measurements_set_.clear();
  v_r_.setZero();
  pose_measurements_.clear();
  MarkAsModified();
}

void ReferenceFrame::MarkAsModified() {
  last_modified_time_ = hal::Tic();
  if (notification_center_) {
    notification_center_->Notify(rslam::map::kUpdateFrameMapEvent, id_);
  }
}

void ReferenceFrame::AddObject(const std::shared_ptr<FrameObject>& fo) {
  LockGuardT lock(mutex_);
  objects_.push_back(fo);
  MarkAsModified();
}

void ReferenceFrame::RemoveObject(size_t i) {
  LockGuardT lock(mutex_);
  CHECK_LT(i, objects_.size());
  objects_.erase(objects_.begin() + i);
  MarkAsModified();
}

void ReferenceFrame::GetObject(size_t i,
                               std::shared_ptr<FrameObject>* o) const {
  LockGuardT lock(mutex_);
  CHECK_LT(i, objects_.size());
  CHECK_NOTNULL(o);
  *o = objects_[i];
}

void ReferenceFrame::GetObjects(
    std::vector<std::shared_ptr<FrameObject> >* objects) const {
  LockGuardT lock(mutex_);
  CHECK_NOTNULL(objects);
  *objects = objects_;
}

size_t ReferenceFrame::NumObjects() const {
  LockGuardT lock(mutex_);
  return objects_.size();
}

void ReferenceFrame::AddPoseMeasurement(const rslam::map::PoseMeasurement& z) {
  LockGuardT lock(mutex_);
  pose_measurements_.push_back(z);
  MarkAsModified();
}

size_t ReferenceFrame::NumPoseMeasurements() const {
  LockGuardT lock(mutex_);
  return pose_measurements_.size();
}

bool ReferenceFrame::GetPoseMeasurement(size_t index,
                                        rslam::map::PoseMeasurement* z) const {
  CHECK_NOTNULL(z);
  LockGuardT lock(mutex_);
  if (index < pose_measurements_.size()) {
    *z = pose_measurements_[index];
    return true;
  }
  return false;
}
