// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef BackEnd_H
#define BackEnd_H

#include <utils/MathTypes.h>
#include <ba/BundleAdjuster.h>
#include <sophus/se3.hpp>
#include <iostream>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/ReferenceFrameId.h>
#include <sparse_tracking/LocalMap.h>
#include <ba/InterpolationBuffer.h>

enum OptimizationStatus {
  OptStatus_DiffOverThreshold = 1,
  OptStatus_DiffBelowThreshold = 2,
  OptStatus_NoChange = 3,
};

namespace rslam {
namespace backend {
/** Results of map refinement */
struct LiftResults {
  std::unordered_map<TransformEdgeId, uint32_t> edge_ba_ids;
  std::unordered_map<ReferenceFrameId, uint32_t> pose_ba_ids;
  std::unordered_map<LandmarkId, uint32_t> landmark_ba_ids;
  ReferenceFrameId root_pose_id;
  TransformEdgeId root_edge_id;
};
}
}

class BackEnd {
 public:
  struct AdaptiveOptions {
    AdaptiveOptions(bool enable, bool async, uint32_t min_size,
                    uint32_t max_size) : do_adaptive(enable),
                                         is_asynchronous(async),
                                         min_window_size(min_size),
                                         max_window_size(max_size) {}
    bool do_adaptive;
    bool is_asynchronous;
    uint32_t min_window_size;
    uint32_t max_window_size;

    // Frame next to root which should not be lifted. Mainly used for
    // async ba to avoid lifting the current frame.
    ReferenceFrameId ignore_frame;
  };

  /** Callbacks after a map refinement optimization. */
  struct RefineMapCallbacks {
    std::function<void(const ba::VisualInertialBundleAdjuster<Scalar>&,
                       const rslam::backend::LiftResults&)>
    visual_inertial_callback;

    std::function<void(const ba::VisualBundleAdjuster<Scalar>&,
                       const rslam::backend::LiftResults&)> visual_callback;
  };

  BackEnd();
  virtual ~BackEnd();

  void Clear();

  void RegisterImuMeasurement(const Eigen::Vector3t& w,
                              const Eigen::Vector3t& a,
                              const double    time);

  void Solve(const unsigned int uMaxIter, const double *pRootParameterBlock);

  void RelaxMap(const unsigned int depth,
                const ReferenceFrameId& root_id,
                const unsigned int max_iter,
                const calibu::CameraRigT<Scalar> &rig);

  bool RefineMap(unsigned int depth,
                 const ReferenceFrameId& root_id,
                 unsigned int max_iter,
                 bool bUseImu,
                 calibu::CameraRigT<Scalar> &rig,
                 const AdaptiveOptions& adaptive_options,
                 bool do_landmark_init,
                 EdgeAttribute attribute,
                 const RefineMapCallbacks& callbacks);

  void SetVehicleConfiguration(const calibu::CameraRigT<Scalar> &VehCfg);

  Eigen::Matrix<Scalar,6,6> GetCurrentCovariance() const { return covariance_; }

  void Init(const std::shared_ptr<SlamMap>& pMap);

  const ba::ImuCalibrationT<Scalar>& GetImuCalibration() {
    return imu_ba_.GetImuCalibration();
  }

  ba::InterpolationBufferT<ba::ImuMeasurementT<Scalar>,Scalar>& GetImuBuffer() {
    return imu_buffer_;
  }

  void IntegrateLastPose(const double start_time,
                         const double end_time,
                         const Eigen::Vector3t v_r,
                         const Eigen::Vector3t g_r,
                         const Eigen::Vector6t b,
                         Sophus::SE3t& t_ab,
                         Eigen::Vector3t& g_r_new,
                         Eigen::Vector3t& v_r_new,
                         std::vector<ba::ImuMeasurementT<Scalar>>& meas);



  bool GaussNewton(const LocalMap &work_set,
                   const std::vector<MultiViewMeasurement> &measurements,
                   const SlamMap &map, Sophus::SE3t &t_ab,
                   bool has_imu,
                   std::vector<ba::ImuMeasurementT<Scalar> > &meas);

  const ba::VisualInertialBundleAdjuster<Scalar>& GetVisualInertialBA() const {
    return imu_ba_;
  }

  const ba::VisualBundleAdjuster<Scalar>& GetVisualBA() const {
    return ba_;
  }

 private:
  OptimizationStatus UpdateLocalMapFromBa(LocalMap &local_map,
                                          const bool is_using_imu,
                                          const bool remove_unused_landmarks,
                                          const bool update_landmarks,
                                          const bool calculate_adpative_metrics,
                                          const bool update_static_set);
  bool LoadLocalMapIntoBa(
      LocalMap &local_map, const bool is_using_imu,
      calibu::CameraRigT<Scalar> &rig, const bool init_new_landmarks,
      const EdgeAttribute attribute = EdgeAttrib_IsBeingOptimized);

  bool LoadPoseIntoBa(
      const LocalMap& local_map,
      const ReferenceFrameId &id,
      PoseContainerT &pose_container,
      bool is_active,
      bool is_using_imu,
      const std::map<SessionId, std::map<uint32_t, uint32_t> >& ba_cam_ids,
      bool init_new_landmarks);

  void _AddPose(const Sophus::SE3t &dT,
                const unsigned int& uId,
                const bool& bConstant);

  void _AddLandmark(const Eigen::Vector3t& dXr,
                    const LandmarkId& lmkId);

  /** Returns true on successful addition and puts ID from BA in ba_id */
  bool _AddImuResidualToEdge(const SlamEdgePtr& edge,
                             LocalMap& local_map,
                             const double weight,
                             unsigned int* ba_id);

  inline uint32_t AddPoseIntoBa(bool is_using_imu, bool is_active,
                                const PoseContainerT& pose);

  inline uint32_t AddLandmarkIntoBa(bool is_using_imu,
                                    bool is_active,
                                    uint32_t lm_ba_cam_id,
                                    uint32_t pose_id,
                                    const LandmarkContainer& lmk);

  inline uint32_t AddProjectionResidualIntoBa(bool is_using_imu,
                                              const Eigen::Vector2t& z,
                                              uint32_t pose_id,
                                              uint32_t lmk_id,
                                              uint32_t z_ba_cam_id);

  inline const ba::PoseT<Scalar>& GetPose(bool is_using_imu, uint32_t ba_id);

  void LiftMap(uint32_t depth,
               const ReferenceFrameId& root_id,
               bool do_landmark_init,
               bool use_imu,
               rslam::backend::LiftResults* results);

  bool PushMap(bool use_imu,
               const rslam::backend::LiftResults& results,
               const RefineMapCallbacks& callbacks);

  bool                                      is_running_;
  ReferenceFrameId                          min_pose_id_;
  ReferenceFrameId                          min_active_pose_id_;
  ReferenceFrameId                          last_min_pose_id_;
  ReferenceFrameId                          anchor_id;
  Sophus::SE3t                              last_anchor_t_wp;
  ba::VisualBundleAdjuster<Scalar>          ba_;
  ba::VisualInertialBundleAdjuster<Scalar>  imu_ba_;
  std::shared_ptr<SlamMap>                  map_;
  std::map<ReferenceFrameId, unsigned int>  pose_ids_;
  std::map<LandmarkId, Eigen::Vector3t>     landmarks_;
  Eigen::Matrix<Scalar,6,6>                 covariance_;
  PoseContainerT                            current_pose_;

  ba::InterpolationBufferT<ba::ImuMeasurementT<Scalar>,Scalar>  imu_buffer_;
  Scalar prev_cond_error_;
};

#endif	/* BackEnd_H */
