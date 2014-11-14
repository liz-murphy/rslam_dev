#pragma once
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "options.h"
#include "track.h"
#include "keypoint.h"
#include "utils.h"
#include <utils/MathTypes.h>
//#include <Utils/PatchUtils.h>
#include "TicToc.h"
#include <calibu/cam/CameraRig.h>
#include "FeatureMask.h"
//#include <Utils/Utils.h>
#include <fstream>
#include <calibu/cam/camera_crtp_interop.h>
#include <Eigen/Eigenvalues>
#include <random>

#define UNINITIALIZED_TRANSFER UINT_MAX

namespace sdtrack {
class SemiDenseTracker {
public:
  static const int kUnusedCell = -1;

  SemiDenseTracker() :
    generator_(0) {}
  void Initialize(const KeypointOptions& keypoint_options,
                  const TrackerOptions& tracker_options,
                  calibu::Rig<Scalar>* rig);

  void StartNewLandmarks(int only_start_in_camera = -1);

  double EvaluateTrackResiduals(
    uint32_t level,
    const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
    std::list<std::shared_ptr<DenseTrack>>& tracks,
    bool transfer_jacobians = false,
    bool optimized_tracks_only = false);

  void TransformTrackTabs(const Sophus::SE3t& t_cb);

  void OptimizeTracks(const OptimizationOptions& options, uint32_t level = -1);
  /// This is legacy to ensure backwards compatibility. Remove when all
  /// libraries update to the new version
  void OptimizeTracks(uint32_t level = -1, bool optimize_landmarks = true,
                      bool optimize_pose = true, bool trust_guess = false);

  void PruneTracks(int only_prune_camera = -1);

  void OptimizePyramidLevel(
    uint32_t level,
    const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
    std::list<std::shared_ptr<DenseTrack>>& tracks,
    const PyramidLevelOptimizationOptions& options,
    OptimizationStats& stats);

  void AddImage(const std::vector<cv::Mat>& images,
                const Sophus::SE3t& t_ab_guess);
  void AddKeyframe() {
    last_image_was_keyframe_ = true;
  }

  std::vector<std::vector<cv::Mat>>& GetImagePyramid() {
    return image_pyramid_;
  }
  void PruneOutliers();
  std::list<std::shared_ptr<DenseTrack>>& GetCurrentTracks() {
    return current_tracks_;
  }
  std::list<std::shared_ptr<DenseTrack>>& GetNewTracks() {
    return new_tracks_;
  }
  const Sophus::SE3t& t_ba() {
    return t_ba_;
  }
  void set_t_ba(const Sophus::SE3t& t_ba) {
    t_ba_ = t_ba;
  }
  uint32_t num_successful_tracks() {
    return num_successful_tracks_;
  }
  uint32_t longest_track_id() {
    return longest_track_id_;
  }

  void BackProjectTrack(std::shared_ptr<DenseTrack> track,
                        bool initialize_pixel_vals = false);

  void Do2dAlignment(const AlignmentOptions &options,
                     const std::vector<std::vector<cv::Mat>>& image_pyrmaid,
                     std::list<std::shared_ptr<DenseTrack>>& tracks,
                     uint32_t level);
  void Do2dTracking(std::list<std::shared_ptr<DenseTrack>>& tracks);
  std::vector<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>>&
    feature_cells() { return feature_cells_; }
private:
  uint32_t StartNewTracks(std::vector<cv::Mat>& image_pyrmaid,
                          std::vector<cv::KeyPoint>& cv_keypoints,
                          uint32_t num_to_start, uint32_t cam_id);


  void TransferPatch(std::shared_ptr<DenseTrack> track,
                     uint32_t level,
                     uint32_t cam_id,
                     const Sophus::SE3t& t_ba,
                     calibu::CameraInterface<Scalar>* cam,
                     PatchTransfer& result, bool transfer_jacobians,
                     bool use_approximation = true);


  inline double GetSubPix(const cv::Mat& image, double x, double y);

  void ReprojectTrackCenters();

  void ExtractKeypoints(const cv::  Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        uint32_t cam_id);

  inline bool IsKeypointValid(const cv::KeyPoint& kp, uint32_t image_width,
                              uint32_t image_height, uint32_t cam_id);

  inline bool IsReprojectionValid(
    const Eigen::Vector2t& pix, const cv::Mat& image);
  inline void GetImageDerivative(
    const cv::Mat& image, const Eigen::Vector2d& pix,
    Eigen::Matrix<double, 1, 2>& di_dpix, double val_pix);

  Sophus::SE3t t_ba_;
  uint32_t num_cameras_;
  bool last_image_was_keyframe_ = true;
  double average_track_length_;
  uint32_t tracks_suitable_for_cam_localization = 0;
  TrackerOptions  tracker_options_;
  KeypointOptions keypoint_options_;
  std::vector<DenseKeypoint> previous_keypoints_;
  cv::FeatureDetector* detector_;
  std::list<std::shared_ptr<DenseTrack>> current_tracks_;
  std::list<std::shared_ptr<DenseTrack>> new_tracks_;
  uint32_t num_successful_tracks_;
  uint32_t next_track_id_;
  uint32_t longest_track_id_;
  FeatureMask mask_;
  std::vector<uint32_t> pyramid_patch_dims_;
  std::vector<std::vector<uint32_t>> pyramid_patch_corner_dims_;
  std::vector<std::vector<std::vector<double>>> pyramid_patch_interp_factors_;
  std::vector<Eigen::Vector2t> pyramid_coord_ratio_;
  std::vector<std::vector<cv::Mat>> image_pyramid_;
  std::vector<double> pyramid_error_thresholds_;
  std::vector<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>>
    feature_cells_;
  std::vector<uint32_t> active_feature_cells_;
  calibu::Rig<Scalar>* camera_rig_;
  Eigen::Matrix4d generators_[6];
  std::default_random_engine generator_;

};
}
