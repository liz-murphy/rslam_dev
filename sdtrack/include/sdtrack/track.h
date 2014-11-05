#pragma once
#include <vector>
#include <Eigen/Eigen>
#include "keypoint.h"
#include <sophus/se3.hpp>

namespace sdtrack
{
  struct Track
  {
    static const uint32_t kUntrackedKeypoint = UINT_MAX;
    uint32_t id;
    std::vector<Eigen::Vector2d> keypoints;
    std::vector<unsigned char> descriptor;
    bool tracked;
  };

  struct PatchTransfer
  {
    std::vector<double> residuals;
    std::vector<double> projected_values;
    std::vector<Eigen::Vector2d> projections;

    Eigen::Vector2d center_projection;
    Eigen::Matrix<double, 2, 3> center_dprojection;
    std::vector<Eigen::Vector2d> valid_projections;
    std::vector<unsigned int> valid_rays;
    std::vector<Eigen::Matrix<double, 2, 4>> dprojections;
    double mean_value;
    uint32_t level;
    double dimension;
    uint32_t patch_dim;

    double rmse = 0;
    double ncc = 1.0;
    uint32_t tracked_pixels;
    uint32_t pixels_attempted;

    void GetProjectedPerimiter(std::vector<Eigen::Vector2d>& points,
                               Eigen::Vector2d& center) const
    {
      center = projections[(projections.size() - 1) / 2];
      for (size_t ii = 0; ii < patch_dim ; ++ii) {
        points.push_back(projections[ii]);
      }

      size_t index = (patch_dim * 2) - 1;
      for (size_t ii = 0; ii < patch_dim - 1 ; ++ii) {
        points.push_back(projections[index]);
        index += patch_dim;
      }

      for (size_t ii = 1; ii < patch_dim ; ++ii) {
        points.push_back(projections[projections.size() - patch_dim]);
      }

      index = (patch_dim * (patch_dim - 2));
      for (size_t ii = 0; ii < patch_dim - 1 ; ++ii) {
        points.push_back(projections[index]);
        index -= patch_dim;
      }
    }
  };

  struct Keypoint
  {
    Keypoint() {}
    Keypoint(const Eigen::Vector2d& kp_val, const bool tracked_val,
                  const uint32_t external_data_val) :
      kp(kp_val), tracked(tracked_val), external_data(external_data_val) {}
    Eigen::Vector2d kp;
    bool tracked = false;
    uint32_t external_data = UINT_MAX;
  };

  struct DenseTrack
  {
    DenseTrack(uint32_t num_pyrmaid_levels,
               const std::vector<uint32_t>& pyramid_dims,
               uint32_t num_cameras):
      ref_keypoint(num_pyrmaid_levels, pyramid_dims)
    {
      offset_2d.resize(num_cameras);
      transfer.resize(num_cameras);
      for (uint32_t ii = 0; ii < num_cameras ; ++ii) {
        transfer[ii].projected_values.resize(
              ref_keypoint.patch_pyramid[0].values.size());
        transfer[ii].projections.resize(
              ref_keypoint.patch_pyramid[0].values.size());
        transfer[ii].patch_dim =
            ref_keypoint.patch_pyramid[0].dim;
      }
      ref_keypoint.track = this;
      external_id.resize(2);
    }

    std::vector<PatchTransfer> transfer;
    double jtj = 0;
    uint32_t opt_id;
    uint32_t residual_offset;
    std::vector<uint32_t> external_id;
    uint32_t ref_cam_id;
    uint32_t id;
    uint32_t num_good_tracked_frames = 0;
    DenseKeypoint ref_keypoint;
    std::vector<std::vector<Keypoint>> keypoints;
    bool residual_used = false;
    bool tracked = false;
    bool is_new = true;
    std::vector<Eigen::Vector2d> offset_2d;
    bool is_outlier = false;
    bool needs_backprojection = false;
    Sophus::SE3d t_ba;

    uint32_t external_data = UINT_MAX;
    uint32_t external_data2 = UINT_MAX;
  };
}
