// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <PbMsgs/Image.h>
#include <CommonFrontEnd/flyby/include/cvflyby/wrappers/ncc_feature_matcher.h>
#include <CommonFrontEnd/CommonFrontEndCVars.h>
#include <miniglog/logging.h>

class CV_EXPORTS FlybyPyramidFeatureDetector : public cv::FeatureDetector {
 public:
  FlybyPyramidFeatureDetector(bool skip_level_zero)
      : skip_level_zero_(skip_level_zero) {}
  virtual ~FlybyPyramidFeatureDetector() {}

  static inline Scalar scale_keypoint(Scalar coord, Scalar scale) {
    return (coord + 0.5) * scale - 0.5;
  }

  void detectImpl(const cv::Mat&,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat&) const override {
    CHECK_NOTNULL(pyramid_.get());
    CHECK(trackers_.size() == pyramid_->NumLevels());

    std::vector<int> feature_ids;
    size_t start_level = skip_level_zero_ ? 1 : 0;
    for (size_t level = start_level; level < pyramid_->NumLevels(); ++level) {
      const cv::Mat& img_level = pyramid_->at(level);
      float scale = 1.0 / std::pow(pyramid_->ScaleFactor(), level);

      Eigen::Matrix2Xd current_meas, previous_meas;
      feature_ids.clear();
      trackers_[level]->AddImage(img_level.data, &current_meas,
                                 &previous_meas, &feature_ids);
      for (size_t i = 0; i < feature_ids.size(); ++i) {
        keypoints.emplace_back(scale_keypoint(current_meas(0, i), scale),
                               scale_keypoint(current_meas(1, i), scale),
                               scale, 0, level, 0, feature_ids[i]);
      }
    }
  }

  void SetImagePyramidPtr(const std::shared_ptr<pb::ImagePyramid>& pyramid) {
    if (pyramid_) {
      last_pyramid_.reset(new pb::ImagePyramid(*pyramid_));
    }
    pyramid_ = pyramid;

    if (trackers_.size() != pyramid_->NumLevels()) {
      trackers_.clear();
      trackers_.reserve(pyramid_->NumLevels());
      for (size_t i = 0; i < pyramid_->NumLevels(); ++i) {
        trackers_.emplace_back(new CvFlyBy::FeatureTracker);
        trackers_.back()->Init(pyramid_->at(i).cols,
                               pyramid_->at(i).rows,
                               g_common_cvars.flyby_num_features,
                               g_common_cvars.flyby_max_disparity_fraction,
                               g_common_cvars.flyby_harris_threshold);
      }
    }
  }

 private:
  std::vector<std::shared_ptr<CvFlyBy::FeatureTracker> > trackers_;
  std::shared_ptr<pb::ImagePyramid> last_pyramid_, pyramid_;
  bool skip_level_zero_;
};
