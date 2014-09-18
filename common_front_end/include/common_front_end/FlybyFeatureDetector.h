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

class CV_EXPORTS FlybyFeatureDetector : public cv::FeatureDetector {
 public:
  FlybyFeatureDetector() {}
  virtual ~FlybyFeatureDetector() {}

  void detectImpl(const cv::Mat&,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat&) const override {
    CHECK(tracker_) << "SetImage must be called on FlybyFeatureDetector";
    Eigen::Matrix2Xd current_meas, previous_meas;
    std::vector<int> feature_ids;
    tracker_->AddImage(image_->data(), &current_meas,
                       &previous_meas, &feature_ids);

    for (size_t i = 0; i < feature_ids.size(); ++i) {
      keypoints.emplace_back(current_meas(0, i),
                             current_meas(1, i),
                             1, 0, 1, 0, feature_ids[i]);
    }
  }

  void SetImage(const std::shared_ptr<pb::Image>& i) {
    last_image_ = image_;
    image_ = i;

    if (!tracker_) {
      tracker_.reset(new CvFlyBy::FeatureTracker);
      tracker_->Init(image_->Width(), image_->Height(),
                     g_common_cvars.flyby_num_features,
                     g_common_cvars.flyby_max_disparity_fraction,
                     g_common_cvars.flyby_harris_threshold);
    }
  }

 private:
  std::shared_ptr<CvFlyBy::FeatureTracker> tracker_;
  std::shared_ptr<pb::Image> image_, last_image_;
};
