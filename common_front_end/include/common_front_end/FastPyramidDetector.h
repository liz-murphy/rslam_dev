// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <utils/Utils.h>
#include <pb_msgs/ImagePyramid.h>

class CV_EXPORTS FastPyramidDetector : public cv::FeatureDetector {
public:
  FastPyramidDetector(int threshold, bool non_max_supression,
                      bool skip_level0 = false) :
      fast_detector_(new cv::FastFeatureDetector(threshold,
                                                 non_max_supression)),
      skip_level0_(skip_level0) {}

  static inline float scale_keypoint(float coord, float scale) {
    return (coord + 0.5) * scale - 0.5;
  }

  void detectImpl(const cv::Mat&,
                  std::vector<cv::KeyPoint>& keypoints,
                  const cv::Mat&) const override {
    std::vector<cv::KeyPoint> level_keypoints;
    size_t start_level = skip_level0_ ? 1 : 0;

    for (size_t level = start_level; level < pyramid_->NumLevels(); ++level) {
      const cv::Mat& img_level = pyramid_->at(level);
      float scale = 1.0 / std::pow(pyramid_->ScaleFactor(), level);

      fast_detector_->detect(img_level, level_keypoints);
      HarrisScore(img_level.data, img_level.cols, img_level.rows,
                  level_keypoints);
      for (cv::KeyPoint& kp : level_keypoints) {
        kp.pt.x = scale_keypoint(kp.pt.x, scale);
        kp.pt.y = scale_keypoint(kp.pt.y, scale);
        kp.size = scale;
        kp.octave = level; // little hack
        kp.angle  = 0;
      }
      keypoints.insert(
          keypoints.end(), level_keypoints.begin(), level_keypoints.end());
      level_keypoints.clear();
    }
  }

  void SetImagePyramidPtr(const std::shared_ptr<pb::ImagePyramid>& pyramid) {
    pyramid_ = pyramid;
  }

private:
  std::shared_ptr<cv::FastFeatureDetector>   fast_detector_;
  bool                                       skip_level0_;
  std::shared_ptr<pb::ImagePyramid>          pyramid_;
};
