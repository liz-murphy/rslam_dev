// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <pb_msgs/ImagePyramid.h>
#include <feature_utils/FeatureConfig.h>

enum DetectorType
{
  FAST          = feature_utils::Feature_FAST,
  DOG           = feature_utils::Feature_DOG,
};

enum DescriptorType
{
  PATCH         = feature_utils::Feature_PATCH,
  FREAK         = feature_utils::Feature_FREAK,
  BRISK         = feature_utils::Feature_BRISK,
  SURF          = feature_utils::Feature_SURF,
  SIFT          = feature_utils::Feature_SIFT
};

class FeatureHandler
{
public:
  //bool Init( const Options& options );
  bool Init( );

  void DetectFeatures(
      const std::shared_ptr<pb::ImagePyramid>& image_pyramid,
      std::vector<cv::KeyPoint>& keypoints);

  void ComputeDescriptors(
      const std::shared_ptr<pb::ImagePyramid>& image_pyramid,
      std::vector<cv::KeyPoint>& keypoints,
      cv::Mat& descriptors
      );

 // const Options& GetOptions() { return m_Options; }

  static DetectorType   StrToDetectorType( std::string s );
  static DescriptorType StrToDescriptorType( std::string s );

  //std::string DetectorTypeStr() const;
  //std::string DescriptorTypeStr() const;

  std::shared_ptr<cv::FeatureDetector> GetDetector() {
    return m_pFeatureDetector;
  }

private:
  //Options                                  m_Options;
  std::shared_ptr<cv::FeatureDetector>     m_pFeatureDetector;
  std::shared_ptr<cv::DescriptorExtractor> m_pDescriptorExtractor;
};
