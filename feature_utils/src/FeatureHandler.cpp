// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <memory>
#include <iostream>
#include <feature_utils/FastPyramidDetector.h>
#include <feature_utils/FeatureHandler.h>
#include <feature_utils/PatchMatch.h>
#include <feature_utils/FeatureParams.h>
#include <miniglog/logging.h>
#include <opencv2/nonfree/features2d.hpp>

bool FeatureHandler::Init() {

  // Initialize the feature detector
  switch(FeatureParams::getParams()->getFeatureDetector()) {
    case feature_utils::Feature_DOG:
      ROS_INFO("Using DOG feature detector");
      m_pFeatureDetector = std::shared_ptr<cv::FeatureDetector>(
          new cv::SurfFeatureDetector(FeatureParams::getParams()->getSurfHessianThreshold()));
      break;
    case feature_utils::Feature_FAST:
      ROS_INFO("Using FAST feature detector");
      m_pFeatureDetector =
          std::shared_ptr<cv::FeatureDetector>(
              new FastPyramidDetector(FeatureParams::getParams()->getFastThreshold(),
                FeatureParams::getParams()->fastDoNonMaxSuppression(),
                FeatureParams::getParams()->fastSkipLevel0()) );
      break;
    default:
      std::cerr << " Feature detector not recognized. ";
      std::cerr << " valid: [FAST] " << std::endl;
      return false;
  }

  // Initialize descriptor extractor
  //switch (options.feature_descriptor)
  switch (FeatureParams::getParams()->getFeatureDescriptor())
  {
    case feature_utils::Feature_PATCH:
      {
        ROS_INFO("Using PATCH descriptors");
        // No descriptor is needed, we are using patch-matching
        m_pDescriptorExtractor = NULL;
        break;
      }
    case feature_utils::Feature_FREAK:
      {
        ROS_INFO("Using FREAK descriptors");
        m_pDescriptorExtractor =
            std::shared_ptr<cv::DescriptorExtractor>(
                new cv::FREAK(
                  FeatureParams::getParams()->freakOrientationNormalized(),
                  FeatureParams::getParams()->freakScaleNormalized(),
                  FeatureParams::getParams()->getFreakPatternScale(),
                  FeatureParams::getParams()->getFreakNOctaves()) );
        break;
      }
    case feature_utils::Feature_SURF:
      {
        ROS_INFO("Using SURF descriptors");
        m_pDescriptorExtractor = std::shared_ptr<cv::DescriptorExtractor>( new cv::SURF(
            FeatureParams::getParams()->getSurfHessianThreshold(),
            FeatureParams::getParams()->getSurfNOctaves(),
            FeatureParams::getParams()->getSurfNOctaveLayers(),
            FeatureParams::getParams()->surfExtended(),
            FeatureParams::getParams()->surfUpright()) );
        break;
      }
    default:
      {
        std::cerr << " Descriptor type not recognized. ";
        std::cerr << " valid: [PATCH, FREAK, SURF] " << std::endl;
        return false;
      }
  }
  return true;
}

void FeatureHandler::DetectFeatures(
    const std::shared_ptr<pb::ImagePyramid>& pyramid,
    std::vector<cv::KeyPoint>& keypoints) {
  //if(m_Options.feature_detector == FAST) {
    if(FeatureParams::getParams()->getFeatureDetector() == feature_utils::Feature_FAST)
    {
      auto* ptr =
          reinterpret_cast<FastPyramidDetector*>(m_pFeatureDetector.get());
      ptr->SetImagePyramidPtr(pyramid);
      ptr->detect(pyramid->at(0), keypoints);
  } 
  else {
    m_pFeatureDetector->detect(pyramid->at(0), keypoints);
  }
}

void FeatureHandler::ComputeDescriptors(
    const std::shared_ptr<pb::ImagePyramid>& pyramid,
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors) {
  if (m_pDescriptorExtractor) {
    m_pDescriptorExtractor->compute(pyramid->at(0), keypoints, descriptors);
  }
}
