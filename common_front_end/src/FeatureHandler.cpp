// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <memory>
#include <iostream>
#include <common_front_end/FastPyramidDetector.h>
#include <common_front_end/FeatureHandler.h>
#include <common_front_end/PatchMatch.h>
#include <common_front_end/SparseSimData.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <miniglog/logging.h>
#include <opencv2/nonfree/features2d.hpp>

//#include <HAL/config.h>
//#include <ThirdParty/FeatureHandler.h>
//#include <common_front_end/FlybyPyramidFeatureDetector.h>

class CV_EXPORTS SimFeatureDetector : public cv::FeatureDetector
{

public:
  SimFeatureDetector(unsigned int cam_id) : cam_id_(cam_id)
  {}

protected:
  void detectImpl(
      const cv::Mat&                  /*Image*/,
      std::vector<cv::KeyPoint>&      keypoints,
      const cv::Mat&                  /*Mask=cv::Mat()*/ ) const
  {
    Eigen::Vector4tArray points;

    // lmk_id, cam_id, u, v
    SparseSimData::Instance()->GetPoints(points);

    for (size_t ii = 0; ii < points.size(); ++ii) {
      //x, y, size, angle, response, octave, class_id
      if (cam_id_ == (unsigned int)points[ii][1]) {
        keypoints.push_back(cv::KeyPoint(points[ii][2], points[ii][3],
                            1, 0, 100000, 0, points[ii][0]));
      }
    }
  }

private:
  unsigned int cam_id_;
};

//bool FeatureHandler::Init(const Options& options) {
bool FeatureHandler::Init() {
  //m_Options = options;

  // Initialize the feature detector
  switch(CommonFrontEndConfig::getConfig()->getFeatureDetector()) {
    case common_front_end::CommonFrontEndParams_DOG:
      ROS_INFO("Using DOG feature detector");
      m_pFeatureDetector = std::shared_ptr<cv::FeatureDetector>(
          new cv::SurfFeatureDetector(CommonFrontEndConfig::getConfig()->getSurfHessianThreshold()));
      break;
    case common_front_end::CommonFrontEndParams_FAST:
      ROS_INFO("Using FAST feature detector");
      m_pFeatureDetector =
          std::shared_ptr<cv::FeatureDetector>(
              new FastPyramidDetector(CommonFrontEndConfig::getConfig()->getFastThreshold(),
                CommonFrontEndConfig::getConfig()->fastDoNonMaxSuppression(),
                CommonFrontEndConfig::getConfig()->fastSkipLevel0()) );
      break;
    default:
      std::cerr << " Feature detector not recognized. ";
      std::cerr << " valid: [FAST] " << std::endl;
      return false;
  }

  // Initialize descriptor extractor
  //switch (options.feature_descriptor)
  switch (CommonFrontEndConfig::getConfig()->getFeatureDescriptor())
  {
    case common_front_end::CommonFrontEndParams_PATCH:
      {
        ROS_INFO("Using PATCH descriptors");
        // No descriptor is needed, we are using patch-matching
        m_pDescriptorExtractor = NULL;
        break;
      }
    case common_front_end::CommonFrontEndParams_FREAK:
      {
        ROS_INFO("Using FREAK descriptors");
        m_pDescriptorExtractor =
            std::shared_ptr<cv::DescriptorExtractor>(
                new cv::FREAK(
                  CommonFrontEndConfig::getConfig()->freakOrientationNormalized(),
                  CommonFrontEndConfig::getConfig()->freakScaleNormalized(),
                  CommonFrontEndConfig::getConfig()->getFreakPatternScale(),
                  CommonFrontEndConfig::getConfig()->getFreakNOctaves()) );
        break;
      }
    case common_front_end::CommonFrontEndParams_SURF:
      {
        ROS_INFO("Using SURF descriptors");
        m_pDescriptorExtractor = std::shared_ptr<cv::DescriptorExtractor>( new cv::SURF(
            CommonFrontEndConfig::getConfig()->getSurfHessianThreshold(),
            CommonFrontEndConfig::getConfig()->getSurfNOctaves(),
            CommonFrontEndConfig::getConfig()->getSurfNOctaveLayers(),
            CommonFrontEndConfig::getConfig()->surfExtended(),
            CommonFrontEndConfig::getConfig()->surfUpright()) );
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
    if(CommonFrontEndConfig::getConfig()->getFeatureDetector() == common_front_end::CommonFrontEndParams_FAST)
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

/*DetectorType FeatureHandler::StrToDetectorType(std::string s) {
  if (!s.compare("FAST")) {
    return FAST;
  } else if(!s.compare("FLYBY")) {
    return FLYBY;
  } else if(!s.compare("TRACK_2D")) {
    return TRACK_2D;
  } else if (!s.compare("SIMULATION")) {
    return SIMULATION;
  } else if (!s.compare("DOG")) {
      return DOG;
  } else {
    return DET_UNDEFINED;
  }
}*/

////////////////////////////////////////////////////////////////////////////////
/*DescriptorType FeatureHandler::StrToDescriptorType(std::string s )
{
  if (!s.compare("PATCH")) {
    return PATCH;
  } else if (!s.compare("FREAK")) {
    return FREAK;
  } else if (!s.compare("SURF")) {
    return SURF;
  } else {
    return DES_UNDEFINED;
  }
}*/

////////////////////////////////////////////////////////////////////////////////
/*std::string FeatureHandler::DetectorTypeStr() const
{
  switch (m_Options.feature_detector) {
    case FAST: return "FAST";
    case FLYBY: return "FLYBY";
    case TRACK_2D: return "TRACK_2D";
    case SIMULATION: return "SIMULATION";
    case DOG: return "DOG";
    default: return "ERROR: Detector type not recognized";
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string FeatureHandler::DescriptorTypeStr() const
{
  switch (m_Options.feature_descriptor) {
    case PATCH: return "PATCH";
    case FREAK: return "FREAK";
    case SURF: return "SURF";
    default: return "ERROR: Descriptor type not recognized";
  }
}*/
