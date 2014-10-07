// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <memory>
#include <iostream>
#include <common_front_end/FastPyramidDetector.h>
#include <common_front_end/FeatureHandler.h>
#include <common_front_end/PatchMatch.h>
#include <common_front_end/SparseSimData.h>
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

bool FeatureHandler::Init(const Options& options) {
  m_Options = options;

  // Initialize the feature detector
  switch(options.feature_detector) {
    case DOG:
      ROS_INFO("Using DOG feature detector");
      m_pFeatureDetector = std::shared_ptr<cv::FeatureDetector>(
          new cv::SurfFeatureDetector(400));
      break;
    case FAST:
      ROS_INFO("Using FAST feature detector");
      m_pFeatureDetector =
          std::shared_ptr<cv::FeatureDetector>(
              new FastPyramidDetector(options.fast_threshold,
                                      options.fast_do_nms,
                                      options.fast_skip_level0));
      break;
   // case TRACK_2D:
    //  m_pFeatureDetector = std::make_shared<Track2dFeatureDetector>();
     // break;
    case SIMULATION:
      m_pFeatureDetector = std::shared_ptr<cv::FeatureDetector>(
          new SimFeatureDetector(options.sim_camera_id));
      break;
    /*case FLYBY:
      m_pFeatureDetector = std::make_shared<FlybyPyramidFeatureDetector>(
          options.fast_skip_level0);
      break;*/
    default:
      std::cerr << " Feature detector not recognized. ";
      std::cerr << " valid: [FAST] " << std::endl;
      return false;
  }

  // Initialize descriptor extractor
  switch (options.feature_descriptor)
  {
    case PATCH:
      {
        // No descriptor is needed, we are using patch-matching
        m_pDescriptorExtractor = NULL;
        break;
      }
    case FREAK:
      {
        m_pDescriptorExtractor =
            std::shared_ptr<cv::DescriptorExtractor>(
                new cv::FREAK(options.freak_is_rotation_invariant,
                              options.freak_is_scale_invariant,
                              options.freak_pattern_scale,
                              options.freak_num_octaves,
                              options.freak_selected_pairs));
        break;
      }
    case SURF:
      {
        m_pDescriptorExtractor = std::shared_ptr<cv::DescriptorExtractor>( new cv::SURF(
            options.surf_fHessianThreshold,
            options.surf_nOctaves,
            options.surf_nOctaveLayers,
            options.surf_bExtended,
            options.surf_bUpright ) );
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
  if(m_Options.feature_detector == FAST) {
    std::cout << "Detecting FAST\n";
    auto* ptr =
        reinterpret_cast<FastPyramidDetector*>(m_pFeatureDetector.get());
    ptr->SetImagePyramidPtr(pyramid);
    ptr->detect(pyramid->at(0), keypoints);
  } /*else if(m_Options.feature_detector == FLYBY) {
      auto* ptr =
          reinterpret_cast<FlybyPyramidFeatureDetector*>(m_pFeatureDetector.get());
      ptr->SetImagePyramidPtr(pyramid);
      ptr->detect(pyramid->at(0), keypoints);
  }*/ else {
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

DetectorType FeatureHandler::StrToDetectorType(std::string s) {
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
}

////////////////////////////////////////////////////////////////////////////////
DescriptorType FeatureHandler::StrToDescriptorType(std::string s )
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
}

////////////////////////////////////////////////////////////////////////////////
std::string FeatureHandler::DetectorTypeStr() const
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
}
