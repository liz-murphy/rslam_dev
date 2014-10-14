// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <features/Features.h>
#include <pb_msgs/ImagePyramid.h>
#include <common_front_end/CommonFrontEndParamsConfig.h>

enum DetectorType
{
  FAST          = common_front_end::CommonFrontEndParams_FAST,
  DOG           = common_front_end::CommonFrontEndParams_DOG,
};

enum DescriptorType
{
  PATCH         = common_front_end::CommonFrontEndParams_PATCH,
  FREAK         = common_front_end::CommonFrontEndParams_FREAK,
  BRISK         = common_front_end::CommonFrontEndParams_BRISK,
  SURF          = common_front_end::CommonFrontEndParams_SURF,
  SIFT          = common_front_end::CommonFrontEndParams_SIFT
};

class FeatureHandler
{

public:

/*  struct Options
  {
    Options() :
      feature_detector(FAST),
      feature_descriptor(PATCH),
      ifeature_detector(common_front_end::CommonFrontEndParams_FAST),
      ifeature_descriptor(common_front_end::CommonFrontEndParams_PATCH),
      sim_camera_id(0),
      fast_threshold(15),
      fast_do_nms(true),
      fast_levels(10),
      fast_level_factor(0.8),
      fast_skip_level0(false),
      freak_is_rotation_invariant(true),
      freak_is_scale_invariant(true),
      freak_num_octaves(3),
      freak_pattern_scale(22.0f),
      freak_selected_pairs(std::vector<int>()),
      brisk_threshold(30),
      brisk_num_octaves(3),
      brisk_do_3d_refinement(true),
      brisk_is_rotation_invariant(true),
      brisk_is_scale_invariant(true),
      brisk_pattern_scale(1.0f),
      dog_num_features(0),
      dog_threshold( 0.04 ),
      dog_edge_threshold(10.0),
      dog_num_octaves( 3 ),
      dog_sigma(1.6),
      surf_fHessianThreshold(400),
      surf_nOctaves(4),
      surf_nOctaveLayers(2),
      surf_bExtended(false),
      surf_bUpright(true)
    {}

    DetectorType    feature_detector;
    DescriptorType  feature_descriptor;

    int ifeature_detector;
    int ifeature_descriptor;

    // simulation parameters
    unsigned int sim_camera_id;

    // fast detector parameters
    int     fast_threshold;
    bool    fast_do_nms;
    int     fast_levels;
    double  fast_level_factor;
    bool    fast_skip_level0;

    // freak descriptor extractor parameters
    bool  freak_is_rotation_invariant;
    bool  freak_is_scale_invariant;
    int   freak_num_octaves;
    float freak_pattern_scale;
    std::vector<int> freak_selected_pairs;
    // brisk detector parameters
    int   brisk_threshold;
    int   brisk_num_octaves;
    bool  brisk_do_3d_refinement;
    // brisk descriptor extractor parameters
    bool  brisk_is_rotation_invariant;
    bool  brisk_is_scale_invariant;
    float brisk_pattern_scale;

    // dog detector (using the openCV SIFT implementation)
    int    dog_num_features;
    double dog_threshold;
    double dog_edge_threshold;
    int    dog_num_octaves;
    double dog_sigma;
    //int    dog_nOctaveLayers;
    //int    dog_firstOctave;
    //int    dog_angleMode;

    float surf_fHessianThreshold;
    int   surf_nOctaves;
    int   surf_nOctaveLayers;
    bool  surf_bExtended;
    bool  surf_bUpright;

  };
*/
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
