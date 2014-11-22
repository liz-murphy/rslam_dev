#ifndef FEATURE_CONFIG_H
#define FEATURE_CONFIG_H

#include <feature_utils/FeatureConfig.h>
#include <feature_utils/FREAKConfig.h>
#include <feature_utils/SURFConfig.h>
#include <feature_utils/FASTConfig.h>

class FeatureParams
{
  public:
    enum {CANONICAL_PATCH_SIZE=9};

    std::string getFeatureDetectorStr()
    {
      switch(feature_detector_)
      {
        case feature_utils::Feature_DOG:
          return "SURF";
        case feature_utils::Feature_FAST:
          return "FAST";
        default:
          return "OOPS";
      }
    }

    static FeatureParams* getParams()
    {
      if(!m_configInstance)
        m_configInstance = new FeatureParams;
      return m_configInstance;
    }

    void configFeatureCalback(feature_utils::FeatureConfig &config, uint32_t level)
    {
      canonical_patch_size_ = config.canonical_patch_size;

      if(feature_detector_ != config.feature_detector)
        reset_required_ = true;

      if(feature_descriptor_ != config.feature_descriptor)
        reset_required_ = true;

      feature_detector_ = config.feature_detector;
      feature_descriptor_ = config.feature_descriptor;

      pyramid_levels_ = config.pyramid_levels;
      pyramid_level_factor_ = config.pyramid_level_factor;

      match_error_threshold_ = config.match_error_threshold;
      match_error_factor_ = config.match_error_factor;

      feature_matching_threshold_ = config.feature_matching_threshold;
    }

    void configFASTCallback(feature_utils::FASTConfig &config, uint32_t level)
    {
      fast_threshold_ = config.fast_threshold;
      fast_do_nms_ = config.do_nms;
      fast_skip_level0_ = config.skip_level0;
      reset_required_ = true; // any changes to features need a reset
    }

    void configFREAKCallback(feature_utils::FREAKConfig &config, uint32_t level)
    {
      freak_orientation_normalized_ = config.orientation_normalized;
      freak_scale_normalized_ = config.scale_normalized;
      freak_pattern_scale_ = config.pattern_scale;
      freak_n_octaves_ = config.n_octaves;

      // If these change you need to reset tracking ...
      reset_required_ = true;
    }

    void configSURFCallback(feature_utils::SURFConfig &config, uint32_t level)
    {
      surf_hessian_threshold_ = config.hessian_threshold;
      surf_n_octaves_ = config.n_octaves;
      surf_n_octave_layers_ = config.n_octave_layers;
      surf_extended_ = config.extended;
      surf_upright_ = config.upright;
      reset_required_ = true;
    }

    double getSurfHessianThreshold(){return surf_hessian_threshold_;};
    int getSurfNOctaves(){return surf_n_octaves_;};
    int getSurfNOctaveLayers(){return surf_n_octave_layers_;};
    bool surfExtended(){return surf_extended_;};
    bool surfUpright(){return surf_upright_;};


    int getFastThreshold(){return fast_threshold_;};
    bool fastDoNonMaxSuppression(){return fast_do_nms_;};
    bool fastSkipLevel0(){return fast_skip_level0_;};

    bool freakOrientationNormalized(){return freak_orientation_normalized_;};
    bool freakScaleNormalized(){return freak_scale_normalized_;};
    float getFreakPatternScale(){return freak_pattern_scale_;};
    int getFreakNOctaves(){return freak_n_octaves_;};

    // general feature tracking stuff
    int getFeatureDetector(){return feature_detector_;};
    int getFeatureDescriptor(){return feature_descriptor_;};

    int getPyramidLevels(){return pyramid_levels_;};
    double getPyramidLevelFactor(){return pyramid_level_factor_;};

    double getMatchErrorThreshold(){return match_error_threshold_;};
    double getMatchErrorFactor(){return match_error_factor_;};

    int getFeatureMatchingThreshold(){return feature_matching_threshold_;};


    // have any of the parameters that have been modified caused a need to restart the front end
    bool resetRequired(){return reset_required_;};
    void resetDone(){reset_required_ = false;};

  private:
    FeatureParams(){
      fast_threshold_ = 20;
      fast_do_nms_ =true;
      fast_skip_level0_=false;

      freak_orientation_normalized_ = true;
      freak_scale_normalized_ = true;
      freak_pattern_scale_ = 22.0;
      freak_n_octaves_ = 4;

      surf_hessian_threshold_ = 400.0;
      surf_n_octaves_ = 4;
      surf_n_octave_layers_ = 2;
      surf_extended_ = true;
      surf_upright_ = false;

      pyramid_levels_ = 4;
      pyramid_level_factor_ = 0.8;

      feature_matching_threshold_ = 100; 

      feature_detector_=feature_utils::Feature_FAST;
      feature_descriptor_=feature_utils::Feature_PATCH;

      match_error_threshold_=30.0;
      match_error_factor_=1.2;


    };
    // FAST feature detection
    int fast_threshold_;
    bool fast_do_nms_;
    bool fast_skip_level0_;

    // freak descriptors
    bool freak_orientation_normalized_;
    bool freak_scale_normalized_;
    float freak_pattern_scale_;
    int freak_n_octaves_;

    // surf descriptors
    double surf_hessian_threshold_;
    int surf_n_octaves_;
    int surf_n_octave_layers_;
    bool surf_extended_;
    bool surf_upright_;

    // other ...
    int feature_detector_;
    int feature_descriptor_;

    int pyramid_levels_;
    double pyramid_level_factor_;

    int canonical_patch_size_;

    double match_error_threshold_;
    double match_error_factor_;

    int feature_matching_threshold_;

    bool reset_required_;

    static FeatureParams* m_configInstance;
};

#endif
