#ifndef BACK_END_CONFIG_H
#define BACK_END_CONFIG_H

#include <optimization/OptimizationParamsConfig.h>

class OptimizationConfig
{
  public:
    static OptimizationConfig* getConfig(){
      if(!s_instance)
        s_instance = new OptimizationConfig;
      return s_instance;
    }

    void configCallback(optimization::OptimizationParamsConfig &config, uint32_t level)
    {
      adaptive_threshold_ = config.adaptive_threshold;
      damping_factor_ = config.damping_factor;
      error_increase_allowed_ = config.error_increase_allowed;
      do_dogleg_ = config.do_dogleg;
      imu_weight_ = config.imu_weight;
      imu_prior_ = config.imu_prior;
      trust_region_init_size_ = config.trust_region_init_size;
      min_frames_for_imu_ = config.min_frames_for_imu;
      gyro_sigma_ = config.gyro_sigma;
      gyro_bias_sigma_ = config.gyro_bias_sigma;
      do_imu_conditioning_ = config.do_imu_conditioning;
      accel_sigma_ = config.accel_sigma;
      accel_bias_sigma_ = config.accel_bias_sigma;
      min_lm_observations_ = config.min_lm_observations;
      adaptive_depth_increase_ = config.adaptive_depth_increase;
      imu_visualization_time_extra_ = config.imu_visualization_time_extra;
    }

  double getAdaptiveThreshold(){return adaptive_threshold_;}; 
  double getDampingFactor(){return damping_factor_;};
  bool errorIncreaseAllowed(){return error_increase_allowed_;};
  bool doDogleg(){return do_dogleg_;};
  double getIMUWeight(){return imu_weight_;};
  double getIMUPrior(){return imu_prior_;};
  double getTrustRegionInitSize(){return trust_region_init_size_;};
  int getMinFramesForIMU(){return min_frames_for_imu_;};
  double getGyroSigma(){return gyro_sigma_;};
  double getGyroBiasSigma(){return gyro_bias_sigma_;};
  bool doIMUConditioning(){return do_imu_conditioning_;}; 
  double getAccelSigma(){return accel_sigma_;};
  double getAccelBiasSigma(){return accel_bias_sigma_;};
  int getMinLMObservations(){return min_lm_observations_;};
  int getAdaptiveDepthIncrease(){return adaptive_depth_increase_;};
  double getIMUVisualizationTimeExtra(){return imu_visualization_time_extra_;};
  private:
    OptimizationConfig() :
      adaptive_threshold_(0.1),
      damping_factor_(1.0),
      error_increase_allowed_(false),
      do_dogleg_(true),
      imu_weight_(1e3),
      imu_prior_(10.0),
      trust_region_init_size_(5.0),
      min_frames_for_imu_(15),
      gyro_sigma_(IMU_GYRO_SIGMA),
      gyro_bias_sigma_(IMU_GYRO_BIAS_SIGMA),
      do_imu_conditioning_(true),
      accel_sigma_(IMU_ACCEL_SIGMA),
      min_lm_observations_(3),
      adaptive_depth_increase_(30),
      imu_visualization_time_extra_(0.3)
  {
  };

    double adaptive_threshold_;
    double damping_factor_;
    bool error_increase_allowed_;
    bool do_dogleg_;
    double imu_weight_;
    double imu_prior_;
    double trust_region_init_size_;
    int min_frames_for_imu_;
    double gyro_sigma_;
    double gyro_bias_sigma_;
    double do_imu_conditioning_;
    double accel_sigma_;
    double accel_bias_sigma_;
    int min_lm_observations_;
    int adaptive_depth_increase_;
    double imu_visualization_time_extra_;

    static OptimizationConfig *s_instance;

};

#endif
