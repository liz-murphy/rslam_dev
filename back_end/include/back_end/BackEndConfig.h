#ifndef BACK_END_CONFIG_H
#define BACK_END_CONFIG_H

class BackEndConfig
{
  public:
    static BackEndConfig * getConfig();
    double adaptive_threshold;
    double damping_factor;
    bool error_increase_allowed;
    bool do_dogleg;
    double imu_weight;
    double imu_prior;
    double trust_region_init_size;
    int debug_level;
    int min_frames_for_imu;
    double gyro_sigma;
    double gyro_bias_sigma;
    bool do_imu_conditioning;
    double accel_sigma;
    double accel_bias_sigma;
    int min_lm_observations;
    int adaptive_depth_increase;
    double imu_visualization_time_extra;
  private:
    BackEndConfig()
    {
      adaptive_threshold = 0.1;
      damping_factor = 1.0;
      error_increase_allowed = false;
      do_dogleg = true;
      imu_weight = 1e3;
      imu_prior = 10.0;
      trust_region_init_size = 5.0;
      debug_level = 1;
      min_frames_for_imu = 15.0;
      gyro_sigma = IMU_GYRO_SIGMA;
      gyro_bias_sigma = IMU_GYRO_BIAS_SIGMA;
      do_imu_conditioning = true;
      accel_sigma = IMU_ACCEL_SIGMA;
      min_lm_observations = 3;
      adaptive_depth_increase = 30;
      imu_visualization_time_extra = 0.3;
    };
    static BackEndConfig * m_configInstance;
};

#endif
