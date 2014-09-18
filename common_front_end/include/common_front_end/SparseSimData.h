#ifndef SPARSESIMDATA_H
#define SPARSESIMDATA_H

#include <string>
#include <map>
#include <vector>
#include <memory>
#include <utils/MathTypes.h>

class SparseSimData
{

  struct FrameSimData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Scalar timestamp;
    Sophus::SE3t pose;
    Eigen::Vector4tArray points;
    Eigen::Vector4tArray accel;
    Eigen::Vector4tArray gyro;
  };

public:
  SparseSimData():has_imu_(false){}
  ~SparseSimData(){}

  ///
  /// \brief ReadData - reads simulated 2d trajectories and imu measurements
  /// \param[in] file_name_points
  /// \param[in] file_name_accel
  /// \param[in] file_name_gyro
  ///
  bool ReadData(std::string file_name_points,
                std::string file_name_accel = "",
                std::string file_name_gyro  = "");


  bool ReadGroundTruth(std::string file_name_poses);


  bool Capture();

  void GetImuData(Eigen::Vector4tArray &accel,
                  Eigen::Vector4tArray &gyro);

  void GetPoints(Eigen::Vector4tArray &points);

  bool GetPose(unsigned int frame_id, Sophus::SE3t& pose);

  bool HasIMU() { return has_imu_; }
  ///
  /// \brief Reset all member variables
  ///
  void Reset();

  ///
  /// \brief Set pointer to the begining of the data
  ///
  void Restart();

  Scalar Timestamp() { return next_frame_->second.timestamp; }

  unsigned int Frame() {
    return next_frame_->first;
  }

  static std::shared_ptr<SparseSimData> Instance();

  template<typename T>
  T GetToken( std::stringstream& ss, char delimiter );

private:

  bool has_imu_;
  typedef std::map<unsigned int, FrameSimData> FrameSimMap;
  FrameSimMap::iterator next_frame_;
  FrameSimMap sim_frames_;

};

#endif // SPARSESIMDATA_H
