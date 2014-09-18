
#include <common_front_end/SparseSimData.h>
#include <iostream>
#include <fstream>
#include <float.h>

std::shared_ptr<SparseSimData> g_sparse_sim;

////////////////////////////////////////////////////////////////////////////////
bool SparseSimData::ReadData(std::string file_name_points,
                             std::string file_name_accel,
                             std::string file_name_gyro)
{
  Reset();

  std::ifstream fin;

  //================================================================
  // read 2d points [timestamp frame_id lmk_id cam_id u v]
  //================================================================
  fin.open(file_name_points);
  if (!fin.is_open()) {
    std::cerr << "Failed opening file: " << file_name_points << std::endl;
    return false;
  }

  Eigen::Vector4t m;
  std::string line;
  char delimiter = ',';

  while (getline(fin,line)) {
    std::stringstream ss(line);
    Scalar timestamp = GetToken<Scalar>(ss, delimiter);
    int frame_id  = GetToken<unsigned int>(ss, delimiter);
    for (unsigned int ii=0; ii<4; ++ii) {
      m[ii] = GetToken<Scalar>(ss, delimiter);
    }
    auto frame = sim_frames_.find(frame_id);
    if (frame == sim_frames_.end()) {
      sim_frames_[frame_id].timestamp = timestamp;
    }
    sim_frames_[frame_id].points.push_back(m);
    //std::cout << timestamp << " " << frame_id << " " << m.transpose() << std::endl;
  }
  fin.close();
  next_frame_ = sim_frames_.end();

  //================================================================
  // read accelerometer [timestamp a0 a1 a2]
  // (we assume that imu measurements are stored in a consecutive
  // order)
  //================================================================
  if (!file_name_accel.empty()) {
    fin.open(file_name_accel);
    if (!fin.is_open()) {
      std::cerr << "Failed opening file: " << file_name_accel << std::endl;
      return false;
    }
    has_imu_ = true;
    Eigen::Vector4t m;
    auto it_frame = sim_frames_.begin();
    while (getline(fin,line) && it_frame != sim_frames_.end()) {
      std::stringstream ss(line);
      for (unsigned int ii=0; ii<4; ++ii) {
        m[ii] = GetToken<Scalar>(ss, delimiter);
      }
      it_frame->second.accel.push_back(m);
      if( m[0] > it_frame->second.timestamp){
        ++it_frame;
      }
    }
    fin.close();
  }

  //================================================================
  // read gyro [timestamp g0 g1 g2]
  //================================================================
  if(!file_name_gyro.empty()){
    fin.open(file_name_gyro);
    if (!fin.is_open()) {
      std::cerr << "Failed opening file: " << file_name_gyro << std::endl;
      return false;
    }
    Eigen::Vector4t m;
    auto it_frame = sim_frames_.begin();
    while (getline(fin,line) && it_frame != sim_frames_.end()) {
      std::stringstream ss(line);
      for (unsigned int ii=0; ii<4; ++ii) {
        m[ii] = GetToken<Scalar>(ss, delimiter);
      }
      it_frame->second.gyro.push_back(m);
      if( m[0] > it_frame->second.timestamp){
        ++it_frame;
      }
    }
    fin.close();
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool SparseSimData::ReadGroundTruth(std::string file_name_poses)
{
  std::ifstream fin;

  //================================================================
  // Read ground truth poses. Each pose is stored in its 4x4 matrix
  // matrix representation ( row major )
  //================================================================
  fin.open(file_name_poses);
  if (!fin.is_open()) {
    std::cerr << "Failed opening file: " << file_name_poses << std::endl;
    return false;
  }

  Eigen::Matrix4t m;
  std::string line;
  char delimiter = ',';

  unsigned int frame_id = 0;
  while (getline(fin,line)) {
    std::stringstream ss(line);
    for (unsigned int ii=0; ii<4; ++ii) {
      for (unsigned int jj=0; jj<4; ++jj) {
        m(ii,jj) = GetToken<Scalar>(ss, delimiter);
      }
    }
    sim_frames_[frame_id].pose = Sophus::SE3t(m);
    frame_id++;
  }
  fin.close();
  next_frame_ = sim_frames_.end();

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool SparseSimData::Capture()
{
  if (next_frame_ == sim_frames_.end()) {
    next_frame_ = sim_frames_.begin();
  } else {
    ++next_frame_;
  }

  if (next_frame_ == sim_frames_.end()) {
    return false;
  } else {
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////////
void SparseSimData::GetImuData(Eigen::Vector4tArray &accel,
                               Eigen::Vector4tArray &gyro)
{
  accel  = next_frame_->second.accel;
  gyro   = next_frame_->second.gyro;
}

////////////////////////////////////////////////////////////////////////////////
void SparseSimData::GetPoints(Eigen::Vector4tArray &points)
{
  points = next_frame_->second.points;
}

////////////////////////////////////////////////////////////////////////////////
bool SparseSimData::GetPose(unsigned int frame_id, Sophus::SE3t &pose)
{
  if (frame_id >= sim_frames_.size()) {
    pose = Sophus::SE3t();
    return false;
  } else {
    pose = sim_frames_[frame_id].pose;
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////////
void SparseSimData::Restart()
{
  next_frame_ = sim_frames_.end();
}


////////////////////////////////////////////////////////////////////////////////
void SparseSimData::Reset()
{
  has_imu_ = false;
  sim_frames_.clear();
  next_frame_ = sim_frames_.end();
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<SparseSimData> SparseSimData::Instance()
{
  if (!g_sparse_sim) {
    g_sparse_sim = std::make_shared<SparseSimData>();
  }
  return g_sparse_sim;
}

////////////////////////////////////////////////////////////////////////////////
template<typename T>
T SparseSimData::GetToken(std::stringstream &ss, char delimiter)
{
  std::string token;
  std::stringstream ss_token;
  getline(ss,token,delimiter);
  ss_token.str(token);
  T val;
  ss_token >> val;
  return val;
}
