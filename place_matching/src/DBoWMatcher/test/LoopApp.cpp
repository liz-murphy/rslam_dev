#include <calibu/utils/Xml.h>
#include <opencv2/opencv.hpp>
#include <Utils/GetPot>
#include <Utils/MathTypes.h>
#include <Utils/Utils.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Camera/Drivers/Rectify/RectifyDriver.h>
#include <ThirdParty/CameraDrivers.h>
#include <miniglog/logging.h>

#include "LoopApp.h"

LoopApp::LoopApp(): images_(pb::ImageArray::Create()),
  is_mono_tracking_(false), active_camera_id_(-1), is_using_sim_data_(false)
{
}

cv::Mat LoopApp::getImage()
{
  double ts, x, y;
  return getImage(ts, x, y);
}

cv::Mat LoopApp::getImage(double &ts, double &x, double &y)
{
  size_t n;
  {
    static size_t count = 0;
    n = count++;
  }

  ts = -1;

  cv::Mat im;
  if(Capture())
  {
    im = images_->at(0)->Mat();

    if(n < timestamps_.size())
    {
      ts = timestamps_[n];

      // look up the gt
      GroundTruth::const_iterator git = gt_.lower_bound(ts);
      if(git != gt_.end())
      {
        x = git->second.first;
        y = git->second.second;
      }
    }
  }

  return im;
}

void LoopApp::Init(GetPot &cl)
{
  InitResetCameras(cl);
}

// Copied from rslam/SlamApp
bool LoopApp::InitResetCameras(GetPot &cl) {
  LOG(INFO) << "Initializing drivers...";

  // Initialize camera
  hal::RectifyDriver *driver = nullptr;
  calibu::CameraRigT<Scalar> crig;

  {
    std::string cam = cl.follow("", "-cam");
#ifdef ANDROID
    if (cam.empty()) {
      cam = "kitkat://";
    }
#endif  // ANDROID

    hal::Uri uri(cam);
    if (uri.properties.Contains("startframe")) {
      start_frame_ = uri.properties.Get("startframe", start_frame_);
    } else {
      start_frame_ = 0;
      uri.properties.Set("startframe", start_frame_);
    }

    try {
      camera_device_ = hal::Camera( uri );
    }
    catch (hal::DeviceException& e) {
      LOG(ERROR) << "Error loading camera device: " << e.what();
      return false;
    }

    driver = camera_device_.GetDriver<hal::RectifyDriver>();
  }

  // see if the user has specified a scheme w rectification

  if (driver) {
    //                calibu::CameraRigT<Scalar> driverRig;
    //                // get camera models from driver
    //                calibu::CameraModelGeneric<Scalar> cmod;
    //                std::cout << pDriver->CameraModel().Type() << std::endl;
    //                cmod = pDriver->CameraModel();
    //                driverRig.Add(cmod, Sophus::SE3Group<Scalar>());
    //                driverRig.Add(cmod, pDriver->T_rl().inverse());
    //                crig = calibu::ToCoordinateConvention(driverRig, calibu::RdfRobotics );
    ////                crig = driverRig;
    //                m_bIsRectified = true;
  }else{
    // load camera models from file
#ifndef ANDROID
    std::string def_dir("");
    if (!is_using_sim_data_) {
      def_dir = camera_device_.GetDeviceProperty(hal::DeviceDirectory);
    }
    std::string src_dir = cl.follow(def_dir.c_str(), "-sdir");
#else
    std::string src_dir = cl.follow("/sdcard/", "-sdir");
#endif

    LOG(INFO) << "Loading timestamps and groundtruth if provided...";
    LoadTimestamps(src_dir + "/timestamps.txt");
    LoadGroundTruth(src_dir + "/groundtruth.csv");

    LOG(INFO) << "Loading camera models...";
    std::string cmod_file = cl.follow("cameras.xml", "-cmod");
    std::string filename = src_dir + "/" + cmod_file;
    LOG(INFO) << "Loading camera models from " << filename;

    calibu::CameraRigT<Scalar> xmlrig = calibu::ReadXmlRig(filename);
    if (xmlrig.cameras.empty()) {
      LOG(FATAL) << "XML Camera rig is empty!";
    }

    // quick hack to prevent people from using LUT models
    if (HasLookupTable( filename )) {
      LOG(ERROR) << "LUT models detected -- please recitify"
                 << " the images first (or use the rectify://"
                 << " driver in the uri)";
      return false;
    }
    crig =
        calibu::ToCoordinateConvention<Scalar>(xmlrig,
                                               calibu::RdfRobotics.cast<Scalar>());

    if (crig.cameras.size() == 1 || active_camera_id_ != -1) {
      is_mono_tracking_ = true;
      // If we have not specified which camera to track, track the first one.
      if (active_camera_id_ == -1) {
        active_camera_id_ = 0;
      }
    } else {
      is_mono_tracking_ = false;
    }

    if (!is_mono_tracking_) {
      LOG(INFO) << "Initializing in stereo mode... ";
      // detect if we are rectified
      Eigen::Vector3t fl =
          crig.cameras[0].T_wc.matrix().block<3,1>(0,1);
      Eigen::Vector3t fr =
          crig.cameras[1].T_wc.matrix().block<3,1>(0,1);
      double angle = acos( fl.dot(fr) );
      std::string sl = crig.cameras[0].camera.Type();
      std::string sr = crig.cameras[1].camera.Type();
      Eigen::Matrix<Scalar,Eigen::Dynamic,1> pl =
          crig.cameras[0].camera.GenericParams();
      Eigen::Matrix<Scalar,Eigen::Dynamic,1> pr =
          crig.cameras[1].camera.GenericParams();
      bool bLinear = (sl == "calibu_fu_fv_u0_v0" &&
                      sr == "calibu_fu_fv_u0_v0");
      bLinear |= (sl == "calibu_f_u0_v0" && sr == "calibu_f_u0_v0");
      if (angle < 1e-6  && bLinear && pl == pr) {
        LOG(INFO) << "Rectified cameras detected.";
        is_rectified_ = true;
      } else {
        LOG(INFO) << "Cameras are NOT rectified. Linear: "
                  << bLinear << " pl: " << pl.transpose() << " pr: "
                  << pr.transpose() << " angle: " << angle;
        is_rectified_ = false;
      }
      is_rectified_ = false;// tmp hack to test MIT
    } else {
      LOG(INFO) << "Initializing in mono mode... ";
    }
  }

  Sophus::SE3t M_rv;
  M_rv.so3() = calibu::RdfRobotics;
  for (CameraModelAndTransformT<Scalar>& model : crig.cameras) {
    model.T_wc = model.T_wc*M_rv;
  }
  LOG(INFO) << "Starting Tvs: " << crig.cameras[0].T_wc.matrix();

  // this is temporary and for calibration only
  //    for( CameraModelAndTransformT<Scalar>& model : crig.cameras){
  //        model.T_wc = Sophus::SE3Group<Scalar>();
  //    }
  // crig.cameras[0].camera.SetGenericParams(crig.cameras[0].camera.GenericParams()
  // + (Eigen::Matrix<Scalar,5,1>() << 50,-50,70,50,0.8).finished());

  if (!is_mono_tracking_) {
    rois_.resize(2);
    Eigen::Vector4t bounds;
    bounds = GetFrustrumBoundingBox( crig, 1, 0 );
    rois_[0] = bounds.cast<int>();
    bounds = GetFrustrumBoundingBox( crig, 0, 1 );
    rois_[1] = bounds.cast<int>();

    LOG(INFO) << "Valid ROI left camera: [" << rois_[0].transpose() << "]";
    LOG(INFO) << "Valid ROI right camera: [" << rois_[1].transpose() << "]";

    if (crig.cameras.size() == 2) {
      rig_ = crig;
    } else {
      LOG(ERROR) << "Camera models not initialized. ";
      return false;
    }

  } else {
    rig_.cameras.clear();
    if (active_camera_id_ < (int)crig.cameras.size()) {
      rig_.Add( crig.cameras[active_camera_id_]);
    } else {
      LOG(ERROR) << "Camera index out of bounds!";
      return false;
    }
  }

  for (size_t ii=0; ii < rig_.cameras.size(); ++ii) {
    LOG(INFO) << ">>>>>>>> Camera " << ii << ":"  << std::endl
              << "Model: " << std::endl
              << rig_.cameras[ii].camera.K() << std::endl
              << "Pose: " << std::endl
              << rig_.cameras[ii].T_wc.matrix();
  }

  return true;
}

// Copies from rslam/SlamApp
bool LoopApp::Capture() {
  /*
  if (is_using_sim_data_) {
    bool success = simulator_->Capture();
    // add imu
    if (success && simulator_->HasIMU() && is_using_sim_imu_) {
      Eigen::Vector4tArray accel;
      Eigen::Vector4tArray gyro;
      simulator_->GetImuData(accel, gyro);
      for (size_t ii=0; ii < accel.size(); ++ii) {
        frontend_->RegisterImuMeasurement(gyro[ii].tail(3),
                                          accel[ii].tail(3),
                                          accel[ii](0));
      }
    }

    for (int ii = 0; ii < rig_.cameras.size() ; ++ii) {
      pb::ImageMsg* img = images_->Ref().add_image();
      img->set_width(640);
      img->set_height(480);
      img->set_format(pb::PB_LUMINANCE);
      img->mutable_data()->resize(640*480);
      std::fill(img->mutable_data()->begin(), img->mutable_data()->end(), 0);
    }
    return success;
  }else
  */
  {
    return camera_device_.Capture(*images_);
  }
}

// Copied from rslam/SlamApp
bool LoopApp::HasLookupTable(const std::string& rig_file) {
  calibu::TiXmlDocument doc;
  if (doc.LoadFile(rig_file)) {
    calibu::TiXmlNode* pNode = doc.FirstChild("rig");
    if (pNode) {
      return pNode->FirstChildElement("lut");
    }
  }
  return false;
}

// Get opencv images from simulated data or from device.
void LoopApp::LoadCurrentImages() {
  if (!is_mono_tracking_ && !images_->Empty())
  {
    image_proc_.DoStereoBrightnessCorrection(
        images_->at(0)->Mat(), images_->at(1)->Mat(),
        rois_[0].data(), rois_[1].data());
  }
}

void LoopApp::LoadTimestamps(const std::string &filename)
{
  timestamps_.clear();

  std::fstream f(filename.c_str(), std::ios::in);
  if(f.is_open())
  {
    while(!f.eof())
    {
      std::string s;
      std::getline(f, s);

      if(!s.empty())
        timestamps_.push_back(DUtils::StringFunctions::fromString<double>(s));
    }
  }
}

void LoopApp::LoadGroundTruth(const std::string &file)
{
  gt_.clear();

  std::fstream f(file.c_str(), std::ios::in);
  if(f.is_open())
  {
    while(!f.eof())
    {
      std::string s;
      std::getline(f, s);

      if(!s.empty())
      {
        std::vector<std::string> tokens;
        DUtils::StringFunctions::split(s, tokens, ",");
        // there should be 4 tokens: timestamp, x, y, theta

        if(tokens.size() >= 3 )
        {
          double ts = DUtils::StringFunctions::fromString<double>(tokens[0]);
          double x = DUtils::StringFunctions::fromString<double>(tokens[1]);
          double y = DUtils::StringFunctions::fromString<double>(tokens[2]);

          gt_.add(ts, x, y);
        }
      }
    } // for each line
  } // if opened
}

void LoopApp::GroundTruth::add(double ts, double x, double y)
{
  if(empty())
  {
    minx = maxx = x;
    miny = maxy = y;
  }
  else
  {
    if(x < minx) minx = x;
    else if(x > maxx) maxx = x;
    if(y < miny) miny = y;
    else if(y > maxy) maxy = y;
  }

  insert(end(), std::make_pair(ts, std::make_pair(x,y)));
}

void LoopApp::getGroundTruthBounds(double &minx, double &maxx, double &miny,
                            double &maxy) const
{
  minx = gt_.minx;
  maxx = gt_.maxx;
  miny = gt_.miny;
  maxy = gt_.maxy;
}
