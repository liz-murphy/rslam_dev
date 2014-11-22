#include <rslam_engine/RslamEngine.h>

#include <unistd.h>
#include <sys/stat.h>

#include <calibu/cam/CameraRig.h>
#include <common/config.h>
#include <miniglog/logging.h>
#include <place_matching/PlaceMatcherFactory.h>
#include <pb_msgs/Matrix.h>
#include <pb_msgs/Pose.pb.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <slam_map/pose_measurement.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/SlamMap.h>
#include <slam_server/NodeSlamClient.h>
#include <slam_server/SlamServer.h>
#include <sparse_front_end/sparse_front_end.h>
#include <sparse_front_end/LiftLocalMap.h>
#include <utils/MathTypes.h>
#include <utils/PoseHelpers.h>
#include <utils/Utils.h>
#include <back_end/back_end.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <semidense_front_end/semi_dense_frontend.h>
#include <optimization/OptimizationParamsConfig.h>

unsigned int g_skip_nframes = 0;
static int g_debug_level  = 1;

static const std::string places_log = "places.log";
static const std::string frontend_log = "frontend.log";

using namespace rslam;
RslamEngine::RslamEngine() : images_(pb::ImageArray::Create()),
  is_mono_tracking_(false),
  is_tracking_2d_(false),
  is_rectified_(false),
  is_continuing_(false),
  have_imu_(false),
  persist_map_(false),
  active_camera_id_(-1),
  frame_number_(0),
  last_used_frame_number_(-1) {
    common_front_end_config_ = CommonFrontEndConfig::getConfig();
    optimization_config_ = OptimizationConfig::getConfig();
    feature_config_ = FeatureParams::getParams();
  }

RslamEngine::~RslamEngine() {}

// Reset member variables.
void RslamEngine::ResetVars() {
  // Clear data structures
  rois_.clear();
  rectified_frames_.clear();

  // Reset pointers
  frontend_.reset();
  place_matcher_.reset();
  map_.reset();
  timer_.reset();
  backend_.reset();

  // Set default config values
  is_mono_tracking_  = false;
  is_tracking_2d_    = false;
  is_rectified_      = false;
  is_continuing_     = false;
  have_imu_          = false;
  persist_map_       = false;
  active_camera_id_  = -1;
}

// Print app  info
void RslamEngine::PrintAppInfo() {
  ROS_INFO("RslamEngine - Initial State.");
  ROS_INFO("Feature detector: %s", FeatureParams::getParams()->getFeatureDetectorStr().c_str());
  ROS_INFO("Work dir: %s", working_directory_.c_str());
  ROS_INFO("Map persistence: %s", persist_map_ ? "true." : "false.");
  ROS_INFO("Using imu: %s", have_imu_ ? "true." : "false.");
  if (map_) {
    ROS_INFO("Building map %s", map_->id().uuid);
  }
}

bool RslamEngine::Reset(const RslamEngineOptions& options,
    const calibu::CameraRigT<Scalar>& rig_in) {
  // Cleanup and initialization of config variables
  common_front_end_config_ = CommonFrontEndConfig::getConfig();
  optimization_config_ = OptimizationConfig::getConfig();
  feature_config_ = FeatureParams::getParams();
  ResetVars();
  // If we are using TRACK_2D, we only need the first camera.
  working_directory_ = options.working_dir;
#ifdef ANDROID
  if (working_directory_.empty()) {
    working_directory_ = "/sdcard/rslam";
  }
#endif  // ANDROID

  if (!working_directory_.empty()) {
    mkdir(working_directory_.c_str(), S_IRWXU);
    if (chdir(working_directory_.c_str())) {
      ROS_ERROR("Could not change working directory to %s",working_directory_.c_str());
    } else {
      ROS_INFO("Changed working directory to %s", working_directory_.c_str());
    }
  }

  have_imu_ = options.imu;
  is_continuing_ = options.continue_run;

  // Initialize drivers
  if (!InitResetCameras(options, rig_in)) {
    return false;
  }

  // Create new map, timer, frontEnd and place matcher objects
  timer_ = std::make_shared<Timer>();
  map_   = options.place_matcher_options.map ?
    options.place_matcher_options.map : std::make_shared<SlamMap>();
  if ((persist_map_ = options.persistent_map)) {
    map_->InitWithPersistence(options.map_file, is_continuing_);
  } else {
    map_->InitInMemory(options.single_track ?
        SlamMap::kSingleSessionStorage :
        SlamMap::kMultipleSessionStorage);
  }

  // Add current camera rig to the map
  SessionId mid = map_->id();
  _CameraRigPtr rig_ptr(new calibu::CameraRigT<Scalar>(rig_));
  map_->AddCamera(mid, rig_ptr);

  place_matcher_ = PlaceMatcherFactory::Create(options.place_matcher_options);

  /* Set up dynamic reconfigure */

  ros::NodeHandle nh_common_fe("/common_front_end");
  ros::NodeHandle nh_sparse_fe("/sparse_front_end");
  ros::NodeHandle nh_sd_fe("/semidense_front_end");
  ros::NodeHandle nh_FAST("/common_front_end/FAST");
  ros::NodeHandle nh_FREAK("/common_front_end/FREAK");
  ros::NodeHandle nh_SURF("/common_front_end/SURF");
  ros::NodeHandle nh_optimization("/optimization");

  common_frontend_dr_srv_.reset(new dynamic_reconfigure::Server<common_front_end::CommonFrontEndParamsConfig>(nh_common_fe));
  common_front_end_cb = boost::bind(&CommonFrontEndConfig::configCallback,common_front_end_config_, _1, _2);
  common_frontend_dr_srv_->setCallback(common_front_end_cb);

  FAST_dr_srv_.reset(new dynamic_reconfigure::Server<feature_utils::FASTConfig>(nh_FAST));
  FAST_cb = boost::bind(&FeatureParams::configFASTCallback, feature_config_, _1, _2);
  FAST_dr_srv_->setCallback(FAST_cb);

  FREAK_dr_srv_.reset(new dynamic_reconfigure::Server<feature_utils::FREAKConfig>(nh_FREAK));
  FREAK_cb = boost::bind(&FeatureParams::configFREAKCallback, feature_config_, _1, _2);
  FREAK_dr_srv_->setCallback(FREAK_cb);

  SURF_dr_srv_.reset(new dynamic_reconfigure::Server<feature_utils::SURFConfig>(nh_SURF));
  SURF_cb = boost::bind(&FeatureParams::configSURFCallback, feature_config_, _1, _2);
  SURF_dr_srv_->setCallback(SURF_cb);

  optimization_dr_srv_.reset(new dynamic_reconfigure::Server<::optimization::OptimizationParamsConfig>(nh_optimization));
  optimization_cb = boost::bind(&OptimizationConfig::configCallback, optimization_config_, _1, _2);
  optimization_dr_srv_->setCallback(optimization_cb);

  if (options.tracker_type_ == Tracker_Sparse) {
    ROS_INFO("Creating sparse front-end.");
    frontend_ = std::make_shared<rslam::sparse::SparseFrontEnd>();

    sparse_frontend_dr_srv_.reset(new dynamic_reconfigure::Server<sparse_front_end::SparseFrontEndConfig>(nh_sparse_fe));
    sparse_front_end_cb = boost::bind(&rslam::sparse::SparseFrontEnd::configCallback, std::static_pointer_cast<rslam::sparse::SparseFrontEnd>(frontend_), _1, _2);
    sparse_frontend_dr_srv_->setCallback(sparse_front_end_cb);
  } 
  else {
    ROS_INFO("Creating semi-dense front-end.");
    frontend_ = std::make_shared<SemiDenseFrontEnd>();

    sd_frontend_dr_srv_.reset(new dynamic_reconfigure::Server<semidense_front_end::SemiDenseConfig>(nh_sd_fe));
    sd_front_end_cb = boost::bind(&SemiDenseFrontEnd::configCallback, std::static_pointer_cast<SemiDenseFrontEnd>(frontend_), _1, _2);
    sd_frontend_dr_srv_->setCallback(sd_front_end_cb);
  }

  if (is_continuing_) {
    place_matcher_->Load(places_log);
    frontend_->Load(frontend_log);
  }

  backend_.reset(new backend::BackEnd);
  backend_->Init(map_);

  return true;
}

bool RslamEngine::Init(const std::shared_ptr<pb::ImageArray>& images) {
  images_ = images;
  
  LoadCurrentImages();

  frontend_->Init(rig_, images_, Timestamp(),
      have_imu_, map_, place_matcher_, timer_);

  first_frame_ = frontend_->current_frame()->id();

  backend_->Run();

  PrintAppInfo();
  return true;
}

// Naive statistic functions. why is this here?
Scalar mean(const std::vector<Scalar>& values) {
  Scalar sum = 0;
  for (Scalar s : values) {
    sum += s;
  }
  return sum / values.size();
}

Scalar stdev(const std::vector<Scalar>& values) {
  Scalar mu = mean(values);
  Scalar sum = 0;
  for (Scalar s : values) {
    sum += (s - mu) * (s - mu);
  }
  return std::sqrt(sum / values.size());
}

class SavePoseMapVisitor : public TransformMapVisitor {
  public:
    SavePoseMapVisitor(const std::string& filename) : pose_csv(filename) {
      static const std::string kHeader =
        "track_id,frame_id,timestamp,x,y,z,p,q,r";
      pose_csv << kHeader << std::endl;
    }

    bool Visit(const SlamFramePtr& cur_node) override {
      static const Eigen::IOFormat CsvFmt(
          Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
      TransformMapVisitor::Visit(cur_node);
      ReferenceFrameId id = cur_node->id();

      pose_csv << id.session_id.uuid << "," << id.id << ","
        << cur_node->time() << ","
        << T2Cart(CurT().matrix()).format(CsvFmt) << std::endl;
      return true;
    }

    std::ofstream pose_csv;
};

void RslamEngine::SaveFrames() const {
  SavePoseMapVisitor visitor("rslam_analytics.csv");
  visitor.set_depth(MapVisitor::kMaxDepth);
  visitor.set_root_id(first_frame());
  visitor.set_should_ignore_broken(true);
  map_->BFS(&visitor);
}

// Save data to disk.
void RslamEngine::Save() {
  ROS_INFO("Saving rslam data");
  map_->Save();
  place_matcher_->Save(places_log);
  frontend_->Save(frontend_log);
  SaveFrames();
  ROS_INFO("Finished saving rslam data");
}

// Get data timestamp.
double RslamEngine::Timestamp() {
    return images_->Timestamp();
}

void RslamEngine::LoadCurrentImages() {
  if (!is_mono_tracking_ && images_ && !images_->Empty()) {
    image_proc_.DoStereoBrightnessCorrection(
        images_->at(0)->Mat(), images_->at(1)->Mat(),
        rois_[0].data(), rois_[1].data());
  }
}

void RslamEngine::ImuCallbackHandler(const pb::ImuMsg& ref) {
  if (frontend_) {
    CHECK(ref.has_accel());
    CHECK(ref.has_gyro());

    Eigen::VectorXd accel, gyro;
    pb::ReadVector(ref.accel(), &accel);
    pb::ReadVector(ref.gyro(), &gyro);

    frontend_->RegisterImuMeasurement(gyro.cast<Scalar>(),
        accel.cast<Scalar>(),
        ref.device_time());
  }
}

void RslamEngine::PosysCallbackHandler(const pb::PoseMsg& ref) {
  if (!frontend_ || !ref.has_pose()) return;

  rslam::map::PoseMeasurement pose;
  pose.timestamp = ref.device_time();

  const auto& data = ref.pose().data();
  const auto& cov = ref.covariance().data();
  if (ref.type() == pb::PoseMsg::LatLongAlt) {
#ifdef HAVE_GEOCON
    /*    if (!lla2local_) {
          lla2local_.reset(geocon::geodetic2local::Create(
          data.Get(0), data.Get(1), data.Get(2)));
          }

          MSP::CCS::CartesianCoordinates cart;
          MSP::CCS::Accuracy acc;
          lla2local_->to_local(data.Get(0), data.Get(1), data.Get(2), cov.Get(0),
          &cart, &acc);
          pose.t_wv.translation() = {cart.x(), cart.y(), cart.z()};

    // Take the 90% error bound and convert it to
    Scalar spherical_std = acc.sphericalError90() / 1.64;
    pose.cov.diagonal().head<3>().setConstant(spherical_std * spherical_std);
    pose.cov.diagonal().tail<3>().setConstant(
    std::numeric_limits<Scalar>::max());*/
#endif
  } else {
    ROS_WARN("Could not register PoseMsg of type %d", ref.type());
    return;
  }

  frontend_->RegisterPoseMeasurement(pose);
}

bool RslamEngine::IterateBa() {
  frontend_->IterateBa();
  return true;
}

void RslamEngine::Iterate(const std::shared_ptr<pb::ImageArray>& images) {
  CHECK(images);
  images_ = images;

  if(common_front_end_config_->getConfig()->resetRequired())
  {
    // Something has changed in the front end that requires a reset (eg feature detector changed)
    //frontend_->reset();
    //common_front_end_config_->getConfig()->resetDone();
  }

  bool should_process = (frame_number_ > last_used_frame_number_ + g_skip_nframes);

  if (should_process) {
    LoadCurrentImages();

    frontend_->Iterate(images_, Timestamp());

    last_used_frame_number_ = frame_number_;
  }

  ++frame_number_;
}

bool RslamEngine::InitResetCameras(const RslamEngineOptions& options,
    const calibu::CameraRigT<Scalar>& rig_in) {
  CHECK(!rig_in.cameras.empty());
  // Initialize camera

  // @todo Add rectification handling code here
  calibu::CameraRigT<Scalar> crig = calibu::ToCoordinateConvention<Scalar>(
      rig_in, calibu::RdfRobotics.cast<Scalar>());

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
    ROS_INFO("Initializing in stereo mode... ");
    // detect if we are rectified
    Eigen::Vector3t fl =
      crig.cameras[0].T_wc.matrix().block<3,1>(0,1);
    Eigen::Vector3t fr =
      crig.cameras[1].T_wc.matrix().block<3,1>(0,1);

    double dot = fl.dot(fr);
    double angle;
    if (dot >= 1.0) {
      angle = 0.0;
    } else if (dot <= -1.0) {
      angle = M_PI;
    } else {
      angle = std::acos(fl.dot(fr));
    }
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
      ROS_INFO("Rectified cameras detected.");
      is_rectified_ = true;
    } else {
      ROS_INFO("Cameras are NOT rectified. Linear: %s, pl: %s, pr: %s, angle: %f",bLinear ? "True":"False", boost::lexical_cast<std::string>(pl.transpose()).c_str(), boost::lexical_cast<std::string>(pr.transpose()).c_str(), angle);
      is_rectified_ = false;
    }
  } else {
    ROS_INFO("Initializing in mono mode... ");
  }

  Sophus::SE3t M_rv;
  M_rv.so3() = calibu::RdfRobotics;
  for (calibu::CameraModelAndTransformT<Scalar>& model : crig.cameras) {
    model.T_wc = model.T_wc*M_rv;
  }
  ROS_INFO("Starting Tvs: %s",boost::lexical_cast<std::string>(crig.cameras[0].T_wc.matrix()).c_str());

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

    ROS_INFO("Valid ROI left camera: [%s]", boost::lexical_cast<std::string>(rois_[0].transpose()).c_str());
    ROS_INFO("Valid ROI right camera: [%s]",boost::lexical_cast<std::string>(rois_[1].transpose()).c_str());

    if (crig.cameras.size() == 2) {
      rig_ = crig;
    } else {
      ROS_ERROR("Camera models not initialized. ");
      return false;
    }

  } else {
    rig_.cameras.clear();
    if (active_camera_id_ < (int)crig.cameras.size()) {
      rig_.Add( crig.cameras[active_camera_id_]);
    } else {
      ROS_ERROR("Camera index out of bounds!");
      return false;
    }
  }

  for (size_t ii=0; ii < rig_.cameras.size(); ++ii) {
    ROS_INFO("Camera %d, Model: %s, Pose: %s", (int)ii,boost::lexical_cast<std::string>(rig_.cameras[ii].camera.K()).c_str(),
        boost::lexical_cast<std::string>(rig_.cameras[ii].T_wc.matrix()).c_str());
  }

  return true;
}

ReferenceFrameId RslamEngine::current_frame_id() const {
  if (!frontend_) return ReferenceFrameId();
  return frontend_->current_frame()->id();
}

/*void RslamEngine::GetCurrentKeypointsForDisplay(
    std::vector<std::vector<cv::KeyPoint> >* keypoints) {
  CHECK_NOTNULL(keypoints);
  if (frontend_) {
    frontend_->GetCurrentKeypointsForDisplay(*keypoints);
  }
}*/

bool RslamEngine::IsInitialized() const {
  if (frontend_) {
    return frontend_->IsInitialized();
  }
  return false;
}
