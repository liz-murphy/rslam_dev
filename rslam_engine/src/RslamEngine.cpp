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
#include <sparse_front_end/FrontEnd.h>
#include <sparse_front_end/FrontEndCVars.h>
#include <sparse_front_end/LiftLocalMap.h>
//#include <rhirdParty/CameraDrivers.h>
#include <utils/MathTypes.h>
#include <utils/PoseHelpers.h>
#include <utils/Utils.h>

#ifdef HAVE_SEMIDENSE_FRONTEND
#include <semidense_front_end/semi_dense_frontend.h>
#endif  // HAVE_SEMIDENSE_FRONTEND

//#ifdef HAVE_GEOCON
#include <geocon/geodetic2local.h>
//#endif  // HAVE_GECON

using namespace rslam;

unsigned int g_skip_nframes = 0;
static int g_debug_level  = 1;

static const std::string places_log = "places.log";
static const std::string frontend_log = "frontend.log";

RslamEngine::RslamEngine() : images_(pb::ImageArray::Create()),
                             is_using_sim_imu_(false),
                             is_using_sim_data_(false),
                             is_mono_tracking_(false),
                             is_tracking_2d_(false),
                             is_rectified_(false),
                             is_continuing_(false),
                             have_imu_(false),
                             persist_map_(false),
                             active_camera_id_(-1),
  frame_number_(0),
  last_used_frame_number_(-1) {
  CVarUtils::AttachCVar<unsigned int>("SkipNFrames", &g_skip_nframes );
  CVarUtils::AttachCVar<int>("ErrorLevel", &google::log_severity_global);
  CVarUtils::AttachCVar<int>("debug.RslamEngine", &g_debug_level );
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
  server_proxy_.reset();

  // Set default config values
  is_using_sim_imu_  = false;
  is_using_sim_data_ = false;
  is_mono_tracking_  = false;
  is_tracking_2d_    = false;
  is_rectified_      = false;
  is_continuing_     = false;
  have_imu_          = false;
  persist_map_       = false;
  active_camera_id_  = -1;
}

// Init / Reset simulated data
void RslamEngine::SetupSimulator(const RslamEngineOptions& options) {
  if (options.simulation) {
    if (simulator_) {
      simulator_->Restart();
    } else {
      simulator_ = SparseSimData::Instance();
      simulator_->ReadData(options.simulation_dir + "/points.csv",
                           options.simulation_dir + "/accel.csv",
                           options.simulation_dir + "/gyro.csv");
      simulator_->ReadGroundTruth(options.simulation_dir + "/poses.csv");
    }
    is_using_sim_imu_ = options.simulation_imu;
    is_using_sim_data_ = true;
    if (options.simulation_mono) {
      is_mono_tracking_ = true;
      active_camera_id_ = 0;
      g_frontend_cvars.use_only_camera_id = 0;
    } else {
      g_frontend_cvars.use_only_camera_id = -1;
    }
  }
}

// Print app  info
void RslamEngine::PrintAppInfo() {
  LOG(INFO) << "==============================================";
  LOG(INFO) << "RslamEngine - Initial State.";
  LOG(INFO) << "==============================================";
  LOG(INFO) << "Feat detector: "   << g_common_cvars.feature_detector;
  LOG(INFO) << "Work dir: "        << working_directory_;
  LOG(INFO) << "Map persistence: " << (persist_map_ ? "true." : "false.");
  LOG(INFO) << "Using imu: "       << (have_imu_ ? "true." : "false.");
  LOG(INFO) << "Using sim data: "  << (is_using_sim_data_ ? "true." : "false.");
  if (map_) {
    LOG(INFO) << "Building map "     << map_->id();
  }
  LOG(INFO) << "==============================================";
}

bool RslamEngine::Reset(const RslamEngineOptions& options,
                        const calibu::CameraRigT<Scalar>& rig_in) {
  // Cleanup and initialization of config variables
  ResetVars();
  // If we are using TRACK_2D, we only need the first camera.
  if (!g_common_cvars.feature_detector.compare("TRACK_2D")) {
    active_camera_id_ = 0;
  } else {
    active_camera_id_ = g_frontend_cvars.use_only_camera_id;
  }

  working_directory_ = options.working_dir;
#ifdef ANDROID
  if (working_directory_.empty()) {
    working_directory_ = "/sdcard/rslam";
  }
#endif  // ANDROID

  if (!working_directory_.empty()) {
    mkdir(working_directory_.c_str(), S_IRWXU);
    if (chdir(working_directory_.c_str())) {
      LOG(FATAL) << "Could not change working directory to "
                 << working_directory_;
    } else {
      LOG(INFO) << "Changed working directory to " << working_directory_;
    }
  }

  have_imu_ = options.imu;
  is_continuing_ = options.continue_run;
  SetupSimulator(options);

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
  if (options.tracker_type_ == Tracker_Sparse) {
    LOG(INFO) << "Creating sparse front-end.";
    frontend_ = std::make_shared<sparse::FrontEnd>();
  } else {
#ifdef HAVE_SEMIDENSE_FRONTEND
    LOG(INFO) << "Creating semi-dense front-end.";
    frontend_ = std::make_shared<SemiDenseFrontEnd>();
#else
    LOG(FATAL) << "Semi-dense front-end is not enabled in this build.";
#endif  // HAVE_SEMIDENSE_FRONTEND
  }

  if (options.use_server) {
    if (options.server_type == "local") {
      server_proxy_ = std::make_shared<SlamServerProxy>(
          std::make_shared<SlamServer>(place_matcher_, map_));
      frontend_->set_server_proxy(server_proxy_);
    } else if (options.server_type == "node") {
#ifdef HAVE_NODE
      server_proxy_ = std::make_shared<SlamServerProxy>(
          std::make_shared<NodeSlamClient>());
      frontend_->set_server_proxy(server_proxy_);
#else  // HAVE_NODE
      LOG(FATAL) << "Node SlamServer client is not available!";
#endif
    }
  }
  if (is_continuing_) {
    place_matcher_->Load(places_log);
    frontend_->Load(frontend_log);
  }
  return true;
}

bool RslamEngine::Init(const std::shared_ptr<pb::ImageArray>& images) {
  images_ = images;
  if (is_using_sim_data_) {
    LoadSimFrame();
  }
  LoadCurrentImages();

  frontend_->Init(rig_, images_, Timestamp(),
                  have_imu_, map_, place_matcher_, timer_,
                  is_using_sim_data_);
  first_frame_ = frontend_->current_frame()->id();
  PrintAppInfo();
  return true;
}

// Capture data from sensors.
bool RslamEngine::LoadSimFrame() {
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

  images_->Ref().Clear();
  for (unsigned int ii = 0; ii < rig_.cameras.size() ; ++ii) {
    pb::ImageMsg* img = images_->Ref().add_image();
    img->set_width(640);
    img->set_height(480);
    img->set_format(pb::PB_LUMINANCE);
    img->mutable_data()->resize(640*480);
    std::fill(img->mutable_data()->begin(), img->mutable_data()->end(), 0);
  }
  return success;
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

// Compute statistics at the end of a run if we have
// ground truth.
void RslamEngine::Finish() {
  if (!is_using_sim_data_ || sim_frames_.empty()) return;

  // Find frame matching frame 0
  SlamFramePtr frame0 = map_->GetFramePtr(sim_frames_.begin()->second);
  if (!frame0) return;

  // Lift map relative to frame 0
  LocalMap lifted;
  rslam::sparse::LiftAllPoses(map_.get(), frame0->id(), lifted);

  Sophus::SE3t true_pose;
  std::vector<Scalar> angular_errors = {0};
  std::vector<Scalar> translation_errors = {0};
  std::vector<Scalar> log_errors = {0};
  size_t num_frames = 0;

  // For every frame in the simulator data
  for (const std::pair<unsigned int, ReferenceFrameId>& sim_id : sim_frames_) {
    // Find matching frame in map
    // Skip if frame doesn't exist
    if (!lifted.poses.count(sim_id.second)) continue;

    const PoseContainerT& est_pose = lifted.poses[sim_id.second];
    if (!simulator_->GetPose(sim_id.first, true_pose)) continue;

    Sophus::SE3t error = est_pose.t_wp.inverse() * true_pose;

    angular_errors.push_back(Sophus::SO3t::log(error.rotationMatrix()).norm());
    translation_errors.push_back(error.translation().norm());
    log_errors.push_back(error.log().norm());

    ++num_frames;
  }

  // Write errors to results file as a YAML style properties file
  std::map<std::string, Scalar> data = {
    {"mean_angular_error", mean(angular_errors)},
    {"mean_translation_error", mean(translation_errors)},
    {"mean_log_error", mean(log_errors)},
    {"stdev_angular_error", stdev(angular_errors)},
    {"stdev_translation_error", stdev(translation_errors)},
    {"stdev_log_error", stdev(log_errors)},
  };

  std::ofstream fout(error_results_file_ + ".csv");
  std::string last_column = data.rbegin()->first;
  for (const auto& pair : data) {
    fout << pair.first << (pair.first != last_column ? "," : "");
  }
  fout << std::endl;

  for (const auto& pair : data) {
    fout << pair.second << (pair.first != last_column ? "," : "");
  }
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
  LOG(g_debug_level) << "Saving rslam data";
  map_->Save();
  place_matcher_->Save(places_log);
  frontend_->Save(frontend_log);
  SaveFrames();
  LOG(g_debug_level) <<  "Finished saving rslam data";
}

// Get data timestamp.
double RslamEngine::Timestamp() {
  if(is_using_sim_data_){
    return simulator_->Timestamp();
  }else{
    return images_->Timestamp();
  }
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
  if (!frontend_) return;

  rslam::map::PoseMeasurement pose;
  pose.timestamp = ref.device_time();

  const auto& data = ref.pose().data();
  const auto& cov = ref.covariance().data();
  if (ref.type() == pb::PoseMsg::LatLongAlt) {
#ifdef HAVE_GEOCON
    if (!lla2local_) {
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
        std::numeric_limits<Scalar>::max());
#endif
  } else {
    LOG(WARNING) << "Could not register PoseMsg of type " << ref.type();
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

  bool should_process = true;
  if (is_using_sim_data_) {
    should_process = LoadSimFrame();
  } else {
    should_process = (frame_number_ > last_used_frame_number_ + g_skip_nframes);
  }

  if (should_process) {
    LoadCurrentImages();
    if (is_using_sim_data_) {
      sim_frames_.insert(
          {simulator_->Frame(), frontend_->current_frame()->id()});
    }

    frontend_->Iterate(images_, Timestamp());

    last_used_frame_number_ = frame_number_;
  }

  ++frame_number_;
}

bool RslamEngine::InitResetCameras(const RslamEngineOptions& options,
                                   const calibu::CameraRigT<Scalar>& rig_in) {
  CHECK(!rig_in.cameras.empty());
  // Initialize camera
  if (is_using_sim_data_ && simulator_->HasIMU() && is_using_sim_imu_) {
    have_imu_ = true;
  }

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
    LOG(INFO) << "Initializing in stereo mode... ";
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
      LOG(INFO) << "Rectified cameras detected.";
      is_rectified_ = true;
    } else {
      LOG(INFO) << "Cameras are NOT rectified. Linear: "
                << bLinear << " pl: " << pl.transpose() << " pr: "
                << pr.transpose() << " angle: " << angle;
      is_rectified_ = false;
    }
  } else {
    LOG(INFO) << "Initializing in mono mode... ";
  }

  Sophus::SE3t M_rv;
  M_rv.so3() = calibu::RdfRobotics;
  for (calibu::CameraModelAndTransformT<Scalar>& model : crig.cameras) {
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
