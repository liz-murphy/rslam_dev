// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <optimization/optimization.h>

#include <unordered_set>
#include <unordered_map>

#include <optimization/chi2inv.h>
#include <optimization/GatherLandmarkFramesRefineMapVisitor.h>
#include <optimization/ParentChainRefineMapVisitor.h>
#include <optimization/PushMap.h>
#include <optimization/OptimizationConfig.h>
#include <slam_map/SlamMap.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <common_front_end/Triangulation.h>
#include <utils/MathTypes.h>
#include <ros/ros.h>

namespace rslam {
  namespace optimization {

Optimization::Optimization() :
  is_running_(false),
  map_(nullptr) {
  Clear();
  //ba::debug_level_threshold = google::log_severity_global;
  //ba::debug_level = OptimizationConfig::getConfig()->debug_level;
}

Optimization::~Optimization() {}

void Optimization::Clear() {
  landmarks_.clear();
  pose_ids_.clear();
}

void Optimization::RegisterImuMeasurement(const Eigen::Vector3t &w,
                                     const Eigen::Vector3t &a,
                                     const double time) {
  ROS_DEBUG_NAMED("optimization", "Registering IMU measurement @t=%f with w = %s and a = %s", 
      time, 
      boost::lexical_cast<std::string>(w.transpose()).c_str(),
      boost::lexical_cast<std::string>(a.transpose()).c_str());

  imu_buffer_.AddElement(ba::ImuMeasurementT<Scalar>(w, a, time));
}

void Optimization::Init(const std::shared_ptr<SlamMap>& pMap) {
  map_ = pMap;
  last_min_pose_id_ = ReferenceFrameId();
}

void Optimization::IntegrateLastPose(
    const double start_time,
    const double end_time,
    const Eigen::Vector3t v_r,
    const Eigen::Vector3t g_r,
    const Eigen::Vector6t b,
    Sophus::SE3t& t_ab,
    Eigen::Vector3t& g_r_new,
    Eigen::Vector3t& v_r_new,
    std::vector<ba::ImuMeasurementT<Scalar>>& meas) {
  ba::ImuPoseT<Scalar> start_pose(Sophus::SE3t(), v_r, Eigen::Vector3t::Zero(),
                                  start_time);
  std::vector<ba::ImuPoseT<Scalar>> poses;
  meas = imu_buffer_.GetRange(start_time, end_time);
  ba::ImuPoseT<Scalar> new_pose =
      ba::ImuResidualT<Scalar>::IntegrateResidual(start_pose, meas,
                                                  b.head<3>(),
                                                  b.tail<3>(),
                                                  g_r, poses);

  t_ab = new_pose.t_wp;
  v_r_new =  new_pose.t_wp.so3().inverse() * new_pose.v_w;
  g_r_new =  new_pose.t_wp.so3().inverse() * g_r;

  ROS_DEBUG_NAMED("optimization", "Integrating last pose from %f to %f with x_ab: %s", 
      start_time,  
      end_time, 
      boost::lexical_cast<std::string>(t_ab.translation().transpose()).c_str());

  // current_pose_ = newPose;
}

void Optimization::Solve(const unsigned int /*uMaxIter*/,
                    const double* /*pRootParameterBlock*/) {}

template <typename BundleAdjusterT>
class LoadRelaxationMapVisitor : public TransformMapVisitor {
 public:
  LoadRelaxationMapVisitor(BundleAdjusterT* ba,
                           std::map<ReferenceFrameId, unsigned int>* pose_ids,
                           const std::shared_ptr<SlamMap>& map)
      : pose_ids_(pose_ids), ba_(ba), map_(map) {
    set_has_explore_edge(true);
  }

  void ExploreEdge(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    const Sophus::SE3t& curT = CurT();
    ReferenceFrameId parent_id = parent->id();
    ReferenceFrameId child_id = child->id();

    unsigned int ba_id_parent, ba_id_child;
    const auto it_parent = pose_ids_->find(parent_id);
    if (it_parent == pose_ids_->end()) {
      ba_id_parent = ba_->AddPose(curT, parent_id == root_id() ? false : true);

      pose_ids_->emplace(parent_id, ba_id_parent);
    } else {
      ba_id_parent = it_parent->second;
    }

    // Transform from parent to child
    Sophus::SE3t t_pc;
    if (!edge->transform(parent_id, child_id, t_pc)) {
      ROS_ERROR("Could not get transform between parent %s and child %s from edge %s",
                 boost::lexical_cast<std::string>(parent_id).c_str(),
                 boost::lexical_cast<std::string>(child_id).c_str(),
                 boost::lexical_cast<std::string>(edge->id()).c_str());
    }

    const auto it_child = pose_ids_->find(child_id);
    if (it_child == pose_ids_->end()) {
      ba_id_child = ba_->AddPose(curT * t_pc, true);
      pose_ids_->emplace(child_id, ba_id_child);
    } else {
      ba_id_child = it_child->second;
    }
    ba_->AddBinaryConstraint(ba_id_parent, ba_id_child, t_pc);
  }

  bool Visit(const SlamFramePtr& cur_node) override {
    TransformMapVisitor::Visit(cur_node);

    ReferenceFrameId frame_id = cur_node->id();
    if (!loaded_cameras_.count(frame_id.session_id)) {
      loaded_cameras_.insert(frame_id.session_id);
      CameraRigPtr rig = map_->GetCamera(frame_id.session_id);
      if (!rig) {
        ROS_ERROR("Could not find rig for %s",boost::lexical_cast<std::string>(frame_id).c_str());
      } else {
        for (unsigned int i = 0; i < rig->cameras.size(); ++i) {
          ba_->AddCamera(rig->cameras[i].camera, rig->cameras[i].T_wc);
        }
      }
    }
    return true;
  }

 private:
  std::map<ReferenceFrameId, unsigned int>* pose_ids_;
  std::unordered_set<SessionId> loaded_cameras_;
  BundleAdjusterT* ba_;
  std::shared_ptr<SlamMap> map_;
};

template <typename BundleAdjusterT>
class PushBAMapVisitor : public MapVisitor {
 public:
  PushBAMapVisitor(BundleAdjusterT* ba,
                   std::map<ReferenceFrameId, unsigned int>* pose_ids,
                   const std::shared_ptr<SlamMap>& map)
      : pose_ids_(pose_ids), ba_(ba), map_(map) {
    set_has_explore_edge(true);
  }

  void ExploreEdge(const SlamFramePtr& parent,
                   const SlamEdgePtr& edge,
                   const SlamFramePtr& child) override {
    auto parent_it = pose_ids_->find(parent->id());
    auto child_it = pose_ids_->find(child->id());
    auto end = pose_ids_->end();
    if (parent_it != end && child_it != end) {
      Sophus::SE3t t_pc = ba_->GetPose(parent_it->second).t_wp.inverse() *
          ba_->GetPose(child_it->second).t_wp;
      if (edge->id().start == parent->id()) {
        edge->set_transform(t_pc);
      } else {
        edge->set_transform(t_pc.inverse());
      }
    }
  }

 private:
  std::map<ReferenceFrameId, unsigned int>* pose_ids_;
  std::unordered_set<SessionId> loaded_cameras_;
  BundleAdjusterT* ba_;
  std::shared_ptr<SlamMap> map_;
};

void Optimization::RelaxMap(unsigned int depth,
                       const ReferenceFrameId& root_id,
                       unsigned int max_iter) {
  ROS_DEBUG_NAMED("optimization","RelaxMap");
  Clear();

  ba::Options<Scalar> options;
  ba_.Init(options, depth);
  ROS_DEBUG_NAMED("optimization","Loading poses");

  LoadRelaxationMapVisitor<decltype(ba_)> loader(&ba_, &pose_ids_, map_);
  loader.set_depth(depth);
  loader.set_root_id(root_id);
  loader.set_should_ignore_broken(true);
  map_->BFS(&loader);

  if(!pose_ids_.count(root_id)) 
    return;

  ba_.SetRootPoseId(pose_ids_[root_id]);

  ROS_DEBUG_NAMED("optimization","Done loading poses");

  ROS_DEBUG_NAMED("optimization","Solving map relaxation");
  ba_.Solve(max_iter);
  ROS_DEBUG_NAMED("optimization","Done solving map relaxation");

  ROS_DEBUG_NAMED("optimization","Pushing map");

  PushBAMapVisitor<decltype(ba_)> pusher(&ba_, &pose_ids_, map_);
  pusher.set_depth(depth);
  pusher.set_root_id(root_id);
  pusher.set_should_ignore_broken(true);
  map_->BFS(&pusher);
  ROS_DEBUG_NAMED("optimization","Done pushing map");

  ROS_DEBUG_NAMED("optimization","RelaxMap");
}

bool Optimization::_AddImuResidualToEdge(const SlamEdgePtr &edge,
                                    LocalMap& local_map,
                                    const double weight,
                                    unsigned int* ba_id) {
  CHECK_NOTNULL(ba_id);
  const TransformEdgeId edge_id = edge->id();
  const auto ita = pose_ids_.find(edge_id.start);
  const auto itb = pose_ids_.find(edge_id.end);
  ROS_DEBUG_NAMED("optimization","Adding imu residual between %s and %s with weight %f", 
      boost::lexical_cast<std::string>(edge_id.start).c_str(), 
      boost::lexical_cast<std::string>(edge_id.end).c_str(), 
      weight);

  if (ita == pose_ids_.end() || itb == pose_ids_.end()) {
    ROS_ERROR("Pose not found when attempting to add imu residual to edge.");
    return false;
  }

  auto start_it = local_map.poses.find(edge_id.start);
  auto end_it = local_map.poses.find(edge_id.end);

  if (start_it == local_map.poses.end()) {
    ROS_ERROR("When attempting to add imu residual, start pose not in ba: %s",
               boost::lexical_cast<std::string>(edge->id()).c_str());
    return false;
  } else if (end_it == local_map.poses.end()) {
    ROS_ERROR("When attempting to add imu residual, end pose not in ba: %s",
               boost::lexical_cast<std::string>(edge->id()).c_str());
    return false;
  }

  const PoseContainerT& start_pose = start_it->second;
  const PoseContainerT& end_pose = end_it->second;
  if (end_pose.time <= start_pose.time) {
    ROS_ERROR("IMU timestamps not sequential on edge %s at times %s to %s",
               boost::lexical_cast<std::string>(edge_id).c_str(),
               boost::lexical_cast<std::string>(start_pose.time).c_str(),
               boost::lexical_cast<std::string>(end_pose.time).c_str());
    return false;
  }

  std::vector<ba::ImuMeasurementT<Scalar> > meas =
      imu_buffer_.GetRange(start_pose.time, end_pose.time);
  if (meas.empty()) {
    ROS_ERROR("No measurements in residual from %f to %f. Buffer start t: %f, end time: %f, num elements: %d",
               start_pose.time,
               end_pose.time,
               imu_buffer_.start_time,
               imu_buffer_.end_time,
               (int)imu_buffer_.elements.size());
    return false;
  }
  *ba_id = imu_ba_.AddImuResidual(start_pose.ba_id, end_pose.ba_id,
                                  meas, weight);
  return true;
}

OptimizationStatus Optimization::UpdateLocalMapFromBa(LocalMap &local_map,
                                                 const bool is_using_imu,
                                                 const bool remove_unused_landmarks,
                                                 const bool update_landmarks,
                                                 const bool calculate_adaptive_metrics,
                                                 const bool update_static_set) {
  bool anchor_t_wp_set = false;
  OptimizationStatus status = OptStatus_NoChange;
  std::vector<Eigen::Matrix<Scalar, 15, 1>> delta_vals;
  Eigen::Matrix<Scalar, 15, 1> root_delta;

  // now update the map from the solution
  ba::Tic();
  for (auto& pair: local_map.poses) {
    auto& local_pose = pair.second;
    if (local_pose.ba_id == UINT_MAX) {
      ROS_DEBUG_NAMED("optimization","Skipping update for pose %s since ba_id = -1", 
          boost::lexical_cast<std::string>(pair.first).c_str());
      continue;
    }

    const auto& ba_pose = GetPose(is_using_imu, local_pose.ba_id);
    if (ba_pose.is_active) {
      if (calculate_adaptive_metrics) {
        Eigen::Matrix<Scalar, 15, 1> delta;
        delta.head<3>() =
            ba_pose.t_wp.translation() - local_pose.t_wp.translation();
        delta.segment<3>(3) =
            (ba_pose.t_wp.so3() * local_pose.t_wp.so3().inverse()).log();
        delta.segment<3>(6) = ba_pose.v_w - local_pose.v_w;
        delta.segment<6>(9) = ba_pose.b - local_pose.b;

        // std::cerr << diff.transpose().format(ba::kCleanFmt) << std::endl;

        if (pair.first.id == min_active_pose_id_.id + 1) {
          root_delta = delta;
        }

        delta_vals.push_back(delta);


        // If this is the last pose, check the amount by which it is being
        // modified
        /*if (pair.first.id == min_active_pose_id_.id + 1) {
        // Get metrics on how much this pose has moved
        const double trans_diff = delta.head<3>().norm();
        const double rot_diff = delta.segment<3>(3).norm();

        const double vel_diff = (delta.segment<3>(6)).norm();
        const double bias_diff = (delta.segment<6>(9)).norm();


        // Indicate whether or not the changes to the parameters are over
        // one or more of the threholds
        if (trans_diff == 0 && rot_diff == 0) {
        status = OptStatus_NoChange;
        } else {
        // std::cerr << "bias for " << pair.first << " is " <<
        //               local_pose.b.transpose() << std::endl;

        if (trans_diff > g_translation_threshold ||
        rot_diff > g_rotation_threshold ||
        vel_diff > g_velocity_threshold ||
        bias_diff > g_bias_threshold) {
        // std::cerr << "Differences larger than threshold." << std::endl;
        status = OptStatus_DiffOverThreshold;
        } else {
        status = OptStatus_DiffBelowThreshold;
        }
        }
        }*/
      }

      local_pose.t_wp = ba_pose.t_wp;

      // Obtain the id of the "anchor pose". The pose after the
      // minimum pose which is used to obtain the relative
      // transformation between two consecutive "local maps".
      if (pair.first.id > min_pose_id_.id && !anchor_t_wp_set) {
        anchor_t_wp_set = true;
        anchor_id = pair.first;
        last_anchor_t_wp = local_pose.t_wp;
        ROS_DEBUG_NAMED("optimization","Setting anchor_t_wp to %s,  id: %s",
            boost::lexical_cast<std::string>(local_pose.t_wp.matrix()).c_str(), 
            boost::lexical_cast<std::string>(anchor_id).c_str());
      }

      if (is_using_imu) {
        local_pose.v_w = ba_pose.v_w;
        local_pose.b = ba_pose.b;

        local_pose.t_vs = ba_pose.t_vs;
        local_pose.cam_params = ba_pose.cam_params;
        // std::cerr << "bias for " << pair.first << " is " <<
        //               local_pose.b.transpose() << std::endl;
      }

      ROS_DEBUG_NAMED("optimization", "setting pose %s to %s with v: %s and b: %s from ba_id %s", 
          boost::lexical_cast<std::string>(pair.first).c_str(), 
          boost::lexical_cast<std::string>(local_pose.t_wp.matrix()).c_str(), 
          boost::lexical_cast<std::string>(local_pose.v_w.transpose()).c_str(), 
          boost::lexical_cast<std::string>(local_pose.b.transpose()).c_str(),
          boost::lexical_cast<std::string>(local_pose.ba_id).c_str()); 
    }
  }

  if (update_static_set && local_map.static_set_pose) {
    const auto& ba_pose = GetPose(is_using_imu,
                                  local_map.static_set_pose->ba_id);

    local_map.static_set_pose->t_wp = ba_pose.t_wp;

    // Update all the active-static boundary edges
    for (const TransformEdgeId& boundary : local_map.active_static_edges) {
      const ReferenceFrameId& static_frame_id =
          (boundary.start.session_id != local_map.map->id() ?
           boundary.start : boundary.end);
      if (static_frame_id == local_map.static_set_pose_id) continue;

      PoseContainerT& pose = local_map.poses[static_frame_id];
      pose.only_pose_optimized = true;
      pose.t_wp = (local_map.static_set_pose->t_wp *
                   local_map.static_poses[static_frame_id]);
    }
  }

  std::vector<LandmarkId> to_delete;

  // the map stores Xr in relative coordinates, so we can just update it from BA
  for (auto & pair: local_map.landmarks) {
    if (pair.second->ba_id != UINT_MAX && update_landmarks) {
      bool is_reliable = is_using_imu ?
          imu_ba_.IsLandmarkReliable(pair.second->ba_id) :
          ba_.IsLandmarkReliable(pair.second->ba_id);

      // promote the landmark to mono, if it was at infinity but was
      // also included in the bundle adjustment (which means
      // that it had enough observations for inclusion).
      if (pair.second->state == eLmkAtInfinity && is_reliable) {
        pair.second->state = eLmkMono;
      }
      // auto prev_w = pair.second.x_w;
      pair.second->x_w = is_using_imu ? imu_ba_.GetLandmark(pair.second->ba_id):
          ba_.GetLandmark(pair.second->ba_id);

      //std::cerr << " landmark optimized [ f:" << pair.first.ref_frame_id.id
      //          << " i: " << pair.first.landmark_index
      //          << " v: " << pair.first.track2d_id << " ]"
      //          << " from: " << pair.second.x_w.transpose();

      if (fabs(pair.second->x_w[3]) < 1e-6) {
        ROS_WARN("optimization: --> abs(inverse depth) < 1e-6, possible division by zero to follow");
      }
      pair.second->x_w /= pair.second->x_w[3];

      ROS_DEBUG_NAMED("optimization","setting lm %s to: %s", boost::lexical_cast<std::string>(pair.first).c_str(), boost::lexical_cast<std::string>(pair.second->x_w.transpose()).c_str());

    } else {
      // Remove this landmark from the local map so that it doesn't get updated
      // incorrectly when we push
      to_delete.push_back(pair.first);
    }
  }

  if (remove_unused_landmarks) {
    for (const LandmarkId& id : to_delete) {
      ROS_DEBUG_NAMED("optimization", "Deleting lm %s", boost::lexical_cast<std::string>(id).c_str());
      local_map.landmarks.erase(id);
    }
  }

  // Calculate update statistics if required.
  if (delta_vals.size() != 0 && calculate_adaptive_metrics) {
    Eigen::Matrix<Scalar, 15, 1> mean_delta, cov;
    mean_delta.setZero();
    for (Eigen::Matrix<Scalar, 15, 1>& diff_val : delta_vals) {
      mean_delta += diff_val;
    }
    mean_delta = mean_delta.array() / delta_vals.size();

    cov.setZero();
    for (Eigen::Matrix<Scalar, 15, 1>& delta_val : delta_vals) {
      //cov += (delta_val - mean_delta).cwiseProduct(c - mean_delta);
      cov += (delta_val).cwiseProduct(delta_val);
    }
    cov = cov.array() / (delta_vals.size() - 1);

    Eigen::Matrix<Scalar, 15, 1> mean_delta_norm, root_delta_norm;
    double mahalanobis_dist = 0;
    for (int ii = 0; ii < 15 ; ++ii) {
      const double difference = root_delta[ii] - mean_delta[ii];
      mahalanobis_dist += difference * (1.0/cov[ii]) * difference;

      mean_delta_norm[ii] = mean_delta[ii] * (1.0/cov[ii]) * mean_delta[ii];
      root_delta_norm[ii] = root_delta[ii] * (1.0/cov[ii]) * root_delta[ii];
    }

    ROS_DEBUG_NAMED("optimization", "mean delta norm: %f", mean_delta_norm.norm());
    ROS_DEBUG_NAMED("optimization", "root delta norm: %f,  root delta %s", 
        root_delta_norm.norm(),
        boost::lexical_cast<std::string>(root_delta_norm.transpose()).c_str());

    //    LOG(debug_level) << "diff: "  << mean.transpose() << std::endl;
    //    LOG(g_debug_level) << "cov: "  << cov.transpose() << std::endl;
    //    LOG(g_debug_level) << "diff_root: "  << root_diff.transpose() << std::endl;
    //    LOG(g_debug_level) << "Mahalanobis dis: " << mahalanobis_dist << std::endl;


    //if (mahalanobis_dist > 8.0) {
    //if (root_delta_norm.norm() > g_adaptive_threshold/*mean_delta_norm.norm()*/) {
    if (root_delta_norm.norm() > mean_delta_norm.norm() * OptimizationConfig::getConfig()->getAdaptiveThreshold()) {
      status = OptStatus_DiffOverThreshold;
    } else {
      status = OptStatus_DiffBelowThreshold;
    }
  }



  // const ba::ImuCalibrationT<Scalar>& calib = imu_ba_.GetImuCalibration();
  // LOG(g_debug_level) << "bg: " << calib.b_g.transpose() << " ba: " <<
  //             calib.b_a.transpose() << std::endl;

  //TODO: update the rig T_wc and params here
  return status;
}

uint32_t Optimization::AddPoseIntoBa(bool is_using_imu, bool is_active,
                                const PoseContainerT& pose) {
  if (is_using_imu) {
    return imu_ba_.AddPose(
        pose.t_wp, pose.t_vs, pose.cam_params, pose.v_w, pose.b, is_active);
  }
  return  ba_.AddPose(pose.t_wp, is_active);
}

uint32_t Optimization::AddLandmarkIntoBa(bool is_using_imu,
                                    bool is_active,
                                    uint32_t lm_ba_cam_id,
                                    uint32_t pose_id,
                                    const LandmarkContainer& lmk) {
  if (is_using_imu) {
    return imu_ba_.AddLandmark(lmk.x_w, pose_id, lm_ba_cam_id, is_active);
  }
  return ba_.AddLandmark(lmk.x_w, pose_id, lm_ba_cam_id, is_active);
}

uint32_t Optimization::AddProjectionResidualIntoBa(bool is_using_imu,
                                              const Eigen::Vector2t& z,
                                              uint32_t pose_id,
                                              uint32_t lmk_id,
                                              uint32_t z_ba_cam_id) {
  if (is_using_imu) {
    return imu_ba_.AddProjectionResidual(z, pose_id, lmk_id, z_ba_cam_id, 2.0);
  }
  return ba_.AddProjectionResidual(z, pose_id, lmk_id, z_ba_cam_id, 2.0);
}

const ba::PoseT<Scalar>& Optimization::GetPose(bool is_using_imu, uint32_t ba_id) {
  if (is_using_imu) {
    return imu_ba_.GetPose(ba_id);
  }
  return ba_.GetPose(ba_id);
}

bool Optimization::LoadPoseIntoBa(
    const LocalMap& local_map,
    const ReferenceFrameId &id,
    PoseContainerT &pose_container,
    bool is_active,
    bool is_using_imu,
    const std::map<SessionId, std::map<uint32_t, uint32_t> >& ba_cam_ids,
    bool init_new_landmarks) {

  ROS_DEBUG_NAMED("optimization","Analyzing pose %s for addition", boost::lexical_cast<std::string>(id).c_str());

  // First we must add this pose, if there are any measurements and/or ref
  // landmarks.
  if (pose_container.measurements.empty() &&
      pose_container.ref_landmarks.empty() && !is_using_imu) {
    return false;
  }

  // If we are initializing new landmarks, all poses are inactive.
  if (init_new_landmarks) {
    is_active = false;
  }

  // We set all poses from other maps to inactive
  if (id.session_id != local_map.map->id()) {
    is_active = false;
  }

  pose_ids_[id] = pose_container.ba_id =
      AddPoseIntoBa(is_using_imu, is_active, pose_container);

  ROS_DEBUG_NAMED("optimization", "Adding %s %s with meas.size %d and ref_lm.size %d and t_wp: %s and b: %s and ba_id: %s",
      (is_active ? "active pose: " : "inactive pose: "),
      boost::lexical_cast<std::string>(id).c_str(),
      (int)pose_container.measurements.size(),
      (int)pose_container.ref_landmarks.size(),
      boost::lexical_cast<std::string>(pose_container.t_wp.matrix()).c_str(),
      boost::lexical_cast<std::string>(pose_container.b.transpose()).c_str(),
      boost::lexical_cast<std::string>(pose_container.ba_id).c_str());

  bool is_lmks_active = !pose_container.is_static_set_pose;

  // Now add any landmarks that belong to this pose, if they pass the
  // tests.
  for (const std::shared_ptr<LandmarkContainer>& lmk_container :
           pose_container.ref_landmarks) {
    ROS_ERROR_COND(lmk_container->id.ref_frame_id != id, "Landmark %s has mismatched ref id to localmap ref id: %s",
        boost::lexical_cast<std::string>(lmk_container->id).c_str(),
        boost::lexical_cast<std::string>(id).c_str());

    auto cam_ids_it = ba_cam_ids.find(id.session_id);
    if (cam_ids_it == ba_cam_ids.end()) {
      ROS_WARN("Could not find cam id for %s",boost::lexical_cast<std::string>(id).c_str()); 
      continue;
    }

    auto lm_cam_id_it = cam_ids_it->second.find(lmk_container->base_camera_id);
    if (lm_cam_id_it == cam_ids_it->second.end()) {
      ROS_WARN("Could not find lm cam id for %d", lmk_container->base_camera_id);
      continue;
    }
    const unsigned int lm_ba_cam_id = lm_cam_id_it->second;

    // need at least two measurements
    bool test1 = (lmk_container->state == eLmkAtInfinity &&
                  lmk_container->measurements.size() >=
                  (size_t)OptimizationConfig::getConfig()->getMinLMObservations());

    bool test2 = (lmk_container->state != eLmkAtInfinity &&
                  lmk_container->measurements.size() > 1 &&
                  !init_new_landmarks);

    if (test1 || test2) {
      // If the tests were passed, add it to the problem.

      lmk_container->ba_id = AddLandmarkIntoBa(
          is_using_imu, is_lmks_active, lm_ba_cam_id,
          pose_container.ba_id, *lmk_container);
      ROS_DEBUG_NAMED("optimization","Adding landmark. %s with ba_id: %s state: %s",
          boost::lexical_cast<std::string>(lmk_container->id).c_str(),
          boost::lexical_cast<std::string>(lmk_container->ba_id).c_str(),
          LandmarkStateToString(lmk_container->state).c_str());
    }
  }

  // Add all measurements taken at this frame as constraints
  for (std::shared_ptr<MeasurementContainer> meas_container :
           pose_container.measurements) {
    ROS_ERROR_COND(meas_container->id.frame_id != id, "Meas %s has mismatched pose id: %s to local map ref id: %s",
        boost::lexical_cast<std::string>(meas_container->id).c_str(),
        boost::lexical_cast<std::string>(meas_container->id.frame_id).c_str(),
        boost::lexical_cast<std::string>(id).c_str());

    LandmarkId lm_id = meas_container->id.landmark_id;
    if (lm_id.ref_frame_id.session_id != local_map.map->id()) {
      continue;
    }

    auto lmk_it = local_map.landmarks.find(lm_id);
    ROS_ERROR_COND(lmk_it == local_map.landmarks.end(),"Measurement %s references landmark outside window.",
        boost::lexical_cast<std::string>(meas_container->id).c_str());

    const std::shared_ptr<LandmarkContainer>& lmk_container = lmk_it->second;
    if (lmk_container->ba_id == UINT_MAX) continue;

    auto cam_ids_it = ba_cam_ids.find(id.session_id);
    if (cam_ids_it == ba_cam_ids.end()) {
      ROS_WARN("Could not find cam id for %s",
          boost::lexical_cast<std::string>(id).c_str());
      continue;
    }

    auto z_cam_id_it = cam_ids_it->second.find(meas_container->cam_id);
    if (z_cam_id_it == cam_ids_it->second.end()) {
      ROS_WARN("Could not find z cam id for %s",
          boost::lexical_cast<std::string>(meas_container->cam_id).c_str());
      continue;
    }

    const unsigned int z_ba_cam_id = z_cam_id_it->second;

    //now add this measurement
    int res_id = AddProjectionResidualIntoBa(
        is_using_imu, meas_container->z, pose_container.ba_id,
        lmk_container->ba_id, z_ba_cam_id);

    ROS_DEBUG_NAMED("optimization","Adding projection residual with fid: %s, lm_id: %s, lmk_ba_id: %s, cam_id: %s, res_id: %s", 
        boost::lexical_cast<std::string>(meas_container->id.frame_id.id).c_str(),
        boost::lexical_cast<std::string>(lmk_container->id).c_str(),
        boost::lexical_cast<std::string>(lmk_container->ba_id).c_str(), 
        boost::lexical_cast<std::string>(meas_container->cam_id).c_str(),
        boost::lexical_cast<std::string>(res_id).c_str());
  }
  return true;
}

bool Optimization::LoadLocalMapIntoBa(LocalMap &local_map,
                                 const bool is_using_imu,
                                 calibu::CameraRigT<Scalar> &rig,
                                 const bool init_new_landmarks,
                                 const EdgeAttribute attribute)
{
  const std::map<SessionId, CameraRigPtr>& rigs = local_map.cameras;
  std::map<SessionId, std::map<unsigned int, unsigned int> > ba_cam_ids;

  min_active_pose_id_ = min_pose_id_ = ReferenceFrameId();
  min_active_pose_id_.session_id = min_pose_id_.session_id = map_->id();

  ba::Options<Scalar> options;
  options.trust_region_size = OptimizationConfig::getConfig()->getTrustRegionInitSize();
  options.gyro_sigma = OptimizationConfig::getConfig()->getGyroSigma();
  options.gyro_bias_sigma = OptimizationConfig::getConfig()->doIMUConditioning();
  options.accel_sigma = OptimizationConfig::getConfig()->getAccelSigma();
  options.accel_bias_sigma = OptimizationConfig::getConfig()->getAccelBiasSigma();

  // initialize the bundle adjuster
  if (is_using_imu && !init_new_landmarks) {
    imu_ba_.Init(options, local_map.poses.size(), local_map.num_measurements,
                 local_map.landmarks.size(), rig.cameras[0].T_wc);

    for (auto& rig_pair : rigs) {
      const CameraRigPtr rig_ptr = rig_pair.second;
      for (unsigned int ii=0; ii<rig_ptr->cameras.size(); ++ii) {
        ba_cam_ids[rig_pair.first][ii] =
            imu_ba_.AddCamera(rig_ptr->cameras[ii].camera,
                              rig_ptr->cameras[ii].T_wc);
      }
    }

    // now based on the global gravity, set the ba gravity direction
    imu_ba_.SetGravity(local_map.g_w);
  } else {
    ba_.Init(options, local_map.poses.size(), local_map.num_measurements,
             local_map.landmarks.size(), rig.cameras[0].T_wc);

    for (auto& rig_pair : rigs) {
      const CameraRigPtr rig_ptr = rig_pair.second;
      for (unsigned int ii=0; ii<rig_ptr->cameras.size(); ++ii) {
        ba_cam_ids[rig_pair.first][ii] =
            ba_.AddCamera(rig_ptr->cameras[ii].camera,
                          rig_ptr->cameras[ii].T_wc);
      }
    }
  }

  // Set all landmark ba_id values to invalid.
  for (auto & pair: local_map.landmarks) {
    pair.second->ba_id = UINT_MAX;
  }

  ba::Tic();

  // LOG(g_debug_level) << "Inserting poses." << std::endl;
  // Go through every pose in the local map.
  for (auto it = local_map.poses.begin(); it != local_map.poses.end(); ++it) {
    CHECK_EQ(it->first.session_id, local_map.map->id());

    // Poses on the inside are always active.
    LoadPoseIntoBa(local_map, it->first, it->second,
                   !local_map.outside_pose_ids.count(it->first),
                   is_using_imu, ba_cam_ids, init_new_landmarks);
  }

  if (pose_ids_.empty()) {
    ROS_DEBUG_NAMED("optimization","pose_ids_ is empty. Returning false.");
    return false;
  }

  // This is the last pose of the parent chain, so it is both the root
  // pose for the optimization and the last pose for IMU conditioning
  min_active_pose_id_ = min_pose_id_ = local_map.ba_root_id;

  if (local_map.static_set_pose && !init_new_landmarks) {
    LoadPoseIntoBa(local_map, local_map.static_set_pose_id,
                   *local_map.static_set_pose, true, is_using_imu,
                   ba_cam_ids, false);
  }

  // Set attributes on the edges so they are coloured differently
  if (!init_new_landmarks) {
    map_->ClearEdgeAttributes(attribute);
    for (const auto& pair : local_map.inside_edge_ids) {
      map_->SetEdgeAttribute(pair.first, attribute);
    }
  }

  if (is_using_imu && !init_new_landmarks &&
      imu_ba_.GetNumProjResiduals() > 0) {
    for (auto& pair : local_map.inside_edge_ids) {
      const TransformEdgeId& edge_id = pair.first;
      pair.second.id = UINT_MAX;
      const SlamEdgePtr edge = map_->GetEdgePtr(edge_id);
      if (pose_ids_.count(edge_id.start) &&
          pose_ids_.count(edge_id.end) &&
          local_map.parent_chain_ids.count(edge_id.start) &&
          local_map.parent_chain_ids.count(edge_id.end)) {
        ROS_DEBUG_NAMED("optimization", "Attempting to add imu residual to edge id %s", boost::lexical_cast<std::string>(edge_id).c_str());
            

        if (!_AddImuResidualToEdge(edge, local_map, OptimizationConfig::getConfig()->getIMUWeight(),
                                   &pair.second.id)) {
          ROS_ERROR("Failed to add IMU residual to edge %s", boost::lexical_cast<std::string>(edge_id).c_str());
        }
      }
    }


    // If we are not marginalizing, then we must condition the IMU results on
    // the previous pose by adding a constraint.

    if (!init_new_landmarks &&
        imu_ba_.GetNumImuResiduals() > 0 && OptimizationConfig::getConfig()->doIMUConditioning()) {
      // Add imu prior edge
      const SlamFramePtr frame = map_->GetFramePtr(min_active_pose_id_);
      TransformEdgeId parent_edge = frame->parent_edge_id();

      ROS_DEBUG_NAMED("optimization", "Attempting to add prior edge with min_pose_id: %s and parent edge id: %s. Other end is %s", boost::lexical_cast<std::string>(min_active_pose_id_).c_str(), boost::lexical_cast<std::string>(parent_edge).c_str(), boost::lexical_cast<std::string>(parent_edge.OtherEnd(min_active_pose_id_)).c_str());

      if (parent_edge.valid()) {
        const SlamEdgePtr edge = map_->GetEdgePtr(parent_edge);
        const auto it_start = pose_ids_.find(parent_edge.start);
        const auto it_end = pose_ids_.find(parent_edge.end);

        if ((!edge->is_broken() || is_using_imu) &&
            it_start != pose_ids_.end() && it_end != pose_ids_.end() &&
            it_start->second < imu_ba_.GetNumPoses()) {

          const ba::PoseT<Scalar>& pose = imu_ba_.GetPose(it_start->second);
          if (pose.is_active && !pose.inertial_residuals.empty()) {
            ROS_ERROR("Pose in prior imu constraint cannot be active. min_active_pose_id_: %s and parent edge id: %s", boost::lexical_cast<std::string>(min_active_pose_id_).c_str(), boost::lexical_cast<std::string>(parent_edge).c_str());
            return false;
          }

          ROS_DEBUG_NAMED("optimization","Adding prior IMU constraint for edge %s with minPoseId %s", boost::lexical_cast<std::string>(parent_edge).c_str(), boost::lexical_cast<std::string>(min_active_pose_id_.id).c_str());

          unsigned int id = 0;
          if (!_AddImuResidualToEdge(edge, local_map,
                                     imu_ba_.IsTranslationEnabled() ?
                                     OptimizationConfig::getConfig()->getIMUPrior() : OptimizationConfig::getConfig()->getIMUPrior()/100.0, &id)) {
            ROS_ERROR("Failed to add prior IMU residual %s", boost::lexical_cast<std::string>(edge->id()).c_str());
            return false;
          }
          ROS_DEBUG_NAMED("optimization","Added prior with ID %d", id);
        }
      }
    }
  }

  if (min_pose_id_.valid() && !init_new_landmarks) {
    if (is_using_imu) {
      imu_ba_.SetRootPoseId(pose_ids_[min_pose_id_]);
    } else {
      ba_.SetRootPoseId(pose_ids_[min_pose_id_]);
    }

    // Before checking last_min_pose_id, make sure it's initialized.
    if (!last_min_pose_id_.valid()) {
      ROS_DEBUG_NAMED("optimization", "Initializing last_min_id to: %s", boost::lexical_cast<std::string>(min_pose_id_).c_str());
      last_min_pose_id_ = min_pose_id_;
    }

    ROS_DEBUG_NAMED("optimization", "init = %s, last_nin_id = %d, min_id = %d", (init_new_landmarks ? "True":"False"), last_min_pose_id_.id, min_pose_id_.id);
  }

  return true;
}

/**
 * Lift the map from the relative SlamMap into the local cartesian BA.
 *
 * Parameters passed from RefineMap
 *
 * @note imu_buffer can be nullptr to disable IMU use.
 */
template <typename BundleAdjuster>
inline void DoLiftMap(uint32_t depth,
                    const ReferenceFrameId& root_id,
                    const ba::Options<Scalar>& options,
                    bool do_landmark_init,
                    SlamMap* map,
                    BundleAdjuster* ba,
                    ba::InterpolationBufferT<ba::ImuMeasurementT<Scalar>,
                    Scalar>* imu_buffer,
                    LiftResults* results) {
  CameraRigPtr rig = map->GetCamera(root_id.session_id);
  if (!rig) return;
  
  ba->Init(options, depth * 3, depth * 50, depth * 50, rig->cameras[0].T_wc);
  ParentChainRefineMapVisitor<BundleAdjuster> parent_visitor(
      ba, imu_buffer, &results->pose_ba_ids, &results->edge_ba_ids,
      &results->landmark_ba_ids, &results->root_edge_id,
      &results->root_pose_id);
  parent_visitor.set_depth(depth);
  parent_visitor.set_root_id(root_id);
  parent_visitor.set_imu_weight(options.imu_weight);
  parent_visitor.set_imu_prior_weight(options.imu_prior);
  parent_visitor.set_disable_poses(do_landmark_init);
  parent_visitor.set_should_ignore_broken(imu_buffer != nullptr);
  map->ParentTraverse(&parent_visitor);

  GatherLandmarkFramesRefineMapVisitor<BundleAdjuster> lmk_visitor(
      do_landmark_init, map, ba, &results->pose_ba_ids,
      &results->landmark_ba_ids, options.min_lm_observations);
  lmk_visitor.set_depth(300);
  lmk_visitor.set_root_id(root_id);
  lmk_visitor.set_use_imu(imu_buffer != nullptr);
  lmk_visitor.set_should_ignore_broken(true);
  map->BFS(&lmk_visitor);
}

void Optimization::LiftMap(uint32_t depth,
                      const ReferenceFrameId& root_id,
                      bool do_landmark_init,
                      bool use_imu,
                      LiftResults* results) {
  ba::Options<Scalar> options;
  options.trust_region_size = OptimizationConfig::getConfig()->getTrustRegionInitSize();
  options.gyro_sigma = OptimizationConfig::getConfig()->getGyroSigma();
  options.gyro_bias_sigma = OptimizationConfig::getConfig()->doIMUConditioning();
  options.accel_sigma = OptimizationConfig::getConfig()->getAccelSigma();
  options.accel_bias_sigma = OptimizationConfig::getConfig()->getAccelBiasSigma();
  options.imu_weight = OptimizationConfig::getConfig()->getIMUWeight();
  options.imu_prior = OptimizationConfig::getConfig()->getIMUPrior();
  options.min_lm_observations = OptimizationConfig::getConfig()->getMinLMObservations();
  if (use_imu) {
    DoLiftMap<decltype(imu_ba_)>(depth, root_id, options, do_landmark_init,
                                 map_.get(), &imu_ba_, &imu_buffer_, results);
  } else {
    DoLiftMap<decltype(ba_)>(depth, root_id, options, do_landmark_init,
                             map_.get(), &ba_, nullptr, results);
  }
}

bool Optimization::PushMap(bool use_imu, const LiftResults& results,
                      const RefineMapCallbacks& callbacks) {
  bool success;
  if (use_imu) {
    success = ::rslam::optimization::PushMap(imu_ba_, results.edge_ba_ids, results.pose_ba_ids,
                             results.landmark_ba_ids, &imu_buffer_, map_.get());
    if (callbacks.visual_inertial_callback) {
      callbacks.visual_inertial_callback(imu_ba_, results);
    }

  } else {
    success = rslam::optimization::PushMap(ba_, results.edge_ba_ids, results.pose_ba_ids,
                             results.landmark_ba_ids, nullptr, map_.get());
    if (callbacks.visual_callback) {
      callbacks.visual_callback(ba_, results);
    }
  }
  return success;
}

template <typename BundleAdjuster>
OptimizationStatus ComputeAdaptiveMetrics(const BundleAdjuster& ba,
                                          Scalar prev_cond_error,
                                          uint32_t root_imu_id,
                                          Scalar* cond_error_out,
                                          double adaptive_threshold) {
  CHECK(cond_error_out);
  const ba::SolutionSummary<Scalar>& summary = ba.GetSolutionSummary();

  Scalar cond_inertial_error = -1.0;
  if (root_imu_id != UINT_MAX) {
    cond_inertial_error = ba.GetImuResidual(root_imu_id).mahalanobis_distance;
  }

  Scalar cond_v_chi2_dist =
      chi2inv(adaptive_threshold, summary.num_cond_proj_residuals * 2);
  Scalar cond_i_chi2_dist =
      chi2inv(adaptive_threshold, BundleAdjuster::kPoseDim);

  const Scalar cond_total_error = cond_inertial_error;
  const Scalar inertial_ratio = cond_inertial_error / cond_i_chi2_dist;
  const Scalar visual_ratio = summary.cond_proj_error / cond_v_chi2_dist;

  Scalar error_change = ((prev_cond_error - cond_total_error) /
                         prev_cond_error);

  *cond_error_out = prev_cond_error;
  if ((inertial_ratio > 1.0 || visual_ratio > 1.0) ||
      (cond_inertial_error >= 0.0 &&
       cond_total_error > prev_cond_error)) {
    return OptStatus_DiffOverThreshold;
  } else if (error_change < 0.0001) {
    return OptStatus_NoChange;
  } else{
    return OptStatus_DiffBelowThreshold;
  }
}

bool Optimization::RefineMap(unsigned int depth,
                        const ReferenceFrameId& root_id,
                        unsigned int max_iter,
                        bool use_imu,
                        calibu::CameraRigT<Scalar> &rig,
                        const AdaptiveOptions& adaptive_options,
                        bool do_landmark_init,
                        EdgeAttribute attribute,
                        const RefineMapCallbacks& callbacks) {
  CHECK(map_) << "Optimization::Init(map) must be called before RefineMap";
  if (!root_id.valid()) return false;

  if (adaptive_options.do_adaptive && adaptive_options.is_asynchronous &&
      map_->NumFrames() < adaptive_options.min_window_size) {
    return false;
  }

  bool is_using_imu = use_imu && map_->NumFrames() >= (unsigned int)OptimizationConfig::getConfig()->getMinFramesForIMU();

  if (do_landmark_init) {
    // First initialize any landmarks at infinity.
    ROS_DEBUG_NAMED("optimization","Initializing landmarks...");

    LiftResults results;
    LiftMap(depth, root_id, true, false, &results);
    if (results.pose_ba_ids.empty()) return false;

    // No dogleg for landmark initialization.
    ba_.options().use_dogleg = false;
    ba_.Solve(100, 1.0, OptimizationConfig::getConfig()->errorIncreaseAllowed());
    ba_.options().use_dogleg = OptimizationConfig::getConfig()->doDogleg();

    if (!PushMap(false, results, callbacks)) return false;
  }

  TransformEdgeId root_edge;
  bool more_adaptive = true;
  int num_iterations = 0;
  while (more_adaptive && num_iterations < 10) {
    ROS_DEBUG_NAMED("optimization","Solving ba...");

    LiftResults results;
    LiftMap(depth, root_id, false, is_using_imu, &results);
    if (results.pose_ba_ids.empty()) return false;

    map_->ClearEdgeAttributes(attribute);
    for (const auto& pair : results.edge_ba_ids) {
      map_->SetEdgeAttribute(pair.first, attribute);
    }

    if (is_using_imu) {
      imu_ba_.options().use_dogleg = OptimizationConfig::getConfig()->doDogleg();
      imu_ba_.Solve(max_iter, OptimizationConfig::getConfig()->getDampingFactor(),
                    OptimizationConfig::getConfig()->errorIncreaseAllowed());
    } else {
      ba_.options().use_dogleg = OptimizationConfig::getConfig()->doDogleg();
      ba_.Solve(max_iter, OptimizationConfig::getConfig()->getDampingFactor(), OptimizationConfig::getConfig()->errorIncreaseAllowed());
    }
    if (!PushMap(is_using_imu, results, callbacks)) return false;

    if (!adaptive_options.do_adaptive) break;

    if (!root_edge.valid()) {
      root_edge = results.root_edge_id;
    }

    // @todo Needs to work without IMU
    uint32_t root_edge_id = UINT_MAX;
    if (is_using_imu) {
      root_edge_id = results.edge_ba_ids[root_edge];
    }
    OptimizationStatus status;
    if (is_using_imu) {
      status = ComputeAdaptiveMetrics<decltype(imu_ba_)>(
          imu_ba_, prev_cond_error_, root_edge_id, &prev_cond_error_, OptimizationConfig::getConfig()->getAdaptiveThreshold());
    } else {
      status = ComputeAdaptiveMetrics<decltype(ba_)>(
          ba_, prev_cond_error_, root_edge_id, &prev_cond_error_, OptimizationConfig::getConfig()->getAdaptiveThreshold());
    }
    switch (status) {
      case OptStatus_DiffOverThreshold:
        ROS_DEBUG_NAMED("optimization", "Expanding window.");

        // Only increase the depth if we have enough poses in the map.
        depth += OptimizationConfig::getConfig()->getAdaptiveDepthIncrease();
        ROS_DEBUG_NAMED("optimization", "Incrementing depth to %d", depth);
        break;

      case OptStatus_DiffBelowThreshold:
      case OptStatus_NoChange:
        more_adaptive = false;
        break;
    }
    ++num_iterations;
  }
  return true;
}

///
/// \brief Responsible for frame-to-frame pose estimation
/// \param[in] work_set
/// \param[in] measurements
/// \param[in] camera rig
/// \param[in] reference to the map
/// \param[in/out] t_ab
/// \return
///
bool Optimization::GaussNewton(
    const LocalMap                              &work_set,
    const std::vector<MultiViewMeasurement>     &measurements,
    const SlamMap                               &map,
    Sophus::SE3t                                &t_ab,
    bool                                        has_imu,
    std::vector<ba::ImuMeasurementT<Scalar>>&   meas)
{
  ba::Options<Scalar> options;
  options.trust_region_size = OptimizationConfig::getConfig()->getTrustRegionInitSize();
  options.gyro_sigma = OptimizationConfig::getConfig()->getGyroSigma();
  options.gyro_bias_sigma = OptimizationConfig::getConfig()->doIMUConditioning();
  options.accel_sigma = OptimizationConfig::getConfig()->getAccelSigma();
  options.accel_bias_sigma = OptimizationConfig::getConfig()->getAccelBiasSigma();

  if (has_imu) {
    imu_ba_.Init(options, 1, measurements.size(), measurements.size());

    // now based on the global gravity, set the ba gravity direction
    imu_ba_.SetGravity(work_set.g_w);
  } else {
    ba_.Init(options, 1, measurements.size(), measurements.size());
  }

  // Add cameras to BA and save ids.
  const std::map<SessionId, CameraRigPtr>& rigs = work_set.cameras;
  std::map< SessionId, std::map<unsigned int, unsigned int> > ba_cam_ids;
  for( auto& rig_pair : rigs ) {
    const CameraRigPtr rig_ptr = rig_pair.second;
    for (unsigned int ii=0; ii<rig_ptr->cameras.size(); ++ii) {
      if (has_imu) {
        ba_cam_ids[rig_pair.first][ii] =
            imu_ba_.AddCamera(rig_ptr->cameras[ii].camera,
                              rig_ptr->cameras[ii].T_wc);
      } else {
        ba_cam_ids[rig_pair.first][ii] =
            ba_.AddCamera(rig_ptr->cameras[ii].camera,
                          rig_ptr->cameras[ii].T_wc);
      }

    }
  }

  uint32_t prev_pose_id;
  uint32_t pose_id;

  PoseContainerT reference_frame = work_set.poses.at(work_set.root_id);
  prev_pose_id = AddPoseIntoBa(has_imu, false, reference_frame);

  reference_frame.t_wp = t_ab;
  pose_id = AddPoseIntoBa(has_imu, true, reference_frame);

  // Add residual blocks.
  for (const MultiViewMeasurement& z: measurements) {
    const std::shared_ptr<LandmarkContainer> lmc =
        work_set.landmarks.at(z.id().landmark_id );
    Landmark lm;
    map.GetLandmark(z.id().landmark_id, &lm);
    const SessionId    lm_session_id = lm.id().ref_frame_id.session_id;
    const SessionId     z_session_id = z.id().frame_id.session_id;
    const unsigned int lm_ba_cam_id =
        ba_cam_ids[lm_session_id][lm.base_camera()];

    unsigned int lm_ba_id = UINT_MAX;
    CameraRigPtr rig_ptr = rigs.at(z_session_id);

    for (size_t cam_id=0; cam_id < z.NumCameras(); ++cam_id) {
      // Get BA camera id for landmark and for measurement
      unsigned int z_ba_cam_id = ba_cam_ids[z_session_id][cam_id];
      // Add landmark and projection residual if we have a good measurement.
      if (z.HasGoodMeasurementInCam(cam_id) &&
          lm.state() != eLmkAtInfinity) {
        if (lm_ba_id == UINT_MAX) {
          lm_ba_id = AddLandmarkIntoBa(has_imu, false, lm_ba_cam_id,
                                       prev_pose_id, *lmc);
        }
        AddProjectionResidualIntoBa(has_imu, z.Pixel(cam_id), pose_id,
                                    lm_ba_id, z_ba_cam_id);
      }
    }
  }

  if (has_imu) {
    imu_ba_.AddImuResidual(prev_pose_id, pose_id, meas);
  }

  if ((has_imu && pose_id > imu_ba_.GetNumPoses()) ||
      (!has_imu && pose_id > ba_.GetNumPoses())) {
    ROS_ERROR("pose_id %s is too large",
      boost::lexical_cast<std::string>(pose_id).c_str());
    return false;
  }

  Scalar proj_error, unary_error, binary_error, inertial_error;
  t_ab = GetPose(has_imu, pose_id).t_wp;
  if (has_imu) {
    imu_ba_.Solve(/*g_tracking_cvars.gn_max_num_iter*/10);
    imu_ba_.GetErrors(proj_error, unary_error, binary_error, inertial_error);
    // LOG(debug_level) << "GN proj error: " << proj_error << " gn imu error: " <<
    //             inertial_error;
  } else {
    ba_.Solve(/*g_tracking_cvars.gn_max_num_iter*/10);
    ba_.GetErrors(proj_error, unary_error, binary_error, inertial_error);
  }

  /*PrintMessage(g_tracking_cvars.estimate_debug_level,
    "  GaussNewton error: %f estimated pose %s",
    proj_error, Pose2Str(t_ab.matrix()));*/
  return true;

}

} // namespace optimization
} // namespace rslam
