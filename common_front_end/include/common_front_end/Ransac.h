// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <cfloat>
#include <random>
#include <calibu/cam/CameraRig.h>
#include <common_front_end/CommonFrontEndConfig.h>
#include <utils/MathTypes.h>
#include <utils/PrintMessage.h>
#include <utils/Utils.h>

///
/// \brief GetUniqueIndices
/// \param[in] selected_cam_id
/// \param[in] points_2d
/// \param[in] camera_ids
/// \param[out] subset_indices
/// \return
///
inline bool GetUniqueIndices(
    const unsigned int                 selected_cam_id,
    const std::vector<Eigen::Vector2t> &points_2d,
    const std::vector<unsigned int>    &cam_ids,
    std::mt19937*                      rng,
    unsigned int                       (&subset_indices)[3])
{
  CHECK_NOTNULL(rng);
  // select 3 good points that are well spread out
  unsigned int& n1 = subset_indices[0];
  unsigned int& n2 = subset_indices[1];
  unsigned int& n3 = subset_indices[2];

  std::uniform_int_distribution<unsigned int> meas_dist(0,
                                                        points_2d.size() - 1);
  do{
    n1 = meas_dist(*rng);
  } while (cam_ids[n1] != selected_cam_id);
  do{
    n2 = meas_dist(*rng);
  } while (n1 == n2 || cam_ids[n2] != selected_cam_id);
  do{
    n3 = meas_dist(*rng);
  } while (n3 == n1 || n3 == n2 || cam_ids[n3] != selected_cam_id);

  return true;
}

///
/// \brief HaveLargeDepth. Checks if inverse depth is close to zero.
/// \param[in] x1
/// \param[in] x2
/// \param[in] x3
/// \return
///
inline bool HaveLargeDepth( Eigen::Vector4t x1,
                            Eigen::Vector4t x2,
                            Eigen::Vector4t x3)
{
  Scalar eps = 1e-3;

  if (x1(3) < eps) return true;
  if (x2(3) < eps) return true;
  if (x3(3) < eps) return true;

  return false;
}

///
/// \brief AreColinear. Checks if 2d points a,b,c are colinear
/// \param[in] a
/// \param[in] b
/// \param[in] c
/// \return true if colinear
///
inline bool AreColinear(
    const Eigen::Vector2t& a,
    const Eigen::Vector2t& b,
    const Eigen::Vector2t& c
    )
{
  // make sure the points are spread out
  const Scalar thresh = 100;
  Scalar dab_squared = (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]);
  if( dab_squared < thresh ){
    return true;
  }

  Scalar dbc_squared = (b[0]-c[0])*(b[0]-c[0]) + (b[1]-c[1])*(b[1]-c[1]);
  if( dbc_squared < thresh ){
    return true;
  }

  Scalar dca_squared = (c[0]-a[0])*(c[0]-a[0]) + (c[1]-a[1])*(c[1]-a[1]);
  if( dca_squared < thresh ){
    return true;
  }

  // want unit dot product near 1
  Scalar amag = std::sqrt( a[0]*a[0] + a[1]*a[1] );
  Scalar bmag = std::sqrt( b[0]*b[0] + b[1]*b[1] );
  Scalar cmag = std::sqrt( c[0]*c[0] + c[1]*c[1] );
  Scalar a0 = a[0] / amag;
  Scalar a1 = a[1] / amag;
  Scalar b0 = b[0] / bmag;
  Scalar b1 = b[1] / bmag;
  Scalar c0 = c[0] / cmag;
  Scalar c1 = c[1] / cmag;

  // a-b dot a-c
  Scalar dot = (a0-b0)*(a0-c0) + (a1-b1)*(a1-c1);
  if( acos(dot) < M_PI/4 ) {
    return true;
  }
  return false; // good not colinear
}

///
/// \brief GoodRandIndices (guided sampling)
/// \param[in] work_set
/// \param[in] cam_id
/// \param[in] measurements
/// \param[out] subset_indices
/// \return
///
inline bool GoodRandIndices(
    const std::vector<Eigen::Vector4t> &points_3d,
    const std::vector<Eigen::Vector2t> &points_2d,
    const std::vector<unsigned int>    &cam_ids,
    unsigned int                       selected_cam_id,
    unsigned int                       (&subset_indices)[3],
    std::mt19937*                      rng
)
{
  /// quickly make sure there are even enough good measurements
  int num_good_indices = 0;
  Scalar eps = 1e-8;
  for (size_t ii = 0; num_good_indices < 3 && ii < points_2d.size(); ii++) {
    //const bool good_measurement =
    //    measurements[ii].HasGoodMeasurementInCam(selected_cam_id);
    if( cam_ids[ii] != selected_cam_id ) {
      continue;
    }

    // make sure the point is not at infinity in case
    // we are using inverse depth. For non inverse depth the fourth
    // coordinate should always be 1.
    if (fabs(points_3d[ii][3]) > eps) {
       num_good_indices++;
    }
  }

  if (num_good_indices < 3) {
    PrintMessage( CommonFrontEndConfig::getConfig()->ransac_debug_level,
                  "    WARNING: only %d good measurements in cam[%d] "
                  "-- not enough for RANSAC\n",
                  num_good_indices, selected_cam_id );
    return false;
  }

  int count = 0;
  int max_trials = 100; // CVar??
  bool points_are_colinear;
  bool points_have_large_depth;
  do{
    if (!GetUniqueIndices(selected_cam_id,
                          points_2d,
                          cam_ids,
                          rng,
                          subset_indices)) {
      return false;
    }
    // quickly check the indicies are good ( check they are not colinear )
    points_are_colinear = AreColinear(points_2d[subset_indices[0]],
                                      points_2d[subset_indices[1]],
                                      points_2d[subset_indices[2]]);

    points_have_large_depth = HaveLargeDepth(points_3d[subset_indices[0]],
                                             points_3d[subset_indices[1]],
                                             points_3d[subset_indices[2]]);

  } while (points_are_colinear && count++ < max_trials);

  if (points_are_colinear || points_have_large_depth){
    PrintMessage( CommonFrontEndConfig::getConfig()->ransac_debug_level,
                  "      RANSAC Failed to select a measurement subset.\n");
    return false;
  }
  return true;
}

///
/// \brief for dealing with outliers in 3D-2D correspondences
/// \param[in] points_3d
/// \param[in] points_2d
/// \param[in] cam_ids,
/// \param[in] camera rig
/// \param[out] estimated frame-to-frame transform
/// \return success or failure
///
inline bool Ransac(const std::vector<Eigen::Vector4t> &points_3d,
                   const std::vector<Eigen::Vector2t> &points_2d,
                   const std::vector<unsigned int>    &cam_ids,
                   const calibu::CameraRigT<Scalar>   &rig,
                   std::mt19937*                      rng,
                   Sophus::SE3t                       &t_ab)
{

  unsigned int sub_set_indices[3];
  // World to sensor transform
  std::vector<Sophus::SE3t> t_sw(rig.cameras.size());
  // Camera pose solutions for subset
  std::vector<Sophus::SE3t > poses;

  const unsigned int max_trials   = CommonFrontEndConfig::getConfig()->ransac_max_trials;
  const unsigned int max_data_trials = 100; // CVar
  const double  prob              = CommonFrontEndConfig::getConfig()->ransac_probability ;
  const double  outlier_threshold = CommonFrontEndConfig::getConfig()->ransac_outlier_threshold;
  const double  eps               = 1.0e-10;
  unsigned int  trial_count       = 0;
  unsigned int  num_trials        = 1;
  double        frac_inliers      = 0.0;
  double        prob_no_outliers  = 0.0;
  bool          success           = false;
  Sophus::SE3t  best_twv;

  const size_t num_meas = points_2d.size();
  unsigned int best_num_inliers = 0;
  double    best_total_error = DBL_MAX;

  PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,"<RANSAC>\n");
  if (num_meas < 3) {
    PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,
                 "  RANSAC: not enough measurements\n");
  }

  unsigned int num_tries = 0;
  unsigned int min_trials = 10;
  std::uniform_int_distribution<unsigned int> cam_dist(
      0, rig.cameras.size() - 1);
  while (((num_trials > trial_count) ||
         (trial_count <= min_trials)) &&
         (num_meas >= 3)) {
    // Select random camera
    unsigned int selected_cam = cam_dist(*rng);

    if (!GoodRandIndices(points_3d,
                         points_2d,
                         cam_ids,
                         selected_cam,
                         sub_set_indices,
                         rng)) {
      PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,
                   "  Unable to select a nondegenerate data set.\n");
      if (num_tries++ > max_data_trials) {
        PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,
                     "  Unable to select a nondegenerate data "
                     "set after %d tries, bailing\n", num_tries);
        return false;
      }
      continue;
    }

    const Eigen::Vector2t& z1 = points_2d[sub_set_indices[0]];
    const Eigen::Vector2t& z2 = points_2d[sub_set_indices[1]];
    const Eigen::Vector2t& z3 = points_2d[sub_set_indices[2]];

    PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,
                 "  Selected MultiViewMeasurements: [%d %d %d] from cam[%d]:"
                 " [%.3f %.3f],  [%.3f %.3f],  [%.3f %.3f]\n",
                 sub_set_indices[0],sub_set_indices[1],sub_set_indices[2],
        selected_cam, z1[0], z1[1], z2[0], z2[1], z3[0], z3[1]);

    // Generate a model from sub-set
    // Get 3D landmarks w.r.t. vehicle
    Eigen::Vector4t x1 = points_3d[sub_set_indices[0]];
    Eigen::Vector4t x2 = points_3d[sub_set_indices[1]];
    Eigen::Vector4t x3 = points_3d[sub_set_indices[2]];

    // and divide by last component. This will give us (x,y,z) if we were using
    // inverse depth parameterization.
    x1.head<3>() /= x1(3);
    x2.head<3>() /= x2(3);
    x3.head<3>() /= x3(3);

    // Solve localization problem
    SolveThreePointPose(z1, z2, z3,
                        x1.head<3>(),
                        x2.head<3>(),
                        x3.head<3>(),
                        rig.cameras[selected_cam].camera,
                        poses);

    // convert poses back to robot coordinate frame
    for (size_t pp=0; pp < poses.size(); ++pp) {
      // convert back to vehicle frame
      poses[pp] = poses[pp] * rig.cameras[selected_cam].T_wc.inverse();
    }

    // Find and count inliers for current model
    unsigned int  best_pose             = 0;
    unsigned int  best_pose_num_inliers = 0;
    double        best_pose_total_error = DBL_MAX;

    for (size_t pp=0; pp < poses.size(); ++pp) {
      Sophus::SE3t t_wv = poses[pp];

      for (size_t cam_id = 0; cam_id < rig.cameras.size(); ++cam_id) {
        t_sw[cam_id] = (t_wv * rig.cameras[cam_id].T_wc).inverse();
      }

      unsigned int pose_num_inliers = 0;
      double total_error   = 0.0;
      // score the whole set
      for( size_t ii=0; ii < points_2d.size(); ++ii) {
        //compute reprojection error in both cameras
        //-- hmm could easily use just one
        const Eigen::Vector2t &z   = points_2d[ii];
        const Eigen::Vector4t &x_w = points_3d[ii];
        const Eigen::Vector4t x_s =
            Sophus::MultHomogeneous(t_sw[cam_ids[ii]],x_w);
        const Eigen::Vector2t p =
            rig.cameras[cam_ids[ii]].camera.Project(x_s.head(3));
        const Eigen::Vector2t e = p - z;
        const double error = e.norm();

        //if "dist" to model is > than the inlier threshold ->FLAG as outlier
        if (error < outlier_threshold) {
          pose_num_inliers++;
          total_error += error;
        } else {
          total_error += outlier_threshold;
        }
      }

      if ((pose_num_inliers > best_pose_num_inliers ) ||
          (pose_num_inliers == best_pose_num_inliers &&
           total_error < best_pose_total_error)) {
        best_pose      = pp;
        best_pose_num_inliers = pose_num_inliers;
        best_pose_total_error = total_error;
      }
    }

    // Save bestinliers & update N
    if (( best_pose_num_inliers > best_num_inliers) ||
        ( best_pose_num_inliers == best_num_inliers &&
          best_pose_total_error < best_total_error)) {
      // Largest set of inliers so far (with minimum error)...
      best_num_inliers = best_pose_num_inliers;
      best_total_error = best_pose_total_error;
      best_twv = poses[best_pose];
      success = true;
    }

    //Update estimate of N, the number of trials to ensure we pick,
    // with probability p, a data set with no outliers.
    frac_inliers = ((double)best_num_inliers) / num_meas;
    prob_no_outliers = 1 - (frac_inliers*frac_inliers*frac_inliers);
    if (prob_no_outliers < eps) {
      prob_no_outliers = eps; // Avoid division by -Inf
    }
    if (prob_no_outliers > (1-eps)) {
      prob_no_outliers = (1-eps); // Avoid division by 0
    }

    num_trials = log( 1-prob ) / log( prob_no_outliers );
    num_tries = 0; // reset the GoodRandIndices counter...
    trial_count++;

    // Safeguard vs infinite loop
    if (trial_count > max_trials) {
      PrintMessage(CommonFrontEndConfig::getConfig()->ransac_debug_level,
                   "  RANSAC reached the maximum number of trials\n");
      break;
    }
  }  //End Model-est cycle---------------

  //check if a solution was found
  if (!success) {
    LOG(CommonFrontEndConfig::getConfig()->ransac_debug_level) << "RANSAC did not find a model";
  } else {
    t_ab = best_twv;
  }

  //PrintMessage(g_frontend_cvars.estimate_debug_level,
  //             "  RANSAC found %d inliers with pose est %s\n",
  //             best_num_inliers, Pose2Str(t_ab.matrix()) );
  LOG(CommonFrontEndConfig::getConfig()->ransac_debug_level) << "<RANSAC>";
  return true;
}
