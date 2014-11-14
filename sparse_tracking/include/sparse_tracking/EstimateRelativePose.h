// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <common_front_end/SystemStatus.h>
#include <common_front_end/Ransac.h>
#include <sparse_tracking/MatchInTime.h>

#include <local_map/LocalMap.h>
#include <slam_map/SlamMapFwd.h>

#include <utils/MathTypes.h>
#include <utils/PoseHelpers.h>
#include <utils/Timer.h>

#include <optimization/optimization.h>

namespace rslam {
namespace sparse {


///////////////////////////////////////////////////////////////////////////////
const char* Pose2Str( const Eigen::Matrix4t& T );

///
/// \brief Compute median reprojection error
/// \param[in] work_set lifted local map
/// \param[in] rig cameras' intrinsics and extrinsics.
/// \param[in] t_ws world to sensor transform
/// \param[in] measurements
/// \return median reprojection error
///
Scalar CalcSigma(
    const LocalMap                            &work_set,
    const Sophus::SE3t                        &t_ab,
    const std::vector<MultiViewMeasurement> &measurements);

///
/// \brief will try to find matches for previously missed landmarks
/// and outliers
/// \param[in] work_set lifted map to work on
/// \param[in] t_ab estimated relative edge
/// \param[in,out] measurements the newly obtained measurements
/// \return
///
int DoReThreading(
    const LocalMap                             &work_set,
    const Sophus::SE3t                         &t_ab,
    std::vector<MultiViewMeasurement>          &measurements,
    const FeatureImageVector                   &images
                  );

///
/// \brief Loop through measuremnts, reproject and flag outliers.
/// \param[in] outlier_threshold
/// \param[in] outlier_flag assigned to the measurement if it's an outlier
/// \param[in] work_set lifted local map
/// \param[in] t_ab Relative transform estimate.
/// \param[in,out] measurements MultiViewMeasurements to flag
/// \param[in,out] msre inlier reprojection error.
/// \param[out] num_landmarks_at_infinity optional output of how many
///             landmarks at infinity were tracked
/// \return number of inliers
///
int FlagOutliers(
    const float                          outlier_threshold,
    const MatchFlag                      outlier_flag,
    const LocalMap                       &work_set,
    const Sophus::SE3t                   &t_ab,
    const SlamMap                        &map,
    std::vector<MultiViewMeasurement>    &measurements,
    double                               &msre,
    int* num_landmarks_at_infinity);

///
/// \brief Predict pixel measurement locations and also tracking homography
/// shape. This info is saved in the local map, and use in MatchInTime.
/// \param t_ab motion hint for tracking homographies (a is the root)
/// \param rig
/// \param local_map used to predict tracking homographies
/// \param map
///
void LiftTrackingData(const CameraRigPtr& rig,
                      const Sophus::SE3t &t_ab,
                      const SlamMap &map,
                      LocalMap &local_map);


///
/// \brief _EstimateRelativePose
/// \param[in] working_set
/// \param[in] input images
/// \param[in] new_measurements
/// \param[in] feature_matches
/// \param[in] no_tic
/// \param[in,out] t_ab is the estimated relative transform.
/// \param[out] num_inliers
/// \return
///
bool EstimateRelativePose(const ReferenceFrameId &frame_id,
    LocalMap                              &working_set,
    optimization::Optimization            &optimization,
    const FeatureImageVector              &images,
    std::vector<MultiViewMeasurement>  &new_measurements,
    std::vector< std::vector<Feature*> >  &feature_matches,
    const bool                            no_tic,
    Sophus::SE3t                          &t_ab,
    double                                learning_rate,
    common::SystemStatus&                 system_status,
    const std::shared_ptr<Timer>&         timer,
    std::vector<ba::ImuMeasurementT<Scalar> > &meas,
    bool                                  has_imu = false);

}
}
