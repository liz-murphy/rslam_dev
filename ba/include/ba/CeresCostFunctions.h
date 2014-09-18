/*
 This file is part of the BA Project.

 Copyright (C) 2013 George Washington University,
 Nima Keivan,
 Gabe Sibley

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#ifndef CERESCOSTFUNCTIONS_H
#define CERESCOSTFUNCTIONS_H

#include "Types.h"

namespace ba {

////////////////////////////////////////////////////////////////////////////////
template<typename T, typename Scalar>
static ImuPoseT<T> IntegratePoseJet(const ImuPoseT<T>& pose,
                                    const Eigen::Matrix<T, 9, 1>& k,
                                    const Scalar dt) {
  const Sophus::SO3Group<T> rv2_v1(
      Sophus::SO3Group < T > ::exp(k.template segment < 3 > (3) * (T) dt));

  ImuPoseT<T> y = pose;
  y.t_wp.translation() += k.template head<3>() * (T) dt;
  memcpy(
      y.t_wp.so3().data(),
      (rv2_v1.unit_quaternion() * pose.t_wp.so3().unit_quaternion()).coeffs()
          .data(),
      sizeof(T) * 4);

  // do euler integration for now
  y.v_w += k.template tail<3>() * (T) dt;
  return y;
}

////////////////////////////////////////////////////////////////////////////////
template<typename T, typename Scalar>
static Eigen::Matrix<T, 9, 1> GetPoseDerivativeJet(
    const ImuPoseT<T>& pose, const Eigen::Matrix<T, 3, 1>& t_w,
    const ImuMeasurementT<Scalar>& z_tart, const ImuMeasurementT<Scalar>& z_end,
    const Eigen::Matrix<T, 3, 1>& bg, const Eigen::Matrix<T, 3, 1>& ba,
    const Scalar dt) {
  double alpha = (z_end.time - (z_tart.time + dt)) / (z_end.time - z_tart.time);
  Eigen::Matrix<Scalar, 3, 1> zg = z_tart.w * alpha
      + z_end.w * (1.0 - alpha);
  Eigen::Matrix<Scalar, 3, 1> za = z_tart.a * alpha
      + z_end.a * (1.0 - alpha);

  Eigen::Matrix<T, 9, 1> deriv;
  //derivative of position is velocity
  // v (velocity)
  deriv.template head<3>() = pose.v_w;
  // w (angular rates)
  deriv.template segment < 3 > (3) = pose.t_wp.so3().Adj()
      * (zg.template cast<T>() + bg);
  // a (acceleration)
  deriv.template segment < 3 > (6) = pose.t_wp.so3()
      * (za.template cast<T>() + ba) - t_w;

  return deriv;
}

////////////////////////////////////////////////////////////////////////////////
template<typename T, typename Scalar>
static ImuPoseT<T> IntegrateImuJet(const ImuPoseT<T>& pose,
                                   const ImuMeasurementT<Scalar>& z_start,
                                   const ImuMeasurementT<Scalar>& z_end,
                                   const Eigen::Matrix<T, 3, 1>& bg,
                                   const Eigen::Matrix<T, 3, 1>& ba,
                                   const Eigen::Matrix<T, 3, 1>& g,
                                   Eigen::Matrix<Scalar, 10, 6>* dy_db = 0,
                                   Eigen::Matrix<Scalar, 10, 10>* dy_dy0 = 0) {
  //construct the state matrix
  Scalar dt = z_end.time - z_start.time;
  if (dt == 0) {
    return pose;
  }
  ImuPoseT<T> res = pose;
  Eigen::Matrix<T, 9, 1> k;

  const Eigen::Matrix<T, 9, 1> k1 = GetPoseDerivativeJet<T, Scalar>(pose, g,
                                                                    z_start,
                                                                    z_end, bg,
                                                                    ba, 0);
  const ImuPoseT<T> y1 = IntegratePoseJet<T, Scalar>(pose, k1, dt * 0.5);
  const Eigen::Matrix<T, 9, 1> k2 = GetPoseDerivativeJet<T, Scalar>(y1, g,
                                                                    z_start,
                                                                    z_end, bg,
                                                                    ba, dt / 2);
  const ImuPoseT<T> y2 = IntegratePoseJet<T, Scalar>(pose, k2, dt * 0.5);
  const Eigen::Matrix<T, 9, 1> k3 = GetPoseDerivativeJet<T, Scalar>(y2, g,
                                                                    z_start,
                                                                    z_end, bg,
                                                                    ba, dt / 2);
  const ImuPoseT<T> y3 = IntegratePoseJet<T, Scalar>(pose, k3, dt);
  const Eigen::Matrix<T, 9, 1> k4 = GetPoseDerivativeJet<T, Scalar>(y3, g,
                                                                    z_start,
                                                                    z_end, bg,
                                                                    ba, dt);
  k = (k1 + (T) 2 * k2 + (T) 2 * k3 + k4);
  res = IntegratePoseJet<T, Scalar>(pose, k, dt / 6.0);

  res.w_w = k.template segment < 3 > (3);
  res.time = z_end.time;
//        pose.m_dW = currentPose.m_dW;
  return res;
}

////////////////////////////////////////////////////////////////////////////////
template<typename T, typename Scalar>
static ImuPoseT<T> IntegrateResidualJet(
    const PoseT<T>& pose,
    const std::vector<ImuMeasurementT<Scalar>>& measurements,
    const Eigen::Matrix<T, 3, 1>& bg, const Eigen::Matrix<T, 3, 1>& ba,
    const Eigen::Matrix<T, 3, 1>& g, std::vector<ImuPoseT<T>>& poses) {
  ImuPoseT<T> imu_pose(pose.t_wp, pose.v_w, Eigen::Matrix<T, 3, 1>::Zero(),
                       pose.time);

  const ImuMeasurementT<Scalar>* prev_meas = 0;
  poses.clear();
  poses.reserve(measurements.size() + 1);
  poses.push_back(imu_pose);

  // integrate forward in time, and retain all the poses
  for (const ImuMeasurementT<Scalar>& meas : measurements) {
    if (prev_meas != 0) {
      imu_pose = IntegrateImuJet<T, Scalar>(imu_pose, *prev_meas, meas, bg, ba,
                                            g);
      poses.push_back(imu_pose);
    }
    prev_meas = &meas;
  }
  return imu_pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Ceres autodifferentiatable cost function for pose errors.
/// The parameters are error state and should be a 6d pose delta
template<typename Scalar = double>
struct GlobalPoseCostFunction {
  GlobalPoseCostFunction(const Sophus::SE3Group<Scalar>& measurement,
                         const double weight = 1.0)
      : t_wc(measurement),
        weight(weight) {
  }

  template<typename T>
  bool operator()(const T* const t_t_ic, const T* const t_t_wi,
                  T* residuals) const {
    const Eigen::Map<const Sophus::SE3Group<T> > t_ic(t_t_ic);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wi(t_t_wi);
    //the pose residuals
    Eigen::Map < Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);

    pose_residuals = (t_wi * t_ic * t_wc.inverse().template cast<T>()).log()
        * (T) weight;

    pose_residuals.tail(3) * (T) weight;
    pose_residuals.head(3) * (T) weight;
    return true;
  }

  const Sophus::SE3Group<Scalar> t_wc;
  const double weight;
};

////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct FullImuCostFunction {
  FullImuCostFunction(const std::vector<ImuMeasurementT<Scalar>>& meas,
                      const double w)
      : measurements(meas),
        weight(w) {
  }

  template<typename T>
  bool operator()(const T* const _tx2, const T* const _tx1,
                  const T* const _tvx2, const T* const _tvx1,
                  const T* const _tg, const T* const _tbg, const T* const _tba,
                  T* residuals) const {
    //the residual vector consists of a 6d pose and a 3d velocity residual
    //the pose residuals
    Eigen::Map < Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);
    //the velocity residuals
    Eigen::Map < Eigen::Matrix<T, 3, 1> > vel_residuals(&residuals[6]);

    // parameter vector consists of a
    // 6d pose delta plus starting
    // velocity and 2d gravity angles
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx2(_tx2);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx1(_tx1);
    const Eigen::Map<const Sophus::SO3Group<T> > R_wx1(&_tx1[0]);
    //the velocity at the starting point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v1(_tvx1);
    //the velocity at the end point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v2(_tvx2);
    //gyro bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > bg(_tbg);
    //accelerometer bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > ba(_tba);
    //the 2d gravity vector (angles)
    const Eigen::Map<const Eigen::Matrix<T, 2, 1> > g(_tg);

    //get the gravity components in 3d based on
    // the 2 angles of the gravity vector
    const Eigen::Matrix<T, 3, 1> g_vector = GetGravityVector<T>(g);

    PoseT<T> start_pose;
    start_pose.t_wp = t_wx1;
    start_pose.v_w = v1;
    start_pose.time = measurements.front().time;
    std::vector<ImuPoseT<T>> vPoses;
    ImuPoseT<T> end_pose = IntegrateResidualJet<T, Scalar>(start_pose,
                                                           measurements, bg, ba,
                                                           g_vector, vPoses);

    //and now calculate the error with this pose
    pose_residuals = (end_pose.t_wp * t_wx2.inverse()).log() * (T) weight;

    //to calculate the velocity error, first augment the IMU integration
    // velocity with gravity and initial velocity
    vel_residuals = (end_pose.v_w - v2) * (T) weight * (T) 0.1;
    return true;
  }

  const std::vector<ImuMeasurementT<Scalar>> measurements;
  const double weight;
};

////////////////////////////////////////////////////////////////////////////////
template<typename ProjModel>
struct ImuReprojectionCostFunctor {
  ImuReprojectionCostFunctor(Eigen::Vector3d pw, Eigen::Vector2d pc)
      : p_w(pw),
        p_c(pc) {
    t_vr = (calibu::RdfVision * calibu::RdfRobotics.inverse());
  }

  template<typename T>
  bool operator()(const T* const _t_wk, const T* const _r_ck,
                  const T* const _t_ck, const T* const cam_params,
                  T* residuals) const {
    Eigen::Map < Eigen::Matrix<T, 2, 1> > r(residuals);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wk(_t_wk);
    const Sophus::SE3Group<T> t_kw = t_wk.inverse();
    const Eigen::Map<const Sophus::SO3Group<T> > R_ck(_r_ck);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_ck(_t_ck);
    const Sophus::SE3Group<T> t_ck(R_ck, p_ck);

    const Eigen::Matrix<T, 3, 1> p_cv = (t_ck * (t_kw * p_w.cast<T>()));
    const Eigen::Matrix<T, 2, 1> z = ProjModel::template Project<T>(p_cv,
                                                                    cam_params);
    r = z - p_c.cast<T>();

    return true;
  }

  Eigen::Vector3d p_w;
  Eigen::Vector2d p_c;
  Sophus::SO3d t_vr;
};

////////////////////////////////////////////////////////////////////////////////
template<typename Scalar = double>
struct SwitchedFullImuCostFunction {
  SwitchedFullImuCostFunction(const std::vector<ImuMeasurementT<Scalar>>& meas,
                              const double weight, const bool* res_switch)
      : measurements(meas),
        weight(weight),
        residal_switch(res_switch) {
  }

  template<typename T>
  bool operator()(const T* const _tx2, const T* const _tx1,
                  const T* const _tvx2, const T* const _tvx1,
                  const T* const _tg, const T* const _tbg, const T* const _tba,
                  T* residuals) const {
    //the residual vector consists of a 6d pose and a 3d velocity residual
    //the pose residuals
    Eigen::Map < Eigen::Matrix<T, 6, 1> > pose_residuals(residuals);
    //the velocity residuals
    Eigen::Map < Eigen::Matrix<T, 3, 1> > vel_residuals(&residuals[6]);

    //parameter vector consists of a 6d pose delta plus starting velocity
    // and 2d gravity angles
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx2(_tx2);
    const Eigen::Map<const Sophus::SE3Group<T> > t_wx1(_tx1);
    const Eigen::Map<const Sophus::SO3Group<T> > r_wx1(&_tx1[0]);
    //the velocity at the starting point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v1(_tvx1);
    //the velocity at the end point
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > v2(_tvx2);
    //gyro bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > bg(_tbg);
    //accelerometer bias
    const Eigen::Map<const Eigen::Matrix<T, 3, 1> > ba(_tba);
    //the 2d gravity vector (angles)
    const Eigen::Map<const Eigen::Matrix<T, 2, 1> > g(_tg);

    //get the gravity components in 3d based on the 2
    // angles of the gravity vector
    const Eigen::Matrix<T, 3, 1> g_vector = GetGravityVector<T>(g);

    PoseT<T> start_pose;
    start_pose.t_wp = t_wx1;
    start_pose.v_w = v1;
    start_pose.time = measurements.front().time;
    std::vector<ImuPoseT<T>> poses;
    ImuPoseT<T> end_pose = IntegrateResidualJet<T, Scalar>(start_pose,
                                                           measurements, bg, ba,
                                                           g_vector, poses);

    //and now calculate the error with this pose
    pose_residuals = (end_pose.t_wp * t_wx2.inverse()).log() * (T) weight;

    // to calculate the velocity error, first augment the IMU integration
    // velocity with gravity and initial velocity
    vel_residuals = (end_pose.v_w - v2) * (T) weight * (T) 0.5;

    if (*residal_switch == true) {
      pose_residuals.template head<3>().setZero();
      vel_residuals.setZero();
    }
    return true;
  }

  const std::vector<ImuMeasurementT<Scalar>> measurements;
  const double weight;
  const bool* residal_switch;
};

}

#endif
