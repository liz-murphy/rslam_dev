// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <rslam.pb.h>
#include <algorithm>
#include <Eigen/Core>
#include <CommonFrontEnd/PatchHomography.h>
#include <PbMsgs/Matrix.h>
#include <PbMsgs/Pose.h>
#include <calibu/cam/CameraRig.h>
#include <miniglog/logging.h>
#include <slam_map/LandmarkId.h>
#include <slam_map/MeasurementId.h>
#include <slam_map/Measurement.h>
#include <slam_map/Landmark.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>

namespace pb {

template<int M, int N, typename Scalar>
void fill_message(const Eigen::Matrix<Scalar, M, N>& mat,
                  DoubleMatrixMsg* msg);

template<unsigned int PatchSize>
void fill_message(const PatchHomography<PatchSize>& ph,
                  PatchHomographyMsg* msg);

void fill_message(const SessionId& id, SessionIdMsg* msg);

void fill_message(const ReferenceFrameId& id, ReferenceFrameIdMsg* msg);

void fill_message(const TransformEdgeId& id, TransformEdgeIdMsg* msg);

void fill_message(const LandmarkId& id, LandmarkIdMsg* msg);

void fill_message(const MeasurementId& id, MeasurementIdMsg* msg);

void fill_message(const Landmark& landmark, LandmarkMsg* msg);

void fill_message(const MultiViewMeasurement& z, MultiViewMeasurementMsg* msg);

void fill_message(const ReferenceFrame& frame, ReferenceFrameMsg* msg);

void fill_message(const TransformEdge& edge, TransformEdgeMsg* msg);

void fill_message(
    const calibu::CameraModelAndTransformT<Scalar>& model_transform,
    CameraModelMsg* msg);

void fill_message(const calibu::CameraRigT<Scalar>& rig, CameraRigMsg* msg);

void parse_message(const SessionIdMsg& msg, SessionId* id);

void parse_message(const ReferenceFrameIdMsg& msg, ReferenceFrameId* id);

void parse_message(const TransformEdgeIdMsg& msg, TransformEdgeId* id);

void parse_message(const LandmarkIdMsg& msg, LandmarkId* id);

void parse_message(const MeasurementIdMsg& msg, MeasurementId* id);

template<int M, int N, typename Scalar>
void parse_message(const DoubleMatrixMsg& msg,
                   Eigen::Matrix<Scalar, M, N>* mat);

template<unsigned int PatchSize>
void parse_message(const PatchHomographyMsg& msg,
                   PatchHomography<PatchSize>* ph);

void parse_message(const MultiViewMeasurementMsg& msg, MultiViewMeasurement* z);

void parse_message(const LandmarkMsg& msg, Landmark* landmark);

void parse_message(const ReferenceFrameMsg& msg, ReferenceFrame* frame);

void parse_message(const TransformEdgeMsg& msg, TransformEdge* edge);

void parse_message(const CameraModelMsg& msg,
                   calibu::CameraModelAndTransformT<Scalar>* model);

void parse_message(const CameraRigMsg& msg, calibu::CameraRigT<Scalar>* rig);

}  // end namespace pb

template<int M, int N, typename Scalar>
void pb::fill_message(const Eigen::Matrix<Scalar, M, N>& mat,
                      DoubleMatrixMsg* msg) {
  msg->set_rows(M);
  msg->set_cols(N);
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      msg->add_data(mat(i, j));
    }
  }
}

template<unsigned int PatchSize>
void pb::fill_message(const PatchHomography<PatchSize>& ph,
                      PatchHomographyMsg* msg) {
  msg->set_state(ph.GetState());
  msg->set_scale(ph.scale());
  pb::fill_message(ph.matrix(), msg->mutable_h());
}

template<int M, int N, typename Scalar>
void pb::parse_message(const DoubleMatrixMsg& msg,
                       Eigen::Matrix<Scalar, M, N>* mat) {
  if (msg.rows() != M || msg.cols() != N) {
    LOG(FATAL) << "Attempting to load mismatched matrices! "
               << "Message has size " << msg.rows() << "x" << msg.cols()
               << " and matrix is of size " << M << "x" << N << std::endl;
  }

  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < N; ++j) {
      mat->operator()(i, j) = msg.data(i*N + j);
    }
  }
}

template<unsigned int PatchSize>
void pb::parse_message(const PatchHomographyMsg& msg,
                       PatchHomography<PatchSize>* ph) {
  ph->SetState((typename PatchHomography<PatchSize>::HomographyState)
               msg.state());
  ph->SetScale(msg.scale());

  Eigen::Matrix3t h;
  pb::parse_message(msg.h(), &h);
  ph->SetMatrix(h);
}
