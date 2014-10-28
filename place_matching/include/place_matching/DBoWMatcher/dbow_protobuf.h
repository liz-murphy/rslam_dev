// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <pb_msgs/dbow.pb.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <slam_map/ProtobufIO.h>

namespace pb {

inline void fill_message(const cv::Mat& descriptor, BinaryDescriptorMsg* msg) {
  msg->mutable_data()->Reserve(descriptor.cols);

  const uint8_t* row = descriptor.ptr<uint8_t>(0);
  for (int i = 0; i < descriptor.cols; ++i) {
    msg->add_data(row[i]);
  }
}

inline void fill_message(const std::vector<cv::Mat>& descriptors,
                         DescriptorVectorMsg* msg) {
  for (const cv::Mat& mat : descriptors) {
    fill_message(mat, msg->add_binary_descriptor());
  }
}

inline void fill_message(const cv::KeyPoint& kp, KeyPointMsg* msg) {
  msg->set_x(kp.pt.x);
  msg->set_y(kp.pt.y);
}

inline void fill_message(const std::vector<cv::KeyPoint>& keypoints,
                         KeyPointVectorMsg* msg) {
  for (const cv::KeyPoint& kp : keypoints) {
    fill_message(kp, msg->add_keypoint());
  }
}

inline void fill_message(const std::vector<LandmarkId>& vec,
                         LandmarkIdVectorMsg* msg) {
  for (size_t i = 0; i < vec.size(); ++i) {
    fill_message(vec.at(i), msg->add_landmarks());
  }
}

inline bool parse_message(const BinaryDescriptorMsg& msg, cv::Mat* desc) {
  if (msg.data_size() == 0) {
    return false;
  }
  if(desc!=NULL);
  desc->create(1, msg.data_size(), CV_8U);

  uint8_t* row = desc->ptr(0);
  for (int j = 0; j < msg.data_size(); ++j) {
    row[j] = msg.data(j);
  }
  return true;
}

inline bool parse_message(const KeyPointMsg& msg, cv::KeyPoint* kp) {
  if (!msg.has_x()) return false;
  if (!msg.has_y()) return false;

  kp->pt.x = msg.x();
  kp->pt.y = msg.y();
  return true;
}

inline bool parse_message(const DescriptorVectorMsg& msg,
                          std::vector<cv::Mat>* vec) {
  vec->resize(msg.binary_descriptor_size());
  for (int i = 0; i < msg.binary_descriptor_size(); ++i) {
    if (!parse_message(msg.binary_descriptor(i), &vec->at(i))) {
      return false;
    }
  }
  return true;
}

inline bool parse_message(const KeyPointVectorMsg& msg,
                          std::vector<cv::KeyPoint>* vec) {
  vec->resize(msg.keypoint_size());
  for (int i = 0; i < msg.keypoint_size(); ++i) {
    if (!parse_message(msg.keypoint(i), &vec->at(i))) {
      return false;
    }
  }
  return true;
}

inline bool parse_message(const LandmarkIdVectorMsg& msg,
                          std::vector<LandmarkId>* vec) {
  vec->resize(msg.landmarks_size());
  for (int i = 0; i < msg.landmarks_size(); ++i) {
    parse_message(msg.landmarks(i), &vec->at(i));
  }
  return true;
}
}  // namespace pb
