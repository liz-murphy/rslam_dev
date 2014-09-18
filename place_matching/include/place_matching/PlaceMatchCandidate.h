// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <opencv2/opencv.hpp>
#include <utils/MathTypes.h>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/LandmarkId.h>

class PlaceMatchCandidate
{
public:
  // Additional information for MultiDBoW

  std::vector<cv::KeyPoint> query; // inlier points from query frame
  std::vector<cv::KeyPoint> match; // inlier points from match frame
  std::vector<LandmarkId> query_landmarks; // inlier landmarks from query frame
  std::vector<LandmarkId> match_landmarks; // inlier landmarks from match frame
  Sophus::SE3t Tqm; // transformation from query to match node (vehicle axes)

public:
  PlaceMatchCandidate() :
      m_nFrame1(0),
      m_dScore(0) {}

  PlaceMatchCandidate(const unsigned int nFrame1, const float dScore) :
      m_nFrame1(nFrame1),
      m_dScore(dScore) {}

  PlaceMatchCandidate(const ReferenceFrameId& frameId, const float dScore) :
      m_nFrame1(frameId.id),
      m_frameId(frameId),
      m_dScore(dScore) {}

  unsigned int getID() {
    return m_nFrame1;
  }

  inline const ReferenceFrameId& getFrameId() const {
    return m_frameId;
  }

  float getScore() {
    return m_dScore;
  }

  inline void setID(int id) {
    m_nFrame1 = id;
  }

  inline void setFrameId(const ReferenceFrameId& frameId) {
    m_frameId = frameId;
  }

  inline void setScore(float score) {
    m_dScore = score;
  }

  friend bool operator< (const PlaceMatchCandidate &p1,
                         const PlaceMatchCandidate &p2) {
    return p1.m_dScore < p2.m_dScore;
  }

 private:
  int m_nFrame1; // old data
  ReferenceFrameId m_frameId;
  float m_dScore;
};
