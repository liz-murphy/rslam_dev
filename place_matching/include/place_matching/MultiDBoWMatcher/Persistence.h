#pragma once

#include <opencv2/opencv.hpp>
#include <slam_map/ReferenceFrameId.h>
#include <slam_map/LandmarkId.h>
#include <place_matching/DBoWMatcher/DBoW2/DBoW2.h>
#include <slam_map/uuid.h>

class Persistence
{
public:

  /**
   * @brief Saves a bowvector in a file storage
   * @param bowvec
   * @param fs
   */
  static void saveBowVector(const DBoW2::BowVector& bowvec,
                            cv::FileStorage& fs);

  /**
   * @brief Loads a bowvector from a file node
   * @param fn
   * @param bowvec
   */
  static void loadBowVector(const cv::FileNode& fn, DBoW2::BowVector& bowvec);

  /**
   * @brief Saves a frameid in a file storage
   * @param frameid
   * @param fs
   */
  static void saveReferenceFrameId(const ReferenceFrameId& frameid,
                                   cv::FileStorage& fs);

  /**
   * @brief Loads a reference frame id from a file node
   * @param fn
   * @param frameid
   */
  static void loadReferenceFrameId(const cv::FileNode& fn,
                                   ReferenceFrameId& frameid);

  /**
   * @brief Saves a landmark id in a file storage
   * @param frameid
   * @param fs
   */
  static void saveLandmarkId(const LandmarkId& landmark, cv::FileStorage& fs);

  /**
   * @brief Loads a landmark id from a file node
   * @param fn
   * @param frameid
   */
  static void loadLandmarkId(const cv::FileNode& fn, LandmarkId& landmark);

  /**
   * @brief Saves a uuid in a file storage
   * @param uuid
   * @param fs
   */
  static void saveUuid(const rslam::uuid::uuid_t& uuid, cv::FileStorage& fs);

  /**
   * @brief Loads a uuid from a file node
   * @param fn
   * @param uuid
   */
  static void loadUuid(const cv::FileNode& fn, rslam::uuid::uuid_t& uuid);
};
