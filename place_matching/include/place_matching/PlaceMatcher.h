
// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef PLACE_MATCHER_H
#define PLACE_MATCHER_H

#include <unistd.h>
#include <limits>
#include <string>
#include <vector>
#include <mutex>
#include <place_matching/PlaceMatchCandidate.h>
#include <slam_map/SlamMap.h>

namespace cv {
  class Mat;
  class KeyPoint;
}  // namespace cv

class PlaceMatcher {
 public:

  /**
   * @brief Normal constructor
   */
  PlaceMatcher(){}

  /**
   * @brief Constructor for methods that use a slam map
   */
  PlaceMatcher(std::shared_ptr<SlamMap> map): m_map(map){}

  virtual ~PlaceMatcher(){}

  virtual size_t NumPlaces() const = 0;

  virtual void Save(const std::string& filename) const {}
  virtual void Load(const std::string& filename) {}

  /// New interface for all the matchers.
  /// By default, these functions call the old interface function, keeping
  /// track of the ReferenceFrameId's and their corresponding unsigned int ids

  /**
   * @brief Sets the map that can be used by some matchers
   * @param map
   */
  virtual void SetMap(std::shared_ptr<SlamMap> map) {
    m_map = map;
  }

  /**
   * @brief Extracts features from an image
   * @param id frame id
   * @param image image
   * @param keys keypoints
   * @param descs descriptors
   */
  virtual void getFeatures(const ReferenceFrameId& id,
                   const cv::Mat& image,
                   std::vector<cv::KeyPoint> &keys,
                   std::vector<cv::Mat>& descs) const {};

  /**
   * @brief Adds a place to the matcher
   * @param id frame id
   * @param keys place feature keypoints
   * @param descs place feature descriptors
   */
  virtual void AddPlace(const ReferenceFrameId& id, const cv::Mat& place);

  /**
   * @brief Detecst matches with the given image.
   * @param id frame id
   * @param place place image
   * @param vPlaceMatches matches, if any
   */
  virtual void GetPotentialPlaceMatches(const ReferenceFrameId& id,
    const cv::Mat& place, std::vector<PlaceMatchCandidate>& vPlaceMatches);

  /**
   * @brief GetPotentialMatchesOrAddPlace This runs loop detection and, if
   * no loop is found, the given place is added to the database
   * @param place place image
   * @param vPlaceMatches
   * @param uFrameId current place id
   * @param vSubset if given, ids of places to match
   */
  virtual void GetPotentialPlaceMatchesOrAddPlace(const ReferenceFrameId& id,
    const cv::Mat& im, std::vector<PlaceMatchCandidate>& vPlaceMatches);

  /**
   * @brief Returns the unique place id (unsigned int) given to the places
   * added. This method is useful for the matchers that implements the old
   * interface only, to directly call some of their functions. New algorithms
   * should not need this, since they can use the ReferenceFrameId in all the
   * functions
   * @param id frame id
   * @return arbitrary internal id
   */
  unsigned int GetExistingInternalPlaceId(const ReferenceFrameId& id) const;

protected:

  /// Interface for old matchers that do not use a SLAM map

  virtual void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      const std::vector<unsigned int> &vSubset,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) {}

  /** Attempt to match against entire history of places */
  virtual void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) {}

  virtual void GetPotentialPlaceMatchesOrAddPlace(
      const cv::Mat &place,
      std::vector<PlaceMatchCandidate>& vPlaceMatches,
      const unsigned int uFrameId = std::numeric_limits<unsigned int>::max(),
      const std::vector<unsigned int> &vSubset = std::vector<unsigned int>());

  virtual void AddTracks(
      unsigned int uFrameId,
      std::vector<std::pair<unsigned int, unsigned int> > &trackIds,
      std::vector<unsigned char *> &features, int desc_size) {}

  virtual void AddPlace(const unsigned int uFrameId,
                        const cv::Mat &newPlace) {}

  /**
   * @brief Saves the information w.r.t. frame indices. This function should
   * be called by the ::Save function of the old matchers
   * @param os stream. A new line is written, followed by a line with the
   * frame indices and a new line
   */
  void saveFrameIndices(std::ofstream& os) const;

  /**
   * @brief Loads the frame indices from a stream. This function should be
   * called by the ::Load function of the old matchers
   * @param is stream. It is assumed that the line next to current one is the
   * one that contains the frame indices
   */
  void loadFrameIndices(std::ifstream& is);

  /**
   * @brief Retrieves keypoints from the tracks of a frame (requires a map)
   * @param id frame id
   * @param keys keypoints
   * @param lms if given, the landmarks of the keypoints are stored here
   */
  virtual void getFrameKeypoints(const ReferenceFrameId& id,
                                 std::vector<cv::KeyPoint> &keys,
                                 std::vector<LandmarkId>* lms = NULL) const;

  /**
   * @brief Stores a frame id and returns its internal index.
   * @param id frame id
   * @return arbitrary internal id
   */
  unsigned int getNewInternalPlaceId(const ReferenceFrameId& id);

private:

  /**
   * @brief Fills the PlaceMatchCandidate::m_frameId member for those matchers
   * that do not fill it by themselves
   * @param matches
   */
  void fillFrameId(std::vector<PlaceMatchCandidate>& matches) const;

protected:

  // For matchers that use a map
  std::shared_ptr<SlamMap> m_map;

  // For matchers that only implement the old interface
  std::map<ReferenceFrameId, int> m_frames; // m_frames[uint] = frameid
  std::map<int, ReferenceFrameId> m_place_ids; // m_place_ids[frameid] = uint
  mutable std::mutex m_mutex_uint;

};

#endif
