#include <opencv2/opencv.hpp>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <place_matching/MultiDBoWMatcher/Persistence.h>

void MultiDBoWMatcher::Node::save(cv::FileStorage& fs) const
{
  // Format:
  // [ frameid keys descs [landmarks] ]

  fs << "[";
  Persistence::saveReferenceFrameId(frameid, fs);
  fs << keys << descs;
  fs << "[";
  for(auto& landmark : landmarks)
    Persistence::saveLandmarkId(landmark, fs);
  fs << "]";
  fs << "]";
}

void MultiDBoWMatcher::Node::load(const cv::FileNode& fn)
{
  Persistence::loadReferenceFrameId(fn[0], frameid);
  cv::read(fn[1], keys);
  cv::read(fn[2], descs);

  landmarks.resize(fn[3].size());
  for(size_t i = 0; i < fn[3].size(); ++i)
    Persistence::loadLandmarkId(fn[3][i], landmarks[i]);
}
