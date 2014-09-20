#include <place_matching/MultiDBoWMatcher/Persistence.h>

void Persistence::saveBowVector(const DBoW2::BowVector& bowvec,
                                cv::FileStorage& fs)
{
  fs << "[:";
  for(DBoW2::BowVector::const_iterator bit = bowvec.begin();
      bit != bowvec.end(); ++bit)
  {
    fs << static_cast<int>(bit->first) << bit->second; // wordid, word value
  }
  fs << "]";
}

void Persistence::loadBowVector(const cv::FileNode& fn,
                                DBoW2::BowVector& bowvec)
{
  bowvec.clear();
  for(size_t i = 1; i < fn.size(); i += 2)
  {
    DBoW2::WordId wid = static_cast<int>(fn[i-1]);
    DBoW2::WordValue v = static_cast<double>(fn[i]);
    bowvec.insert(std::make_pair(wid, v));
  }
}

void Persistence::saveReferenceFrameId(const ReferenceFrameId& frameid,
                                       cv::FileStorage& fs)
{
  fs << "[:" << static_cast<int>(frameid.id);
  Persistence::saveUuid(frameid.session_id.uuid, fs);
  fs << "]";
}

void Persistence::loadReferenceFrameId(const cv::FileNode& fn,
                                       ReferenceFrameId& frameid)
{
  frameid.id = static_cast<uint32_t>(static_cast<int>(fn[0]));
  Persistence::loadUuid(fn[1], frameid.session_id.uuid);
}

void Persistence::saveLandmarkId(const LandmarkId& landmark,
                                 cv::FileStorage& fs)
{
  fs << "[" <<
        static_cast<int>(landmark.landmark_index) <<
        static_cast<int>(landmark.track2d_id);
  Persistence::saveReferenceFrameId(landmark.ref_frame_id, fs);
  fs << "]";
}

void Persistence::loadLandmarkId(const cv::FileNode& fn, LandmarkId& landmark)
{
  landmark.landmark_index = static_cast<int>(fn[0]);
  landmark.track2d_id = static_cast<int>(fn[1]);
  Persistence::loadReferenceFrameId(fn[2], landmark.ref_frame_id);
}

void Persistence::saveUuid(const rslam::uuid::uuid_t& uuid,
                           cv::FileStorage& fs)
{
  std::stringstream ss;
  ss << uuid;
  fs << ss.str();
}

void Persistence::loadUuid(const cv::FileNode& fn, rslam::uuid::uuid_t& uuid)
{
  std::string str;
  fn >> str;
  if (rslam::uuid::uuid_parse(&str[0], uuid)) {
    std::cerr << "UUID PARSE ERROR " << std::endl;
  }
}
