#include <mutex>
#include <opencv2/opencv.hpp>
#include <slam_map/ReferenceFrame.h>
#include <place_matching/PlaceMatcher.h>

void PlaceMatcher::GetPotentialPlaceMatchesOrAddPlace(
    const cv::Mat &place,
    std::vector<PlaceMatchCandidate>& vPlaceMatches,
    const unsigned int uFrameId,
    const std::vector<unsigned int> &vSubset)
{
  if (vSubset.empty()) {
    GetPotentialPlaceMatches(place, vPlaceMatches);
  } else {
    GetPotentialPlaceMatches(place, vSubset, vPlaceMatches);
  }

  if(vPlaceMatches.empty()) AddPlace(uFrameId, place);
}

void PlaceMatcher::getFrameKeypoints(const ReferenceFrameId& id,
                                     std::vector<cv::KeyPoint> &keys,
                                     std::vector<LandmarkId>* lms) const
{
  keys.clear();
  if(lms) lms->clear();
  if(!m_map) return;

  const SlamFramePtr& frame = m_map->GetFramePtr(id);
  if(!frame) return;

  const size_t N = frame->NumMeasurements();
  keys.reserve(N);
  for(size_t i = 0; i < N; ++i)
  {
    MultiViewMeasurement m;
    if(frame->GetMeasurement(i, &m))
    {
      const int cam = 0;
      if(m.HasGoodMeasurementInCam(cam))
      {
        const Eigen::Matrix<Scalar, 2, 1>& px = m.Pixel(cam);

        // notes about cv keypoint:
        // size is the diameter of the patch
        // orientation is [0,360) or -1 if not applicable, in the image coords
        // cv::KeyPoint k(x, y, size, angle, response, octave, classid);
        keys.emplace_back(px.data()[0], px.data()[1],
            31.*m.Scale(cam), m.Orientation(cam), 1., cvRound(m.Scale(cam)));
        if(lms) lms->emplace_back(m.id().landmark_id);
      }
    }
  }
}

void PlaceMatcher::AddPlace(const ReferenceFrameId& id, const cv::Mat& place)
{
  unsigned int uint;
  {
    std::lock_guard<std::mutex> lock(m_mutex_uint);
    uint = m_frames.size() + 1;
    m_frames.emplace(id, uint);
    m_place_ids.emplace(uint, id);
  }
  AddPlace(uint, place);
}

void PlaceMatcher::GetPotentialPlaceMatches(const ReferenceFrameId& id,
  const cv::Mat& place, std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
  GetPotentialPlaceMatches(place, vPlaceMatches);
  fillFrameId(vPlaceMatches);
}

void PlaceMatcher::GetPotentialPlaceMatchesOrAddPlace(
    const ReferenceFrameId& id, const cv::Mat& im,
    std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
  unsigned int uint;
  {
    std::lock_guard<std::mutex> lock(m_mutex_uint);
    uint = m_frames.size() + 1;
    m_frames.emplace(id, uint);
    m_place_ids.emplace(uint, id);
  }
  GetPotentialPlaceMatchesOrAddPlace(im, vPlaceMatches, uint);
  fillFrameId(vPlaceMatches);
}

unsigned int PlaceMatcher::GetExistingInternalPlaceId
  (const ReferenceFrameId& id) const
{
  std::lock_guard<std::mutex> lock(m_mutex_uint);
  auto fit = m_frames.find(id);
  return fit->second;
}

unsigned int PlaceMatcher::getNewInternalPlaceId(const ReferenceFrameId& id)
{
  std::lock_guard<std::mutex> lock(m_mutex_uint);
  unsigned int place_id = m_frames.size() + 1;
  m_frames.emplace(id,  place_id);
  m_place_ids.emplace(place_id, id);
  return place_id;
}

void PlaceMatcher::fillFrameId(std::vector<PlaceMatchCandidate>& matches) const
{
  std::lock_guard<std::mutex> lock(m_mutex_uint);
  for(auto& m : matches)
  {
    int id = m.getID();
    if(0 <= id && static_cast<size_t>(id) < m_frames.size())
      m.setFrameId(m_place_ids.at(id));
    else // this should not happen
      m.setFrameId(ReferenceFrameId());
  }
}

void PlaceMatcher::saveFrameIndices(std::ostream& os) const
{
  os << std::endl;
  {
    std::lock_guard<std::mutex> lock(m_mutex_uint);
    for(const auto& frame : m_frames) {
      os << frame.first.id << " "
         << frame.first.session_id;
    }
  }
  os << std::endl;
}

void PlaceMatcher::loadFrameIndices(std::istream& is)
{
  std::string line;
  std::getline(is, line); // skip current line
  std::getline(is, line); // get next line
  std::stringstream ss(line);

  std::map<ReferenceFrameId, int> frames;
  std::map<int, ReferenceFrameId> place_ids;
  ReferenceFrameId frame;

  unsigned int place_id = 0;
  while(ss >> frame.id) {
    std::string uuid;
    ss >> uuid;
    if (rslam::uuid::uuid_parse(&uuid[0], frame.session_id.uuid)) {
      frames.emplace(frame, ++place_id);
      place_ids.emplace(place_id, frame);
    }
  }

  std::lock_guard<std::mutex> lock(m_mutex_uint);
  m_frames.swap(frames);
  m_place_ids.swap(place_ids);
}
