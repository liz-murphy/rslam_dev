#include <mutex>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <place_matching/MultiDBoWMatcher/Persistence.h>

void MultiDBoWMatcher::Session::setLastBowVector(DBoW2::BowVector& bow)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_last_bowvec = std::move(bow);
}

void MultiDBoWMatcher::Session::resetTrack()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  m_track.reset();
}

bool MultiDBoWMatcher::Session::pushTrack(const AdjacencyWindow& window,
                                          double distance)
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_track.push(window, distance);
}

void MultiDBoWMatcher::Session::save(cv::FileStorage& fs) const
{
  // Format:
  // [ id bowvec track ]
  std::lock_guard<std::mutex> lock(m_mutex);

  fs << "[";
  Persistence::saveUuid(id.uuid, fs);
  Persistence::saveBowVector(m_last_bowvec, fs);
  m_track.save(fs);
  fs << "]";
}

void MultiDBoWMatcher::Session::load(const cv::FileNode& fn)
{
  std::lock_guard<std::mutex> lock(m_mutex);

  Persistence::loadUuid(fn[0], id.uuid);
  Persistence::loadBowVector(fn[1], m_last_bowvec);
  m_track.load(fn[2]);
}
