#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>

bool MultiDBoWMatcher::WindowTrack::push(const AdjacencyWindow& window,
                                         double max_matching_distance)
{
  bool ret = false;

  if(m_last_window.isClose(window, max_matching_distance))
  {
    ++m_count;
    ret = true;
  }
  else
    m_count = 1;

  m_last_window = window;

  return ret;
}

void MultiDBoWMatcher::WindowTrack::save(cv::FileStorage &fs) const
{
  // We dont actually store the track, so that it will be loaded as empty
  // Format:
  // [ [] 0 ]

  // this format allows to load data in the future if we decide to add this
  fs << "[" <<
        "[" << "]" << 0 <<
        "]";
}

void MultiDBoWMatcher::WindowTrack::load(const cv::FileNode &fn)
{
}
