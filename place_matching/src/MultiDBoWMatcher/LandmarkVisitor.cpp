#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>

MultiDBoWMatcher::LandmarkVisitor::LandmarkVisitor(
    const std::vector<Landmark>& landmarks,
    const ReferenceFrameId& frame,
    const Sophus::SE3t& T_wc)
    : m_landmarks(landmarks), m_T_wc(T_wc)
{
  set_depth(100);
  set_has_visit(true);
  set_root_id(frame);

  m_points.resize(landmarks.size());

  size_t idx = 0;
  for(auto lit = landmarks.begin(); lit != landmarks.end(); ++lit, ++idx)
    m_open[lit->id().ref_frame_id].push_back(idx);
}

std::vector<cv::Point3f>& MultiDBoWMatcher::LandmarkVisitor::getPoints()
{
  return m_points;
}

bool MultiDBoWMatcher::LandmarkVisitor::IsDone()
{
  return m_open.empty();
}

std::vector<unsigned int>
MultiDBoWMatcher::LandmarkVisitor::getLostLandmarks() {
  std::vector<unsigned int> lms;
  for (const auto& open_frame : m_open) {
    if (open_frame.second.empty()) continue;

    lms.insert(lms.end(), open_frame.second.begin(), open_frame.second.end());
  }

  return lms;
}

bool MultiDBoWMatcher::LandmarkVisitor::Visit(const SlamFramePtr& cur_node)
{
  TransformMapVisitor::Visit(cur_node);

  auto it = m_open.find(cur_node->id());
  if(it != m_open.end())
  {
    Sophus::SE3t Tc = CurT();
    const std::vector<size_t>& i_points = it->second;

    for(size_t i : i_points) {
      const Eigen::Vector4t& xrp4 = m_landmarks[i].xrp();
      Eigen::Vector3t xrp(xrp4[0] / xrp4[3],
                          xrp4[1] / xrp4[3],
                          xrp4[2] / xrp4[3]);
      Eigen::Vector3t x_robot_lm = m_T_wc.inverse() * Tc * xrp;

      // Convert to cv axes
      m_points[i] = cv::Point3f(x_robot_lm[0], x_robot_lm[1], x_robot_lm[2]);
    }
    m_open.erase(it);
  }

  return !m_open.empty();
}
