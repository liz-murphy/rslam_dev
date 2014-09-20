#include <slam_map/ReferenceFrame.h>
#include <slam_map/TransformEdge.h>
#include <slam_map/SlamMapFwd.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>


MultiDBoWMatcher::WindowVisitor::WindowVisitor
  (AdjacencyWindow &win, const ReferenceFrameId& init_node, double max_dist,
   unsigned int max_depth)
  : m_win(win), m_max_dist(max_dist)
{
  set_root_id(init_node);
  set_depth(max_depth);
  set_has_visit(true);
  set_has_explore_node(true);
}

bool MultiDBoWMatcher::WindowVisitor::Visit(const SlamFramePtr& cur_node)
{
  m_visited_nodes.push_back(cur_node->id());
  return TransformMapVisitor::Visit(cur_node);
}

bool MultiDBoWMatcher::WindowVisitor::ExploreNode(const SlamFramePtr& parent,
                                                  const SlamEdgePtr& edge,
                                                  const SlamFramePtr& child)
{
  const ReferenceFrameId& cid = child->id();

  bool accept_child = true;
  double dist = CurT().translation().norm();

  if(dist <= m_max_dist)
  {
    // check if the child node was already in the window
    auto wit = m_win.lower_bound(cid);

    if(wit == m_win.end() || wit->first != cid)
      m_win.emplace_hint(wit, cid, dist); // add the child to the window
    else if(dist < wit->second)
      wit->second = dist; // update child with a new distance
    else
      accept_child = false; // the child was already expanded
  }
  else
    accept_child = false;

  if(accept_child) TransformMapVisitor::ExploreNode(parent, edge, child);

  return accept_child;
}

std::vector<ReferenceFrameId>& MultiDBoWMatcher::WindowVisitor::visitedNodes()
{
  return m_visited_nodes;
}
