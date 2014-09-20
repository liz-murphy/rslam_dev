#include <slam_map/ReferenceFrameId.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>

size_t
MultiDBoWMatcher::AdjacencyWindowManager::updateWindow(
    DBoW2::EntryId eid, const ReferenceFrameId& frameid, double score)
{
  std::map<ReferenceFrameId, std::set<size_t> >::iterator wit =
      m_node_to_windows.lower_bound(frameid);

  // select window of this entry id
  size_t widx;
  if(wit == m_node_to_windows.end() || wit->first != frameid)
  {
    // new window
    wit = m_node_to_windows.emplace_hint(wit, frameid, std::set<size_t>());
    widx = m_adj_windows.size();
    wit->second.insert(widx);
    m_adj_windows.emplace_back(MultiDBoWMatcher::AdjacencyWindow());
  }
  else
  {
    // at least one existing window contains this entry
    widx = *wit->second.begin(); // wit->second is set<size_t>

    // merge others if necessary
    std::vector<size_t> wids; // copy indices
    wids.insert(wids.end(), wit->second.begin(), wit->second.end());

    for(std::vector<size_t>::const_iterator widit = wids.begin() + 1;
        widit != wids.end(); ++widit)
    {
      mergeWindows(widx, *widit);

      // the merged windows are not needed anylonger, but we cannot remove
      // them from m_adj_windows because we need to keep the indices of
      // m_node_to_windows consistent. We can release the memory, though.
      m_adj_windows[*widit].clear();
    }
  }

  // add item to the window (or update) with distance 0
  m_adj_windows[widx][frameid] = 0.;

  // accumulate score
  m_adj_windows[widx].score += score;

  // add generator
  m_adj_windows[widx].generators.push_back(std::make_pair(eid, score));

  return widx;
}

void MultiDBoWMatcher::AdjacencyWindowManager::mergeWindows
  (const size_t i_target, const size_t i_merged)
{
  if(i_target == i_merged) return;

  MultiDBoWMatcher::AdjacencyWindow& target = m_adj_windows[i_target];
  MultiDBoWMatcher::AdjacencyWindow& merged = m_adj_windows[i_merged];

  // merge score and generators
  target.score += merged.score;
  target.generators.insert(target.generators.end(),
                           merged.generators.begin(),
                           merged.generators.end());

  std::list<std::pair<ReferenceFrameId, double> > new_nodes;
  MultiDBoWMatcher::AdjacencyWindow::iterator tit, mit;
  tit = target.begin();
  mit = merged.begin();

  while(tit != target.end() && mit != merged.end())
  {
    if(tit->first == mit->first)
    {
      // item both in target and merged. Keep the minimum distance
      if(mit->second < tit->second) tit->second = mit->second;

      ++tit;
      ++mit;
    }
    else if(mit->first < tit->first)
    {
      // an entry in merged that is not in target, flag for addition
      new_nodes.emplace_back(mit->first, mit->second);
      ++mit;
    }
    else // an entry in target that is not in merged, nothing to do
      ++tit;
  }
  // make sure all the elements of merged are included into target
  for(; mit != merged.end(); ++mit)
    new_nodes.emplace_back(mit->first, mit->second);

  // add the elements now
  for(auto nit = new_nodes.begin(); nit != new_nodes.end(); ++nit)
  {
    // add to target
    target.emplace(nit->first, nit->second);

    // delete merged window from the node and add the target one
    std::map<ReferenceFrameId, std::set<size_t> >::iterator mit =
        m_node_to_windows.find(nit->first);

    if(mit != m_node_to_windows.end())
    {
      mit->second.erase(i_merged);
      mit->second.insert(i_target);
    }
    // else: shouldnt happen
  }
}

void MultiDBoWMatcher::AdjacencyWindowManager::expand(
    size_t widx, const std::shared_ptr<SlamMap>& map,
    const ReferenceFrameId& node, double max_distance)
{
  AdjacencyWindow& window = m_adj_windows[widx];
  std::vector<ReferenceFrameId> added;

  window.expand(map, node, max_distance, &added);

  for(const auto& id : added) m_node_to_windows[id].insert(widx);
}

void MultiDBoWMatcher::AdjacencyWindowManager::getWindows
  (std::vector<MultiDBoWMatcher::AdjacencyWindow>& windows) const
{
  windows.clear();
  windows.reserve(m_adj_windows.size());

  std::vector<unsigned int> i_order;
  DUtils::STL::indexSort(m_adj_windows.begin(), m_adj_windows.end(), i_order,
                         MultiDBoWMatcher::AdjacencyWindow::gtScores);

  for(unsigned int idx : i_order)
  {
    if(!m_adj_windows[idx].empty())
      windows.push_back(m_adj_windows[idx]);
  }
}

void MultiDBoWMatcher::AdjacencyWindowManager::pruneWindows()
{
  for(auto& win : m_adj_windows) win.prune();
}
