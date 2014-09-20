#include <slam_map/SlamMap.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>

void MultiDBoWMatcher::AdjacencyWindow::prune()
{
  for(auto nit = this->begin(); nit != this->end(); )
  {
    if(nit->second != 0)
      this->erase(nit++); // erase and get the next iterator before invalidation
    else
      ++nit;
  }
}

void MultiDBoWMatcher::AdjacencyWindow::expand
  (const std::shared_ptr<SlamMap>& map, const ReferenceFrameId& init_node,
   double max_distance,
   std::vector<ReferenceFrameId>* added)
{
  if(added) added->clear();
  if(max_distance <= 0) return;

  WindowVisitor visitor(*this, init_node, max_distance);
  map->BFS(&visitor);

  if(added) *added = std::move(visitor.visitedNodes());
}



bool MultiDBoWMatcher::AdjacencyWindow::isClose
  (const AdjacencyWindow& b, double max_distance) const
{
  const AdjacencyWindow& A = *this;
  const AdjacencyWindow& B = b;

  if(A.empty() || B.empty()) return false;

  if((A.rbegin()->first < B.begin()->first) ||
     (B.rbegin()->first < A.begin()->first))
    return false;

  AdjacencyWindow::const_iterator ait, bit;
  ait = A.begin();
  bit = B.begin();

  while(ait != A.end() && bit != B.end())
  {
    if(ait->first == bit->first)
    {
      // not necessary since the windows are expanded beforehand
      //int d = ait->second + bit->second;
      //if(d <= max_distance) return true;
      return true;

      ++ait;
      ++bit;
    }
    else if(ait->first < bit->first)
      ++ait;
    else
      ++bit;
  }

  return false;
}

void MultiDBoWMatcher::AdjacencyWindow::getBestNode(DBoW2::EntryId &eid,
                                                    double& score) const
{
  if(generators.empty())
  {
    eid = 0;
    score = 0;
    return;
  }

  auto it =
    std::max_element(generators.begin(), generators.end(), cmpGenerators());

  eid = it->first;
  score = it->second;
}

bool MultiDBoWMatcher::AdjacencyWindow::gtScores
  (const AdjacencyWindow& a, const AdjacencyWindow& b)
{
  return a.score > b.score;
}

bool MultiDBoWMatcher::AdjacencyWindow::cmpGenerators::operator ()
  (const std::pair<DBoW2::EntryId, double>& a,
   const std::pair<DBoW2::EntryId, double>& b)
{
  return a.second < b.second;
}
