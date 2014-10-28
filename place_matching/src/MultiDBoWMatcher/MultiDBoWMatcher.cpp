#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <slam_map/SlamMap.h>
#include <slam_map/ReferenceFrame.h>
#include <slam_map/ReferenceFrameId.h>
#include <place_matching/DBoWMatcher/DUtils/DUtils.h>
#include <place_matching/DBoWMatcher/DBoW2/DBoW2.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <ros/ros.h>
static int kDebugLevel = 2;

MultiDBoWMatcher::MultiDBoWMatcher(std::shared_ptr<SlamMap> map):
  PlaceMatcher(map)
{
}

MultiDBoWMatcher::MultiDBoWMatcher(std::shared_ptr<SlamMap> map,
                                   const std::string &voc_file,
                                   const Parameters &params,
                                   const std::string &descriptor):
  PlaceMatcher(map)
{
  setVocabulary(voc_file, params);
  if(!descriptor.empty()) setDescriptorType(descriptor);
}

void MultiDBoWMatcher::setVocabulary(const std::string &voc_file,
                                     const Parameters &params)
{
  {
    std::lock_guard<std::mutex> lock(m_mutex_params);
    m_params = params;
  }

  if(voc_file.empty())
    throw std::invalid_argument("No vocabulary given");

  if(!DUtils::FileFunctions::FileExists(voc_file.c_str()))
    throw std::invalid_argument("Could not find vocabulary file");

  // check if voc file name is formatted as DETECTOR_DESCRIPTOR_...
  std::string voc_path, voc_name, voc_ext;
  DUtils::FileFunctions::FileParts(voc_file, voc_path, voc_name, voc_ext);

  std::string descriptor;
  {
    std::vector<std::string> tokens;
    DUtils::StringFunctions::split(voc_name, tokens, "_");
    if(tokens.size() > 2)
    {
      descriptor = DUtils::StringFunctions::toUpper(tokens[1]);
    }
    else if(tokens.size() == 2)
    {
      descriptor = DUtils::StringFunctions::toUpper(tokens[0]);
    }
  }

  if(!descriptor.empty()) setDescriptorType(descriptor);

  // load the vocabulary
  DBoW2::TemplatedVocabulary<cv::Mat, FMat8UBinary> voc(voc_file);

  {
    std::lock_guard<std::mutex> lock(m_mutex_database);
    m_database.reset(new DBoW2::TemplatedDatabase<cv::Mat, FMat8UBinary>
                     (voc, params.use_di, params.di_level));
  }
}

void MultiDBoWMatcher::setDescriptorType(const std::string &descriptor)
{
  // if the descriptor binary?
  if(!isBinary(descriptor))
    throw std::invalid_argument("Only binary descriptors are supported");

  cv::Ptr<cv::DescriptorExtractor> desex =
      cv::DescriptorExtractor::create(descriptor);

  if(desex.empty())
    throw std::invalid_argument("Could not create a descriptor extractor for " +
                         descriptor);

  // store the detectors
  {
    std::lock_guard<std::mutex> lock(m_mutex_descriptor);
    m_descriptor_extractor = desex;
  }
}

void MultiDBoWMatcher::clear()
{
  std::lock(m_mutex_database, m_mutex_sessions);
  std::lock_guard<std::mutex> lock0(m_mutex_database, std::adopt_lock);
  std::lock_guard<std::mutex> lock1(m_mutex_sessions, std::adopt_lock);

  if(m_database) m_database->clear();
  m_db_to_node.clear();
  m_existing_frames.clear();
  m_sessions.clear();
}

void MultiDBoWMatcher::getFeatures(const ReferenceFrameId& id,
                                   const cv::Mat &im,
                                   std::vector<cv::KeyPoint>& keys,
                                   std::vector<cv::Mat>& descs,
                                   std::vector<LandmarkId>& lms) const
{
  keys.clear();
  descs.clear();
  lms.clear();
  if(m_descriptor_extractor.empty()) return;

  getFrameKeypoints(id, keys, &lms);

  if(!keys.empty())
  {
    // the descriptor extractor can remove keypoints from the vector.
    // if that happens, we have to remove the landmarks too
    size_t class_id = 0;
    std::for_each(keys.begin(), keys.end(), [&](cv::KeyPoint &k){
      k.class_id = class_id++;
    });

    cv::Mat all_descs;
    m_descriptor_extractor->compute(im, keys, all_descs);

    descs.reserve(all_descs.rows);
    for(int i = 0; i < all_descs.rows; ++i)
      descs.push_back(all_descs.row(i));

    // remove landmarks that are not longer needed
    if(keys.size() != lms.size())
    {
      std::vector<unsigned int> i_lms;
      i_lms.reserve(keys.size());
      for(const auto& k : keys) i_lms.push_back(k.class_id);

      // @todo avoid so many copies
      std::vector<LandmarkId> good_lms;
      DUtils::STL::copyIndices(lms, i_lms, good_lms);
      lms = std::move(good_lms);
    }
  }
}

size_t MultiDBoWMatcher::NumPlaces() const
{
  std::lock_guard<std::mutex> lock(m_mutex_database);
  return (m_database ? m_database->size() : 0);
}

void MultiDBoWMatcher::AddPlace(const ReferenceFrameId& id,
                                const cv::Mat& image)
{
#if DEBUG_SAVE_IMAGES
  m_images[id] = image.clone();
#endif

  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  std::vector<LandmarkId> landmarks;
  cv::Mat im2 = normalizeImage(image);
  getFeatures(id, im2, keys, descs, landmarks);

  AddPlace(id, keys, descs, landmarks);
}

void MultiDBoWMatcher::AddPlace(const ReferenceFrameId& id,
                                const std::vector<cv::KeyPoint>& keys,
                                const std::vector<cv::Mat>& descs,
                                const std::vector<LandmarkId>& lms)
{
  if(!m_database || keys.empty()) return;


  std::lock_guard<std::mutex> lock(m_mutex_database);
  auto fit = m_existing_frames.lower_bound(id);
  if(fit == m_existing_frames.end() || *fit != id)
  {
    m_existing_frames.emplace_hint(fit, id);
    DBoW2::EntryId eid = m_database->add(descs);
    m_db_to_node.emplace(eid, Node(id, keys, descs, lms));
  }
}

void MultiDBoWMatcher::GetPlace(const ReferenceFrameId& id,
                                std::vector<cv::KeyPoint>& keys,
                                std::vector<cv::Mat>& descs,
                                std::vector<LandmarkId>* landmarks) const
{
  keys.clear();
  descs.clear();
  if(landmarks) landmarks->clear();
  if(!m_database) return;

  std::lock_guard<std::mutex> lock(m_mutex_database);
  for(const auto& dbnode : m_db_to_node)
    if(dbnode.second.frameid == id)
    {
      keys = dbnode.second.keys;
      descs = dbnode.second.descs;
      if(landmarks) *landmarks = dbnode.second.landmarks;
    }
}

void MultiDBoWMatcher::GetPotentialPlaceMatches(
  const ReferenceFrameId& id,
  const cv::Mat& image,
  std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
#if DEBUG_SAVE_IMAGES
  m_images[id] = image.clone();
#endif

  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  std::vector<LandmarkId> landmarks;
  cv::Mat im2 = normalizeImage(image);
  getFeatures(id, im2, keys, descs, landmarks);

  doDetection(id, keys, descs, landmarks, false, vPlaceMatches);
}

void MultiDBoWMatcher::GetPotentialPlaceMatches(
  const ReferenceFrameId& id,
  const std::vector<cv::KeyPoint>& keys,
  const std::vector<cv::Mat>& descs,
  const std::vector<LandmarkId>& landmarks,
  std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
  doDetection(id, keys, descs, landmarks, false, vPlaceMatches);
}

void MultiDBoWMatcher::GetPotentialPlaceMatchesOrAddPlace
  (const ReferenceFrameId& id,
   const cv::Mat& image,
   std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
#if DEBUG_SAVE_IMAGES
  m_images[id] = image.clone();
#endif

  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  std::vector<LandmarkId> landmarks;
  cv::Mat im2 = normalizeImage(image);
  getFeatures(id, im2, keys, descs, landmarks);

  doDetection(id, keys, descs, landmarks, true, vPlaceMatches);
}

void MultiDBoWMatcher::GetPotentialPlaceMatchesOrAddPlace
  (const ReferenceFrameId& id,
   const std::vector<cv::KeyPoint>& keys,
   const std::vector<cv::Mat>& descs,
   const std::vector<LandmarkId>& landmarks,
   std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
  doDetection(id, keys, descs, landmarks, true, vPlaceMatches);
}

void MultiDBoWMatcher::doDetection
  (const ReferenceFrameId& id,
   const std::vector<cv::KeyPoint>& keys,
   const std::vector<cv::Mat>& descs,
   const std::vector<LandmarkId>& landmarks,
   bool add_place,
   std::vector<PlaceMatchCandidate>& vPlaceMatches)
{
  vPlaceMatches.clear();
  if(!m_map || !m_database || keys.empty()) return;

  Session& session = getSession(id.session_id);

  // get current bowvec
  DBoW2::BowVector bowvec;
  DBoW2::FeatureVector fevec;

  if(m_params.use_di)
    m_database->getVocabulary()->transform(descs, bowvec, fevec,
                                           m_params.di_level);
  else
    m_database->getVocabulary()->transform(descs, bowvec);

  // query database
  DBoW2::QueryResults query;
  m_database->query(bowvec, query, m_params.max_db_results);

  ROS_DEBUG_NAMED("MultiDBoWMatcher", "Query returned %d results", (int)query.size());

  // remove results with low score or by dislocal
  pruneQuery(session, id, bowvec, query);
  ROS_DEBUG_NAMED("MultiDBoWMatcher", "After prune %d results remain", (int)query.size());

  if(query.empty())
  {
    session.resetTrack();
  }
  else
  {
    // group into windows
    std::vector<AdjacencyWindow> windows;
    groupWindows(query, windows);

    // windows are arranged by score, get the best one
    AdjacencyWindow& window = windows[0];

    DBoW2::EntryId matched_eid;
    double matched_score;

    window.getBestNode(matched_eid, matched_score);

    const ReferenceFrameId& matched_node =
        m_db_to_node.find(matched_eid)->second.frameid;

    // track the window
    bool do_geom = false;

    if(m_params.k <= 0) {
      do_geom = true;
    } else {
      expandWindow(window, m_params.max_dist_matching_windows/2.);

      bool window_tracked = session.pushTrack
          (window, m_params.max_dist_matching_windows);

      do_geom = window_tracked &&
          session.trackSize() >= static_cast<size_t>(m_params.k);
      ROS_DEBUG_NAMED("MultiDBoWMatcher","Window tracked?: %s, w/track size: %d", window_tracked ? "yes" : "no", (int)session.trackSize());
    }

    if(do_geom) {
      auto it = m_db_to_node.find(matched_eid);
      if (it == m_db_to_node.end()) {
        ROS_ERROR("Matched node not found");
      }
      auto match_it = it->second.landmarks.begin();
      auto query_it = landmarks.begin();
      while (query_it != landmarks.end()) {
        // Check query < match
        if (*query_it < *match_it) {
          ++query_it;
        } else if (*query_it == *match_it) {
          do_geom = false;
          break;
        } else if (match_it == it->second.landmarks.end()) {
          break;
        } else {
          ++match_it;
        }
      }
    }

    if(do_geom) {
      // there is temporal consistency
      GeometricData data;
      bool geom_check = (m_params.geom_algorithm == Parameters::NONE ||
        isGeometricalConsistent(id, keys, descs, landmarks, fevec,
                                matched_eid, data));

      ROS_DEBUG_NAMED("MultiDBoWMatcher","Geometry check result: %s", geom_check ? "true":"false");

      if(geom_check) // loop found
      {
        ROS_DEBUG_NAMED("MultiDBoWMatcher","Loop found between query %s and matched %s with transformation %s",
            boost::lexical_cast<std::string>(id).c_str(),
            boost::lexical_cast<std::string>(matched_node).c_str(),
            boost::lexical_cast<std::string>(data.Tqm.matrix()).c_str());

        vPlaceMatches.emplace_back(matched_node, matched_score);
        vPlaceMatches.back().query = std::move(data.query_keys);
        vPlaceMatches.back().match = std::move(data.match_keys);
        vPlaceMatches.back().query_landmarks = std::move(data.query_landmarks);
        vPlaceMatches.back().match_landmarks = std::move(data.match_landmarks);
        vPlaceMatches.back().Tqm = std::move(data.Tqm);
      }

    } // if do geom
  } // if ! query empty

  // add place if there was no match
  if(add_place && vPlaceMatches.empty() && !keys.empty())
  {
    AddPlace(id, keys, descs, landmarks);
  }

  // update last bowvec if we are using it (this moves the reference)
  if(m_params.use_nss) session.setLastBowVector(bowvec);

#if DEBUG_SAVE_IMAGES
  if(!vPlaceMatches.empty())
  {
    cv::Mat qim = m_images[id].clone();
    cv::Mat mim = m_images[vPlaceMatches[0].getFrameId()].clone();

    std::vector<std::vector<cv::DMatch> > matches
        (vPlaceMatches[0].query.size());
    for(auto mit = matches.begin(); mit != matches.end(); ++mit)
    {
      mit->push_back(cv::DMatch(mit - matches.begin(), mit - matches.begin(),
                                0));
    }

    cv::Mat im;
    cv::drawMatches(qim, vPlaceMatches[0].query, mim, vPlaceMatches[0].match,
        matches, im);

    DUtils::FileFunctions::MkDir("matches");
    char buf[256];
    sprintf(buf, "matches/%04d_%04d.png", id.id,
            vPlaceMatches[0].getFrameId().id);
    cv::imwrite(buf, im);
  }
#endif
}

MultiDBoWMatcher::Session& MultiDBoWMatcher::getSession(const SessionId& sid)
{
  std::lock_guard<std::mutex> lock(m_mutex_sessions);

  auto sit = m_sessions.lower_bound(sid);
  if(sit == m_sessions.end() || sit->first != sid)
    sit = m_sessions.emplace_hint(sit, sid, std::make_shared<Session>(sid));
  return *sit->second;
}

void MultiDBoWMatcher::pruneQuery(const Session& session,
                                  const ReferenceFrameId& current_frameid,
                                  const DBoW2::BowVector& bowvec,
                                  DBoW2::QueryResults& query) const
{
  double nss_factor = 1.;
  if(m_params.use_nss)
  {
    const DBoW2::BowVector& last_bowvec = session.getLastBowVector();
    if(last_bowvec.empty())
    {
      // we cannot compute the nss yet
      query.clear();
      return;
    }

    nss_factor = m_database->getVocabulary()->score(bowvec, last_bowvec);

    if(nss_factor < m_params.min_nss_factor)
    {
      query.clear();
      return;
    }
  }

  double nss_alpha = m_params.alpha * nss_factor;

  std::vector<unsigned int> i_remove;

  size_t qidx = 0;
  for(DBoW2::QueryResults::const_iterator qit = query.begin();
      qit != query.end(); ++qit, ++qidx)
  {
    if(qit->Score < nss_alpha)
    {
      // this and all the next results does not reach the minimum score,
      // remove all
      query.resize(qidx);
      break;
    }
    else
    {
      // check dislocal
      const auto mit = m_db_to_node.find(qit->Id);

      if(mit != m_db_to_node.end())
      {
        const ReferenceFrameId& frameid = mit->second.frameid;

        if(frameid.session_id == session.id)
        {
          // results with nodes created by other clients are not removed
          SlamFramePtr match_frame = m_map->GetFramePtr(frameid);
          SlamFramePtr current_frame = m_map->GetFramePtr(current_frameid);
          if (!match_frame || !current_frame) continue;

          if(current_frame->time() == 0.)
          {
            ROS_DEBUG_NAMED("MultiDBoWMatcher","Timestamp is 0");
          }

          double et = current_frame->time() - match_frame->time();
          if(et <= m_params.dislocal) i_remove.push_back(qidx);
        }
      }
      else
      {
        // this should not happen
        i_remove.push_back(qidx);
      }
    }
  }

  DUtils::STL::removeIndices(query, i_remove, true);
}

void MultiDBoWMatcher::groupWindows(const DBoW2::QueryResults& query,
  std::vector<AdjacencyWindow>& windows) const
{
  AdjacencyWindowManager manager;

  for(DBoW2::QueryResults::const_iterator qit = query.begin();
      qit != query.end(); ++qit)
  {
    DBoW2::EntryId eid = qit->Id;
    const ReferenceFrameId& frameid = m_db_to_node.find(eid)->second.frameid;
    double score = qit->Score;

    // get window
    size_t widx = manager.updateWindow(eid, frameid, score);

    // expand adjacent nodes
    auto nit = m_db_to_node.find(eid);
    if(nit != m_db_to_node.end())
    {
      manager.expand(widx, m_map, nit->second.frameid,
                     m_params.max_dist_window_nodes);
    }

  } // for each query result

  // remove auxiliary nodes that do not belong to the windows
  manager.pruneWindows();

  // return windows
  manager.getWindows(windows);
}

void MultiDBoWMatcher::expandWindow(AdjacencyWindow& window, double distance)
  const
{
  // get the indices
  std::vector<ReferenceFrameId> nodes;
  nodes.reserve(window.size());
  for(auto& wit : window) nodes.push_back(wit.first);

  // expand each node
  for(const auto& node : nodes) window.expand(m_map, node, distance);
}

bool MultiDBoWMatcher::isGeometricalConsistent(
    const ReferenceFrameId& cur_frameid,
    const std::vector<cv::KeyPoint>& cur_keys,
    const std::vector<cv::Mat>& cur_descs,
    const std::vector<LandmarkId>& cur_lms,
    const DBoW2::FeatureVector& cur_fevec,
    const DBoW2::EntryId& match_eid,
    GeometricData& data) const
{
  data.clear();
  auto it = m_db_to_node.find(match_eid);
  if (it == m_db_to_node.end()) {
    ROS_ERROR("MultiDBoWMatcher: Matched node not found");
  }
  const Node& match = it->second;

  // get correspondences
  std::vector<unsigned int> i_corr_match, i_corr_cur;

  if(cur_fevec.empty())
    getCorrespondencesExhaustive(cur_descs, match.descs,
                                 i_corr_cur, i_corr_match);
  else
    getCorrespondencesDI(cur_descs, cur_fevec,
                         match.descs, m_database->retrieveFeatures(match_eid),
                         i_corr_cur, i_corr_match);

  // store putative correspondences
  {
    DUtils::STL::copyIndices(cur_keys, i_corr_cur, data.query_keys);
    DUtils::STL::copyIndices(match.keys, i_corr_match, data.match_keys);
  }

  // apply geom check
  switch(m_params.geom_algorithm)
  {
    case Parameters::HOMOGRAPHY:
    case Parameters::FUNDAMENTAL_MATRIX:
      return checkEpipolar(cur_keys, i_corr_cur, match.keys, i_corr_match,
                           data);
    case Parameters::PNP:
      return checkRigidTransformation(cur_frameid, cur_keys, cur_lms,
                                      i_corr_cur, match.frameid, match.keys,
                                      match.landmarks, i_corr_match,
                                      data);
    default:
      return false;
  }

}

void MultiDBoWMatcher::getCorrespondencesExhaustive(
    const std::vector<cv::Mat>& cur_descs,
    const std::vector<cv::Mat>& match_descs,
    std::vector<unsigned int>& i_corr_cur,
    std::vector<unsigned int>& i_corr_match) const
{
  std::vector<unsigned int> i_match, i_cur;
  i_match.reserve(match_descs.size());
  for(size_t i = 0; i < match_descs.size(); ++i) i_match.push_back(i);

  i_cur.reserve(cur_descs.size());
  for(size_t i = 0; i < cur_descs.size(); ++i) i_cur.push_back(i);

  getMatches(match_descs, i_match, cur_descs, i_cur,
             i_corr_match, i_corr_cur);
}

void MultiDBoWMatcher::getCorrespondencesDI(
    const std::vector<cv::Mat>& cur_descs,
    const DBoW2::FeatureVector& cur_fevec,
    const std::vector<cv::Mat>& match_descs,
    const DBoW2::FeatureVector& match_fevec,
    std::vector<unsigned int>& i_corr_cur,
    std::vector<unsigned int>& i_corr_match) const
{
  i_corr_cur.clear();
  i_corr_match.clear();

  DBoW2::FeatureVector::const_iterator old_it, cur_it;
  const DBoW2::FeatureVector::const_iterator old_end = match_fevec.end();
  const DBoW2::FeatureVector::const_iterator cur_end = cur_fevec.end();

  old_it = match_fevec.begin();
  cur_it = cur_fevec.begin();

  while(old_it != old_end && cur_it != cur_end)
  {
    if(old_it->first == cur_it->first)
    {
      // compute matches
      std::vector<unsigned int> i_old_now, i_cur_now;

      getMatches(match_descs, old_it->second,
        cur_descs, cur_it->second,
        i_old_now, i_cur_now);

      i_corr_match.insert(i_corr_match.end(),
                          i_old_now.begin(), i_old_now.end());
      i_corr_cur.insert(i_corr_cur.end(), i_cur_now.begin(), i_cur_now.end());

      // move old_it and cur_it forward
      ++old_it;
      ++cur_it;
    }
    else if(old_it->first < cur_it->first)
    {
      // move old_it forward
      ++old_it;
    }
    else
    {
      // move cur_it forward
      ++cur_it;
    }
  }
}

bool MultiDBoWMatcher::checkEpipolar(const std::vector<cv::KeyPoint>& cur_keys,
                   const std::vector<unsigned int>& i_cur,
                   const std::vector<cv::KeyPoint>& match_keys,
                   const std::vector<unsigned int>& i_match,
                   GeometricData& data) const
{
  // calculate now the fundamental or homography matrix
  if(static_cast<int>(i_match.size()) >= m_params.min_Fpoints)
  {
    std::vector<cv::Point2f> old_points, cur_points;

    // add matches to the vectors to calculate the fundamental matrix
    std::vector<unsigned int>::const_iterator oit, cit;
    oit = i_match.begin();
    cit = i_cur.begin();

    for(; oit != i_match.end(); ++oit, ++cit)
    {
      const cv::KeyPoint &old_k = match_keys[*oit];
      const cv::KeyPoint &cur_k = cur_keys[*cit];

      old_points.push_back(old_k.pt);
      cur_points.push_back(cur_k.pt);
    }

    cv::Mat oldMat(old_points.size(), 2, CV_32F, &old_points[0]);
    cv::Mat curMat(cur_points.size(), 2, CV_32F, &cur_points[0]);

    std::vector<unsigned char> status;
    cv::Mat F;

    if(m_params.geom_algorithm == Parameters::FUNDAMENTAL_MATRIX)
    {
      F = cv::findFundamentalMat(oldMat, curMat, cv::FM_RANSAC,
                             m_params.max_reprojection_error,
                             m_params.ransac_probability, status);
    }
    else
    {
      F = cv::findHomography(oldMat, curMat, CV_RANSAC,
                             m_params.max_reprojection_error, status);
    }

    if(cv::countNonZero(status) < m_params.min_Fpoints)
      F.release();

    if(!F.empty())
    {
      // this assume that the calling function already filled
      // query and matched vectors
      std::vector<unsigned int> i_remove;
      for(size_t i = 0; i < status.size(); ++i)
        if(status[i] == 0) i_remove.push_back(i);
      DUtils::STL::removeIndices(data.query_keys, i_remove, true);
      DUtils::STL::removeIndices(data.match_keys, i_remove, true);
    }

    return !F.empty();
  }

  return false;
}

bool MultiDBoWMatcher::checkRigidTransformation(
    const ReferenceFrameId& cur_frameid,
    const std::vector<cv::KeyPoint>& cur_keys,
    const std::vector<LandmarkId>& cur_lms,
    const std::vector<unsigned int>& i_cur,
    const ReferenceFrameId& match_frameid,
    const std::vector<cv::KeyPoint>& match_keys,
    const std::vector<LandmarkId>& match_lms,
    const std::vector<unsigned int>& i_match,
    GeometricData& data) const
{
  // Reminder:
  // rslam uses x forward, y right, z down
  // open uses z forward, x right, y down
  // data.Tqm must be in rslam axes

  const CameraRigPtr& rig = m_map->GetCamera(cur_frameid.session_id);
  if (!rig) {
    ROS_INFO("No camera available");
    return false;
  }

  const calibu::CameraModelInterfaceT<Scalar>& cam = rig->cameras[0].camera;

  std::vector<cv::Point2f> img;
  std::vector<cv::Point3f> d3;
  img.reserve(i_cur.size());

  std::vector<unsigned int> i_added_cur, i_added_match;
  i_added_cur.reserve(i_cur.size());
  i_added_match.reserve(i_match.size());

  ROS_DEBUG_NAMED("MultiDBoWMatcher","%d putative landmarks", (int)i_cur.size());

  // get reliable landmarks
  std::vector<Landmark> valid_landmarks;
  std::vector<unsigned int>::const_iterator cit, mit;
  for(cit = i_cur.begin(), mit = i_match.begin(); cit != i_cur.end();
      ++cit, ++mit)
  {
    valid_landmarks.push_back(Landmark());
    bool ok = false;

    if(m_map->GetLandmark(match_lms[*mit], &valid_landmarks.back()))
    {
      LandmarkState state = valid_landmarks.back().state();
      if(state != eLmkAtInfinity && state != eLmkNotReliable)
      {
        ok = true;

        Eigen::Vector2t p2(cur_keys[*cit].pt.x, cur_keys[*cit].pt.y);
        p2 = calibu::Project(cam.Unproject(p2));

        img.emplace_back(p2.x(), p2.y());

        i_added_cur.emplace_back(*cit);
        i_added_match.emplace_back(*mit);
      }
    }
    if(!ok) valid_landmarks.pop_back();
  }

  // Store which frame -> [(landmark idx, keypoint idx)]
  if(img.size() < static_cast<size_t>(m_params.min_Fpoints)) {
    ROS_DEBUG_NAMED("MultiDBoWMatcher","Only %d good landmarks matched", (int)img.size());
    return false;
  }else{
    ROS_DEBUG_NAMED("MultiDBoWMatcher"," %d good landmarks matched", (int)img.size());
  }

  // get the 3d position of the landmarks in the matched reference frame
  {
    LandmarkVisitor visitor(valid_landmarks, match_frameid,
                            rig->cameras[0].T_wc);
    m_map->BFS(&visitor);

    std::vector<unsigned int> lost_landmarks = visitor.getLostLandmarks();
    d3 = std::move(visitor.getPoints());

    if(!lost_landmarks.empty())
    {
      ROS_DEBUG_NAMED("MultiDBoWMatcher","Removing %d landmarks",(int)lost_landmarks.size());

      DUtils::STL::removeIndices(d3, lost_landmarks, false);
      DUtils::STL::removeIndices(img, lost_landmarks, false);
      DUtils::STL::removeIndices(i_added_cur, lost_landmarks, false);
      DUtils::STL::removeIndices(i_added_match, lost_landmarks, false);
    }
  }

  // do the pnp now
  const int its = 50;
  std::vector<int> i_i_inliers;
  cv::Mat rvec, tvec;
  cv::solvePnPRansac(d3, img, cv::Mat::eye(3, 3, CV_32F), cv::noArray(),
                     rvec, tvec, false, its,
                     m_params.max_reprojection_error / cam.K()(0,0),
                     m_params.min_Fpoints, i_i_inliers);

  Eigen::Vector3t rot, trans;
  if(rvec.type() == CV_32F)
  {
    for(short i = 0; i < 3; ++i){
      rot[i] = rvec.ptr<float>()[i];
      trans[i] = tvec.ptr<float>()[i];
    }
  }else{
    for(short i = 0; i < 3; ++i){
      rot[i] = rvec.ptr<double>()[i];
      trans[i] = tvec.ptr<double>()[i];
    }
  }

  if(std::isnan(rot[0]) || std::isnan(rot[1]) || std::isnan(rot[2]) ||
     i_i_inliers.size() < static_cast<size_t>(m_params.min_Fpoints)) {
    ROS_DEBUG_NAMED("MultiDBoWMatcher","Only %d geometric inliers (or nan)", (int)i_i_inliers.size());
    return false;
  }

  // update data
  {
    data.query_keys.clear();
    data.query_landmarks.clear();
    data.match_keys.clear();
    data.match_landmarks.clear();

    data.query_keys.reserve(i_i_inliers.size());
    data.query_landmarks.reserve(i_i_inliers.size());
    data.match_keys.reserve(i_i_inliers.size());
    data.match_landmarks.reserve(i_i_inliers.size());

    for(auto ii : i_i_inliers)
    {
      data.query_keys.emplace_back(cur_keys[i_added_cur[ii]]);
      data.query_landmarks.emplace_back(cur_lms[i_added_cur[ii]]);

      data.match_keys.emplace_back(match_keys[i_added_match[ii]]);
      data.match_landmarks.emplace_back(match_lms[i_added_match[ii]]);
    }
  }

  data.Tqm =
      rig->cameras[0].T_wc *
      Sophus::SE3t(Sophus::SO3t::exp(rot), trans)
      * rig->cameras[0].T_wc.inverse();

  return true;
}

void MultiDBoWMatcher::getMatches(const std::vector<cv::Mat> &A,
  const std::vector<unsigned int>& i_A,
  const std::vector<cv::Mat> &B,
  const std::vector<unsigned int>& i_B,
  std::vector<unsigned int>& i_match_A,
  std::vector<unsigned int>& i_match_B) const
{
  std::vector<double> match_distances;
  i_match_A.clear();
  i_match_B.clear();

  if(i_A.empty() || i_B.empty()) return;

  i_match_A.reserve(std::min(i_A.size(), i_B.size()));
  i_match_B.reserve(std::min(i_A.size(), i_B.size()));
  match_distances.reserve(std::min(i_A.size(), i_B.size()));

  typedef FMat8UBinary F;

  for(std::vector<unsigned int>::const_iterator ait = i_A.begin();
      ait != i_A.end(); ++ait)
  {
    std::vector<unsigned int>::const_iterator bit = i_B.begin();
    unsigned int best_b = *bit;
    double best_d1 = F::distance(A[*ait], B[*bit]);
    double best_d2 = best_d1;

    for(++bit; bit < i_B.end(); ++bit)
    {
      double d = F::distance(A[*ait], B[*bit]);
      if(d < best_d1)
      {
        best_d2 = best_d1;
        best_d1 = d;
        best_b = *bit;
      }
      else if(d < best_d2)
      {
        best_d2 = d;
      }
    }

    if(best_d1 / best_d2 <= m_params.max_neighbor_ratio)
    {
      std::vector<unsigned int>::const_iterator mit =
          std::find(i_match_B.begin(), i_match_B.end(), best_b);

      if(mit == i_match_B.end())
      {
        i_match_A.push_back(*ait);
        i_match_B.push_back(best_b);
        match_distances.push_back(best_d1);
      }
      else
      {
        const unsigned int idx = mit - i_match_B.begin();

        if(best_d1 < match_distances[idx])
        {
          match_distances[idx] = best_d1;
          i_match_A[idx] = *ait;
        }
      }
    }

  } // for each item in A
}

cv::Mat MultiDBoWMatcher::normalizeImage(const cv::Mat &im) const
{
  if(m_params.image_normalization == Parameters::EQUALIZATION)
  {
    cv::Mat ret;
    cv::equalizeHist(im, ret);
    return ret;
  }
  else if(m_params.image_normalization == Parameters::MORPHOLOGICAL_CLOSING)
  {
    static const cv::Mat block =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11));
    cv::Mat closing;
    cv::morphologyEx(im, closing, cv::MORPH_CLOSE, block);

    cv::Mat f, fc, f2;
    im.convertTo(f, CV_32F);
    closing.convertTo(fc, CV_32F);
    f2 = f / fc;

    cv::Mat ret;
    cv::normalize(f2, f2, 0, 255, CV_MINMAX);
    f2.convertTo(ret, CV_8U);

    return ret;
  }
  else
    return im;
}

void MultiDBoWMatcher::Save(const std::string& filename) const
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);
  if(!fs.isOpened()) {
    return;
  }

  save(fs, "multitrackdetector");
}

void MultiDBoWMatcher::Load(const std::string& filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if(!fs.isOpened()) {
    return;
  }

  load(fs, "multitrackdetector");
}

void MultiDBoWMatcher::save(cv::FileStorage& fs, const std::string& name) const
{
  // Format YAML:
  // vocabulary
  // {
  //   ...
  // }
  // database
  // {
  //   ...
  // }
  // (name)
  // {
  //   descriptor_extractor { ... }
  //   params
  //   [
  //     ...
  //   ]
  //   sessions
  //   [
  //     ...
  //   ]
  //   nodes
  //   [
  //     [db_id node]
  //     ...
  //   ]
  // }

  // we need all the mutexes
  std::lock(m_mutex_database, m_mutex_descriptor, m_mutex_params,
            m_mutex_sessions);
  std::lock_guard<std::mutex> lock0(m_mutex_database, std::adopt_lock);
  std::lock_guard<std::mutex> lock1(m_mutex_descriptor, std::adopt_lock);
  std::lock_guard<std::mutex> lock2(m_mutex_params, std::adopt_lock);
  std::lock_guard<std::mutex> lock3(m_mutex_sessions, std::adopt_lock);

  if(m_database) m_database->save(fs); // this stores the voc too

  fs << name << "{";

  if(!m_descriptor_extractor.empty())
  {
    std::string name = m_descriptor_extractor->name(); // "Feature2D.XX"
    std::vector<std::string> tokens;
    DUtils::StringFunctions::split(name, tokens, ".");

    if(tokens.size() == 2)
    {
      fs << "descriptor_extractor_name" << tokens[1]; // "XX"
      fs << "descriptor_extractor" << "{:";
      m_descriptor_extractor->write(fs);
      fs << "}";
    }
  }

  // params
  m_params.save(fs);

  fs << "sessions" << "[";
  for(const auto& s : m_sessions) s.second->save(fs);
  fs << "]";

  // nodes
  fs << "nodes" << "[";
  for(const std::pair<DBoW2::EntryId, Node>& dbnode : m_db_to_node)
  {
    fs << "[" << static_cast<int>(dbnode.first);
    dbnode.second.save(fs);
    fs << "]";
  }
  fs << "]"; // nodes

  fs << "}"; // whole entry
}

void MultiDBoWMatcher::load(cv::FileStorage& fs, const std::string& name)
{
  clear(); // this uses the mutexes

  // we need all the mutexes
  std::lock(m_mutex_database, m_mutex_descriptor, m_mutex_params,
            m_mutex_sessions);
  std::lock_guard<std::mutex> lock0(m_mutex_database, std::adopt_lock);
  std::lock_guard<std::mutex> lock1(m_mutex_descriptor, std::adopt_lock);
  std::lock_guard<std::mutex> lock2(m_mutex_params, std::adopt_lock);
  std::lock_guard<std::mutex> lock3(m_mutex_sessions, std::adopt_lock);

  if(!fs["database"].empty())
  {
    m_database.reset(new DBoW2::TemplatedDatabase<cv::Mat, FMat8UBinary>());
    m_database->load(fs);
  }
  else
    m_database.reset();

  cv::FileNode fn = fs[name];

  std::string feature_name =
      static_cast<std::string>(fn["descriptor_extractor_name"]);
  if(!feature_name.empty())
  {
    if(!isBinary(feature_name))
      throw std::runtime_error("Only binary descriptors are supported");

    cv::FileNode fn2 = fn["descriptor_extractor"];
    if(!fn2.empty())
    {
      m_descriptor_extractor = cv::DescriptorExtractor::create(feature_name);
      if(!m_descriptor_extractor.empty())
        m_descriptor_extractor->read(fn2);
      else
        throw std::runtime_error("Could not create a descriptor extractor "
                                 "for " + feature_name);
    }
  }

  // params
  m_params.load(fn["params"]);

  // sessions
  {
    cv::FileNode fn2 = fn["sessions"];
    for(size_t i = 0; i < fn2.size(); ++i)
    {
      std::shared_ptr<Session> session = std::make_shared<Session>();
      session->load(fn2[i]);
      m_sessions.emplace(session->id, session);
    }
  }

  // nodes
  {
    cv::FileNode fn2 = fn["nodes"];
    for(size_t i = 0; i < fn2.size(); ++i)
    {
      // [db_id node]
      DBoW2::EntryId eid = static_cast<DBoW2::EntryId>
          (static_cast<int>(fn2[i][0]));

      Node& node =
        m_db_to_node.emplace_hint(m_db_to_node.end(), eid, Node())->second;

      node.load(fn2[i][1]);

      m_existing_frames.insert(node.frameid);
    }
  }
}
