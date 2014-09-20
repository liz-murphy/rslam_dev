#ifndef MULTIDBOWMATCHER_H
#define MULTIDBOWMATCHER_H

#include <string>
#include <mutex>
#include <miniglog/logging.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <slam_map/SlamMap.h>
#include <slam_map/MapVisitor/TransformMapVisitor.h>
#include <pb_msgs/ImageArray.h>
#include <pb_msgs/Image.h>
#include <place_matching/PlaceMatcher.h>
#include <place_matching/DBoWMatcher/DBoW2/DBoW2.h>
#include <place_matching/DBoWMatcher/FMat8UBinary.h>

#define DEBUG_SAVE_IMAGES 0

/// Multi session DBoW Matcher for binary features
class MultiDBoWMatcher : public PlaceMatcher
{
public:

  /// General Parameters
  struct Parameters
  {
    /// Use normalized similarity score
    bool use_nss;
    /// Score threshold
    double alpha;
    /// Temporal consistency parameter
    int k;
    /// Use direct index
    bool use_di;
    /// Direct index level (0 = leaves)
    int di_level;
    /// Time until which recent nodes are ignored when querying (disallow local)
    double dislocal; // seconds
    /// Number of elements to retrieve from the database
    int max_db_results;
    /// Min nss factor to compute a nss score
    double min_nss_factor;

    /// Max neighbor ratio to find correspondences
    double max_neighbor_ratio;
    /// Algorithm for geometrical check
    enum GEOM_ALGORITHM { NONE, FUNDAMENTAL_MATRIX, HOMOGRAPHY, PNP }
                        geom_algorithm;
    /// Min number of consistent points in the geometrical check
    int min_Fpoints;
    /// Max reprojection error in geometrical check iterations
    double max_reprojection_error;
    /// Ransac success probability
    double ransac_probability;

    /// Maximum distance between nodes in a window (in map units)
    double max_dist_window_nodes;
    /// Maximum distance between two windows that match (in map units)
    double max_dist_matching_windows;

    /// Brightness normalization
    enum IMAGE_NORMALIZATION
      { NO_NORMALIZATION, EQUALIZATION, MORPHOLOGICAL_CLOSING }
      image_normalization;

    /**
     * @brief Initiates default
     */
    Parameters();

    /**
     * @brief Saves parameters into a file storage
     * @param fs
     */
    void save(cv::FileStorage &fs) const;

    /**
     * @brief Loads parameters from file node
     * @param fn
     */
    void load(const cv::FileNode& fn);
  };

public:
  /**
   * @brief Creates an empty matchet with a reference to the map
   * @param map Slam map
   */
  MultiDBoWMatcher(std::shared_ptr<SlamMap> map);
  ~MultiDBoWMatcher(){}

  /**
   * @brief Creates a matcher with a vocabulary and a set of parameters
   * @param voc_file vocabulary file name (may code the descriptor,
   * e.g. ORB_k10L5.voc.gz)
   * @param params matcher parameters
   * @param descriptor if given, this descriptor is used overriding the any
   * that may be coded in voc_file
   * @see seeVocabulary seeDescriptorType
   */
  MultiDBoWMatcher(std::shared_ptr<SlamMap> map, const std::string &voc_file,
                   const Parameters &params, const std::string &descriptor="");

  /**
   * @brief Sets the vocabulary of the matcher
   * @param filename vocabulary filename. It can encode the features to use.
   * @param params parameters of the detector
   * @note If filename is DETECTOR_EXTRACTOR_... (e.g. FAST_ORB_voc.gz),
   * EXTRACTOR is used as descriptor extractor and the DETECTOR is ignored
   * (we use the points detected by the SLAM tracking all the time).
   * If filename is FEATURE_... (e.g. ORB_voc.gz), FEATURE is used as
   * descriptor extractor.
   * If no EXTRACTOR or FEATURE can be obtained from the filename,
   * neither is assumed and setFeature must be called.
   * @throws std::invalid_argument if filename cannot be opened
   */
  void setVocabulary(const std::string &voc_file, const Parameters &params);

  /**
   * @brief Sets the binary descriptor to use if it was not given by the
   * vocabulary file
   * @param descriptor name of descriptor
   * @throws std::invalid_argument if detector or extractor cannot be set
   * @note Valid strings are those of binary features accepted by OpenCV,
   * such as "ORB", "BRIEF", "BRISK", "FREAK"
   */
  void setDescriptorType(const std::string &descriptor);

  /**
   * @brief Clears the contents of the matcher
   */
  void clear();

  /** Inherited functions **/

  void getFeatures(const ReferenceFrameId& id,
                   const cv::Mat& image,
                   std::vector<cv::KeyPoint> &keys,
                   std::vector<cv::Mat>& descs,
                   std::vector<LandmarkId>& lms) const;

  void getFeatures(const ReferenceFrameId& id,
                   const cv::Mat& image,
                   std::vector<cv::KeyPoint> &keys,
                   std::vector<cv::Mat>& descs) const override
  {
    std::vector<LandmarkId> lms;
    getFeatures(id, image, keys, descs, lms);
  }

  size_t NumPlaces() const override;

  void GetPotentialPlaceMatches
      (const ReferenceFrameId& id,
       const cv::Mat& image,
       std::vector<PlaceMatchCandidate>& vPlaceMatches) override;

  void GetPotentialPlaceMatchesOrAddPlace
    (const ReferenceFrameId& id,
     const cv::Mat& image,
     std::vector<PlaceMatchCandidate>& vPlaceMatches) override;

  void AddPlace(const ReferenceFrameId& id, const cv::Mat& image) override;

  /**
   * @brief Returns the keypoints of an already added frame
   * @param id
   * @param keys
   * @param descs
   * @param landmarks if given, landmarks are returned
   */
  void GetPlace(const ReferenceFrameId& id, std::vector<cv::KeyPoint>& keys,
                std::vector<cv::Mat>& descs,
                std::vector<LandmarkId>* landmarks) const;

  void AddPlace(const ReferenceFrameId& id,
                const std::vector<cv::KeyPoint>& keys,
                const std::vector<cv::Mat>& descs,
                const std::vector<LandmarkId>& lms);

  void GetPotentialPlaceMatches
      (const ReferenceFrameId& id,
       const std::vector<cv::KeyPoint>& keys,
       const std::vector<cv::Mat>& descs,
       const std::vector<LandmarkId>& landmarks,
       std::vector<PlaceMatchCandidate>& vPlaceMatches);

  void GetPotentialPlaceMatchesOrAddPlace
    (const ReferenceFrameId& id,
     const std::vector<cv::KeyPoint>& keys,
     const std::vector<cv::Mat>& descs,
     const std::vector<LandmarkId>& landmarks,
     std::vector<PlaceMatchCandidate>& vPlaceMatches);

  void Save(const std::string& filename) const override;

  void Load(const std::string& filename) override;

private:

  // Inherited non-implemented functions
  void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      const std::vector<unsigned int> &vSubset,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) override {}

  void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) override {}

  void AddPlace(const unsigned int uFrameId,
                const cv::Mat &newPlace) override {};

  void GetPotentialPlaceMatchesOrAddPlace(
      const cv::Mat &place,
      std::vector<PlaceMatchCandidate>& vPlaceMatches,
      const unsigned int uFrameId = std::numeric_limits<unsigned int>::max(),
      const std::vector<unsigned int> &vSubset = std::vector<unsigned int>()){}

private:

  /// Set of adjacent nodes with their distance (map units) to the closest
  /// generator. This is used to avoid redundant expansions.
  class AdjacencyWindow: public std::map<ReferenceFrameId, double>
  {
  public:
    /// Accumulated score
    double score;

    /// Generator nodes
    /// <id, single score>
    std::vector<std::pair<DBoW2::EntryId, double> > generators;

  public:

    AdjacencyWindow(): score(0){}

    /**
     * @brief Clears the window and resets the score
     */
    inline void clear()
    {
      score = 0.;
      generators.clear();
      std::map<ReferenceFrameId, double>::clear();
    }

    /**
     * @brief Expands the window to add the adjacent nodes of a given one
     * @param node node to start with
     * @param max_distance max distance to the node of the added elements
     * @param added[out] (default: NULL) if given, the indices of the nodes
     * added to the window are stored here
     */
    void expand(const std::shared_ptr<SlamMap>& map,
                const ReferenceFrameId& init_node, double max_distance,
                std::vector<ReferenceFrameId>* added = NULL);

    /**
     * @brief Checks if the some pair of generator nodes of two windows are
     * close
     * @param b other window
     * @return true iif the min distance between some generator node of this
     * window and some generator node of the given window is lower than
     * the given distance
     */
    bool isClose(const AdjacencyWindow& b, double max_distance) const;

    /**
     * @brief Returns the generator of the window with highest score
     * @param eid[out] id of the node
     * @param score[out] score of the node
     */
    void getBestNode(DBoW2::EntryId& eid, double& score) const;

    /**
     * @brief Remove elements that are not generators
     */
    void prune();

    /**
     * @brief Order function to arrange windows in descending order of score
     * @param a
     * @param b
     * @return true iif a.score > b.score
     */
    static bool gtScores(const AdjacencyWindow& a, const AdjacencyWindow& b);

    /**
     * @brief Saves data into a file storage
     * @param fs
     */
    void save(cv::FileStorage &fs) const;

    /**
     * @brief Loads data from a file node
     * @param fn
     */
    void load(const cv::FileNode &fn);

    class cmpGenerators
    {
    public:
      /**
       * @brief Order function to arrange pairs of <nodes, score> by descending
       * score
       * @param a
       * @param b
       * @return true iif a.score < b.score
       */
      bool operator() (const std::pair<DBoW2::EntryId, double>& a,
                       const std::pair<DBoW2::EntryId, double>& b);
    };

  }; // class AdjacencyWindow

  /// MapVisitor to expand AdjacencyWindows
  class WindowVisitor: public TransformMapVisitor
  {
  public:

    /**
     * @brief WindowVisitor
     * @param win associated adjacency window to expand
     * @param init_node initial node to start expansion
     * @param max_dist max distance in map units
     * @param max_depth max distance in edges
     */
    WindowVisitor(AdjacencyWindow& win, const ReferenceFrameId& init_node,
                  double max_dist, unsigned int max_depth = 40);

    /**
     * Called when a new node is visited
     *
     * @returns true if the children of this node should be added.
     */
    bool Visit(const SlamFramePtr& cur_node) override;

    /**
     * Called before adding a node to the search queue
     *
     * @returns true if this node should be added to the visit list.
     */
    bool ExploreNode(const SlamFramePtr& parent,
                     const SlamEdgePtr& edge,
                     const SlamFramePtr& child) override;

    /**
     * @brief Returns visited nodes after expansion
     * @return reference to vector of visited nodes that can be std::moved
     */
    std::vector<ReferenceFrameId>& visitedNodes();

  private:

    AdjacencyWindow& m_win;
    double m_max_dist;
    std::vector<ReferenceFrameId> m_visited_nodes;

  }; // class WindowVisitor


  /// Auxiliary class to manage adjacency windows
  class AdjacencyWindowManager
  {
  public:

    /**
     * @brief Returns the window (before expanding) that contains the
     * given entry. If such a window does not exist, it is created. If several
     * windows contain the entry, these are merged. The returned window
     * already contains the given entry as a generator and its score.
     * @param eid entry id
     * @param frameid reference frame id of the entry id
     * @param score entry score
     * @return index of the window that contains the given item. This index
     * indexes m_adj_windows
     */
    size_t updateWindow(DBoW2::EntryId eid, const ReferenceFrameId &frameid,
                        double score);

    /**
     * @brief Expands a window to add the adjacent nodes of a given one
     * @param widx window index in m_adj_windows
     * @param map
     * @param node node to start with
     * @param max_distance max distance to the node of the added elements
     */
    void expand(size_t widx, const std::shared_ptr<SlamMap>& map,
                const ReferenceFrameId& node, double max_distance);

    /**
     * @brief Prune windows to remove elements that are not generators
     */
    void pruneWindows();

    /**
     * @brief Returns the windows
     * @param windows
     */
    void getWindows(std::vector<MultiDBoWMatcher::AdjacencyWindow>&windows)
      const;

  protected:

    /**
     * @brief Merges the content of the i_merged-th adj window with the
     * i_target-th window. The references from the nodes to the merged window
     * are removed, but the actual window in m_adj_windows is not.
     * @param i_target index of m_adj_windows of the target window
     * @param i_merged index of m_adj_windows of the merged window
     */
    void mergeWindows(const size_t i_target, const size_t i_merged);

  protected:

    /// Adjacency windows
    std::vector<AdjacencyWindow> m_adj_windows;

    /// Map from entry id to indices of adj_windows that contain the entry
    std::map<ReferenceFrameId, std::set<size_t> > m_node_to_windows;

  }; // class AdjacencyWindowManager

  /// Track of matching windows over time
  class WindowTrack
  {
  public:
    WindowTrack(): m_count(0){}

    /**
     * @brief Inserts the given window to the track if possible
     * @param window
     * @return true iif the given window matched the last one in the track
     */
    bool push(const AdjacencyWindow& window, double max_matching_distance);

    /**
     * @brief Returns the number of matches in the track
     * @return number of windows matched
     */
    inline size_t size() const { return m_count; }

    /**
     * @brief Resets the tract
     */
    inline void reset() { m_count = 0; m_last_window.clear(); }

    /**
     * @brief Saves data into a file storage
     * @param fs
     */
    void save(cv::FileStorage &fs) const;

    /**
     * @brief Loads data from a file node
     * @param fn
     */
    void load(const cv::FileNode &fn);

  protected:
    AdjacencyWindow m_last_window;   // last window
    size_t m_count;                  // number of windows in the track

  }; // class WindowTrack

  /// Session data
  class Session
  {
  public:
    SessionId id;                   // session id

  public:

    Session(){}
    Session(const SessionId& sid): id(sid){}

    /**
     * @brief Gets the last bow vector
     * @return last bow vector
     */
    inline const DBoW2::BowVector& getLastBowVector() const
    {
      return m_last_bowvec;
    }

    /**
     * @brief Sets the last bowvec used
     * @param bow reference moved and emptied
     */
    void setLastBowVector(DBoW2::BowVector& bow);

    /**
     * @brief Resets the window track
     */
    void resetTrack();

    /**
     * @brief Adds a window to the track
     * @param window
     * @param distance
     * @return true iif window matched
     */
    bool pushTrack(const AdjacencyWindow& window, double distance);

    /**
     * @brief Size of track
     * @return number of items in the window track
     */
    inline size_t trackSize() const { return m_track.size(); }

    /**
     * @brief Saves to a file storage
     * @param fs
     */
    void save(cv::FileStorage& fs) const;

    /**
     * @brief Loads from filenode
     * @param fn
     */
    void load(const cv::FileNode& fn);

  private:
    DBoW2::BowVector m_last_bowvec;   // last bow vector got
    WindowTrack m_track;              // window track
    mutable std::mutex m_mutex;
  };

  /// Info of slam node
  class Node
  {
  public:
    ReferenceFrameId frameid;
    std::vector<cv::KeyPoint> keys;
    std::vector<cv::Mat> descs;
    std::vector<LandmarkId> landmarks;

    Node(){}
    Node(const ReferenceFrameId& f, const std::vector<cv::KeyPoint>& k,
         const std::vector<cv::Mat>& d, const std::vector<LandmarkId>& l)
      : frameid(f), keys(k), descs(d), landmarks(l){
      CHECK(!l.empty());
    }

    /**
     * @brief Saves to a file storage
     * @param fs
     */
    void save(cv::FileStorage& fs) const;

    /**
     * @brief Loads from filenode
     * @param fn
     */
    void load(const cv::FileNode& fn);
  };

  /// MapVisitor to convert landmark coordinates
  class LandmarkVisitor: public TransformMapVisitor
  {
  public:

    /**
     * @brief LandmarkVisitor
     * @param landmarks for which compute coordinates
     * @param frame origin frame
     */
    LandmarkVisitor(const std::vector<Landmark>& landmarks,
                    const ReferenceFrameId& frame,
                    const Sophus::SE3t& T_wc);

    bool Visit(const SlamFramePtr& cur_node) override;

    bool IsDone() override;

    /**
     * @brief Retrieves the coordinates
     * @return 3d points in origin frame, with axes according to the landmark
     * convention: z forward, x right, y down
     */
    std::vector<cv::Point3f>& getPoints();

    /**
     * Recover the indices of the provided landmarks which were not
     * matched during the BFS.
     */
    std::vector<unsigned int> getLostLandmarks();

  private:
    // frame, indices of landmarks
    const std::vector<Landmark>& m_landmarks;
    std::map<ReferenceFrameId, std::vector<size_t> > m_open;
    std::vector<cv::Point3f> m_points;
    Sophus::SE3t m_T_wc;
  };

  /// Data returned by different geometrical checks
  struct GeometricData
  {
    /// Inliert/putative points
    std::vector<cv::KeyPoint> query_keys, match_keys;
    /// Inlier landmarks
    std::vector<LandmarkId> query_landmarks, match_landmarks;
    /// Transformation from query pose to matched pose
    Sophus::SE3t Tqm;

    inline void clear() {
      query_keys.clear(); match_keys.clear();
      query_landmarks.clear(); match_landmarks.clear();
      Tqm = Sophus::SE3t();
    }
  };

private:

  /**
   * @brief Returns a session from its id
   * @param sid
   * @return session
   */
  Session& getSession(const SessionId &sid);

  /**
   * @brief Checks that a descriptor is binary
   * @param descriptor
   * @return
   */
  inline bool isBinary(const std::string& descriptor) const
  {
    return descriptor == "ORB" || descriptor == "FREAK" ||
      descriptor == "BRISK" || descriptor == "BRIEF";
  }

  /**
   * @brief doDetection Performs the full loop detection
   * @param id current frame id
   * @param keys current keypoints
   * @param descs current descriptors
   * @param landmarks current landmarks
   * @param add_place if true, the place is added if no loop is found
   * @param vPlaceMatches
   */
  void doDetection(const ReferenceFrameId& id,
                   const std::vector<cv::KeyPoint>& keys,
                   const std::vector<cv::Mat>& descs,
                   const std::vector<LandmarkId>& landmarks,
                   bool add_place,
                   std::vector<PlaceMatchCandidate>& vPlaceMatches);

  /**
   * @brief Removes query results with low score or to close to the current
   * place
   * @param session current session
   * @param cur_id current frame id
   * @param bowvec query bowvec
   * @param query query results
   */
  void pruneQuery(const Session& session, const ReferenceFrameId& cur_id,
                  const DBoW2::BowVector& bowvec,
                  DBoW2::QueryResults& query) const;

  /**
   * @brief Groups the results of a database query into node windows
   * @param query results of database query
   * @param windows[out] resulting windows
   */
  void groupWindows(const DBoW2::QueryResults& query,
    std::vector<AdjacencyWindow>& windows) const;

  /**
   * @brief Expands the given window the given distance for window matching
   * @param window
   * @param distance
   */
  void expandWindow(AdjacencyWindow& window, double distance) const;

  /**
   * @brief Check if there is geometrical consistency between the current
   * place and a match. If fevec is given, the direct index is used
   * @param cur_frameid current frame id
   * @param cur_keys current keypoints
   * @param cur_descs current descriptors
   * @param cur_lms current landmarks
   * @param cur_fevec current fevec of keypoints (or empty if not used)
   * @param match_eid matched db entry
   * @param data[out] geometric info used, check dependent
   * @return true iif geometrical consistency
   */
  bool isGeometricalConsistent(const ReferenceFrameId& cur_frameid,
                               const std::vector<cv::KeyPoint>& cur_keys,
                               const std::vector<cv::Mat>& cur_descs,
                               const std::vector<LandmarkId>& cur_lms,
                               const DBoW2::FeatureVector& cur_fevec,
                               const DBoW2::EntryId& match_eid,
                               GeometricData& data) const;

  /**
   * @brief Compute putative correspondences exhaustively
   * @param cur_descs current descriptors
   * @param match_descs matched descriptors
   * @param i_corr_cur indices of cur_descs of correspondences
   * @param i_corr_match indices of match_descs of correspondences
   */
  void getCorrespondencesExhaustive(
      const std::vector<cv::Mat>& cur_descs,
      const std::vector<cv::Mat>& match_descs,
      std::vector<unsigned int>& i_corr_cur,
      std::vector<unsigned int>& i_corr_match) const;

  /**
   * @brief Computes putative correspondences using the direct index
   * @param cur_descs current descriptors
   * @param cur_fevec current feature vector
   * @param match_descs matched descriptors
   * @param match_fevec matched featur vector
   * @param i_corr_cur indices of cur_descs of correspondences
   * @param i_corr_match indices of match_descs of correspondences
   */
  void getCorrespondencesDI(
      const std::vector<cv::Mat>& cur_descs,
      const DBoW2::FeatureVector& cur_fevec,
      const std::vector<cv::Mat>& match_descs,
      const DBoW2::FeatureVector& match_fevec,
      std::vector<unsigned int>& i_corr_cur,
      std::vector<unsigned int>& i_corr_match) const;

  /**
   * Calculate the matches between the descriptors A[i_A] and the descriptors
   * B[i_B].
   * @param A set A of descriptors
   * @param i_A only descriptors A[i_A] will be checked
   * @param B set B of descriptors
   * @param i_B only descriptors B[i_B] will be checked
   * @param i_match_A (out) indices of descriptors matched (s.t. A[i_match_A])
   * @param i_match_B (out) indices of descriptors matched (s.t. B[i_match_B])
   */
  void getMatches(const std::vector<cv::Mat> &A,
                  const std::vector<unsigned int>& i_A,
                  const std::vector<cv::Mat> &B,
                  const std::vector<unsigned int>& i_B,
                  std::vector<unsigned int>& i_match_A,
                  std::vector<unsigned int>& i_match_B) const;

  /**
   * @brief Check for consistency by computing a 3D transformation with RANSAC
   * @param cur_frameid current frame
   * @param cur_keys current keypoints
   * @param cur_lms landmarks of current keypoints
   * @param i_cur indices of putatives correspondences of current keypoints
   * @param match_frameid matched frame
   * @param match_keys keypoints of matched frame
   * @param match_lms landmarks of matched frame
   * @param i_match indices of putative correspondences of matched frame keys
   * @param data
   * @return true iif T can be computed
   */
  bool checkRigidTransformation(
      const ReferenceFrameId& cur_frameid,
      const std::vector<cv::KeyPoint>& cur_keys,
      const std::vector<LandmarkId>& cur_lms,
      const std::vector<unsigned int>& i_cur,
      const ReferenceFrameId& match_frameid,
      const std::vector<cv::KeyPoint>& match_keys,
      const std::vector<LandmarkId>& match_lms,
      const std::vector<unsigned int>& i_match,
      GeometricData& data) const;

  /**
   * @brief Check for robust epipolar consistency with RANSAC
   * @param cur_keys current keypoints
   * @param i_cur indices of putative correpsondences
   * @param match_keys matched keypoints
   * @param i_match indices of putative correspondences
   * @param data
   * @return true iif check is satisfied
   */
  bool checkEpipolar(const std::vector<cv::KeyPoint>& cur_keys,
                     const std::vector<unsigned int>& i_cur,
                     const std::vector<cv::KeyPoint>& match_keys,
                     const std::vector<unsigned int>& i_match,
                     GeometricData& data) const;

  /**
   * @brief Normalizes the brightness of the image
   * @param im (CV_8U)
   * @return normalized image (CV_8U)
   */
  cv::Mat normalizeImage(const cv::Mat &im) const;

  /**
   * @brief Saves data into a filestorage
   * @param fs
   * @param name entry name
   */
  void save(cv::FileStorage& fs, const std::string& name) const;

  /**
   * @brief Loads data from a filestorage
   * @param fs
   * @param name entry name
   */
  void load(cv::FileStorage& fs, const std::string& name);

private:

  /// Descriptor extractor
  cv::Ptr<cv::DescriptorExtractor> m_descriptor_extractor;

  /// Image database
  std::unique_ptr<DBoW2::TemplatedDatabase<cv::Mat, FMat8UBinary>> m_database;

  /// Map from database to frame nodes and its features
  std::map<DBoW2::EntryId, Node> m_db_to_node;

  /// Set with all the frames stored in nodes
  std::set<ReferenceFrameId> m_existing_frames;

  /// Loop detector parameters
  Parameters m_params;

  /// Active sessions
  std::map<SessionId, std::shared_ptr<Session> > m_sessions;

  /// Mutexes
  mutable std::mutex m_mutex_params;
  mutable std::mutex m_mutex_database;
  mutable std::mutex m_mutex_descriptor;
  mutable std::mutex m_mutex_sessions;

#if DEBUG_SAVE_IMAGES
  std::map<ReferenceFrameId, cv::Mat> m_images;
#endif

};

#endif // MULTIDBOWMATCHER_H
