#ifndef DBOWMATCHER_H
#define DBOWMATCHER_H

#include <opencv2/opencv.hpp>
#include <place_matching/DBoWMatcher/DLoopDetector/DLoopDetector.h>
#include <place_matching/PlaceMatcher.h>
#include <place_matching/DBoWMatcher/FMat8UBinary.h>

/// Loop detector based on DBoW2
/// Galvez-Lopez, D. et al, T-RO 2012
class DBoWMatcher : public PlaceMatcher
{
public:

  /// Loop detector paramters
  typedef DLoopDetector::Parameters Parameters;

  /// Export/Import data
  struct InternalData
  {
    const std::vector<std::vector<cv::KeyPoint> > * image_keys;
    const std::vector<std::vector<cv::Mat> > * image_descriptors;
    std::vector<unsigned int> frame_indices;
    const Parameters * params;

    InternalData(): image_keys(nullptr), image_descriptors(nullptr),
      params(nullptr) {}
  };

public:
    /**
     * @brief DBoWMatcher creates a loop detector with a given vocabulary
     *    and calls setVocabulary
     * @see setVocabulary
     */
    DBoWMatcher(const std::string &voc_file,
                const Parameters& params = Parameters(1),
                std::string detector = "",
                std::string descriptor = "");

    /**
     * @brief DBoWMatcher Creates an empty matcher. A vocabulary is required
     * before using it
     */
    DBoWMatcher();
    ~DBoWMatcher();

    /**
     * @brief setVocabulary Sets the vocabulary of the matcher
     * @param voc_file vocabulary file of any feature
     * @param params parameters of the loop detector.
     * @param detector name of the keypoint detector to use as defined in OpenCV
     *    (e.g. "FAST", "ORB"). This detector can be different from the one
     *    used to create the vocabulary.
     * @param descriptor name of the descriptor to use as defined in
     *    OpenCV (e.g. "ORB", "FREAK"). This descriptor must be the same as
     *    the one used to create the vocabulary.
     * @note If detector or descriptor are not given, they are tried to
     *    be set according to the vocabulary filename if it formated like this:
     *    DETECTOR_DESCRIPTOR_..., e.g.: FAST_BRIEF_k10L6.voc.gz, or like this:
     *    FEATURE_..., e.g.: ORB_k10L6.voc.gz. In the latter case, FEATURE
     *    must be both a detector and a descriptor name.
     *    If it is not possible and one of them is given, the other is set
     *    the same. If still not possible, an exception is thrown.
     *    If the name of the descriptor is given and it does not match the
     *    vocabulary file name, a warning will be shown with LOG.
     */
    void setVocabulary(const std::string &voc_file,
                       const Parameters& params = Parameters(1),
                       std::string detector = "",
                       std::string descriptor = "");


    /**
     * @brief Sets an option of the keypoint detector
     * @param name option name
     * @param value value
     */
    template<class T>
    void setFeatureOption(const std::string& name, const T& value);

    /**
     * @brief Gets an option of the keypoint detector
     * @param name option name
     * @return value
     */
    template<class T>
    T getFeatureOption(const std::string& name) const;

    /**
     * @brief size Returns the number of places added into the database
     * @return number of places
     */
    size_t size() const;

    size_t NumPlaces() const override {
        return size();
    }

    // Versions with ReferenceFrameId
    using PlaceMatcher::AddPlace;
    using PlaceMatcher::GetPotentialPlaceMatches;
    using PlaceMatcher::GetPotentialPlaceMatchesOrAddPlace;

    void AddPlace(const ReferenceFrameId& frameid,
                  const std::vector<cv::KeyPoint>& keys,
                  const std::vector<cv::Mat>& descs);

    inline void getFeatures(const ReferenceFrameId& id,
                            const cv::Mat& image,
                            std::vector<cv::KeyPoint> &keys,
                            std::vector<cv::Mat>& descs) const override
    {
      getFeatures(image, keys, descs);
    }

    /**
     * @brief GetPlace Retrieves the keys and descriptors of a given place.
     * If the id does not exist, empty vectors are returned.
     * @param uFrameId frame id
     * @param keys place keys
     * @param descs place descriptors. cv::Mat's are not cloned.
     */
    void GetPlace(const unsigned int uFrameId,
                  std::vector<cv::KeyPoint> &keys,
                  std::vector<cv::Mat> &descs) const;

    void Save(const std::string& filename) const override;
    void Load(const std::string& filename) override;

private:

    /**
     * @brief AddPlace
     * @param uFrameId new place frame id
     * @param newPlace new place
     */
    virtual void AddPlace(const unsigned int uFrameId,
            const cv::Mat &newPlace) override;

    /**
     * @brief GetPotentialPlaceMatches Attempt to match against entire history
     * of places
     * @param uFrameId current place frame id
     * @param queryPlace current place
     * @param vPlaceMatches results
     */
    virtual void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) override;

    /**
     * @brief GetPotentialPlaceMatches
     * @param uFrameId current place frame id
     * @param queryPlace current place
     * @param vSubset accepted frame ids
     * @param vPlaceMatches results
     */
    void GetPotentialPlaceMatches(
      const cv::Mat &queryPlace,
      const std::vector<unsigned int> &vSubset,
      std::vector<PlaceMatchCandidate> &vPlaceMatches) override;

    /**
     * @brief GetPotentialPlaceMatches
     * @param uFrameId current place frame id
     * @param queryKeys current place keypoints
     * @param queryDescs current place descriptors
     * @param width original image width
     * @param height original image height
     * @param vSubset accepted frame ids
     * @param vPlaceMatches results
     */
    void GetPotentialPlaceMatches(
        const std::vector<cv::KeyPoint> &queryKeys,
        const std::vector<cv::Mat> &queryDescs,
        int width, int height,
        const std::vector<unsigned int> &vSubset,
        std::vector<PlaceMatchCandidate> &vPlaceMatches);

    /**
     * @brief GetPotentialPlaceMatches Attempt to match against entire history
     * of places
     * @param uFrameId current place frame id
     * @param queryKeys current place keypoints
     * @param queryDescs current place descriptors
     * @param width original image width
     * @param height original image height
     * @param vPlaceMatches results
     */
    void GetPotentialPlaceMatches(
      const std::vector<cv::KeyPoint> &queryKeys,
      const std::vector<cv::Mat> &queryDescs,
      int width, int height,
      std::vector<PlaceMatchCandidate> &vPlaceMatches);

    /**
     * @brief AddPlace
     * @param uFrameId new place frame id
     * @param newKeys current place keypoints
     * @param enwDescs current place descriptors
     */
    void AddPlace(const unsigned int uFrameId,
                  const std::vector<cv::KeyPoint> &newKeys,
                  const std::vector<cv::Mat> &newDescs);

    /**
     * @brief getFeatures Gets features from an image in DBoW2 format
     * @param im image
     * @param keys[out] keypoints got
     * @param descs[out] descriptors got
     */
    void getFeatures(const cv::Mat &im, std::vector<cv::KeyPoint>& keys,
                     std::vector<cv::Mat>& descs) const;

    /**
     * @brief GetPotentialMatchesOrAddPlace This runs loop detection and, if
     * no loop is found, the given place is added to the database
     * @param place place image
     * @param vPlaceMatches
     * @param uFrameId current place id. If not given, it is set to the
     * current size of the database.
     * @param vSubset if given, ids of places to match
     */
    virtual void GetPotentialMatchesOrAddPlace(
        const cv::Mat &place,
        std::vector<PlaceMatchCandidate>& vPlaceMatches,
        const unsigned int uFrameId = std::numeric_limits<unsigned int>::max(),
        const std::vector<unsigned int> &vSubset = std::vector<unsigned int>());

    /**
     * @brief GetPotentialMatchesOrAddPlace This runs loop detection and, if
     * no loop is found, the given place is added to the database
     * @param keys place keypoints
     * @param descs place descriptors
     * @param width original image width
     * @param height original image height
     * @param vPlaceMatches
     * @param uFrameId current place id. If not given, it is set to the
     * current size of the database.
     * @param vSubset if given, ids of places to match
     */
    void GetPotentialMatchesOrAddPlace(
        const std::vector<cv::KeyPoint> &keys,
        const std::vector<cv::Mat> &descs,
        int width, int height,
        std::vector<PlaceMatchCandidate>& vPlaceMatches,
        const unsigned int uFrameId = std::numeric_limits<unsigned int>::max(),
        const std::vector<unsigned int> &vSubset = std::vector<unsigned int>());

    /**
     * @brief exportData Exposes internal data to external methods
     * @returns Returns pointers to the internal data
     */
    const InternalData exportData() const;

    /**
     * @brief importData Adds data to the internal database
     * @param data data to copy (the cv::Mat's are copied, but not cloned).
     * If some field is NULL, it is ignored
     */
    void importData(const InternalData& data);

protected:

    typedef DLoopDetector::TemplatedLoopDetector<cv::Mat, FMat8UBinary>
      BinaryLoopDetector;

    /// Loop detector
    std::unique_ptr<BinaryLoopDetector> m_loop_detector;

    /// Feature detector
    cv::Ptr<cv::FeatureDetector> m_detector;

    /// Descriptor extractor
    cv::Ptr<cv::DescriptorExtractor> m_descriptor;

    /// Map from internal entry ids to external frame ids
    std::map<DBoW2::EntryId, unsigned int> m_entry_to_frame;

    /// Map from external frame ids to internal entry ids
    std::map<unsigned int, DBoW2::EntryId> m_frame_to_entry;
};

template<class T>
void DBoWMatcher::setFeatureOption(const std::string& name, const T& value)
{
  if(m_detector) m_detector->set(name, value);
}

template<class T>
T DBoWMatcher::getFeatureOption(const std::string& name) const
{
  if(m_detector)
    return m_detector->get<T>(name);
  else
    return T(0);
}

#endif // DBOWMATCHER_H
