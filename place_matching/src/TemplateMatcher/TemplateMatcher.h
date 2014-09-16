#ifndef TEMPLATE_MATCHER_H
#define TEMPLATE_MATCHER_H

#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <PbMsgs/Image.h>
#include <PlaceMatching/PlaceMatchCandidate.h>
#include <PlaceMatching/PlaceMatcher.h>
#include <PlaceMatching/template.pb.h>

class TemplateMatcher : public PlaceMatcher {
 public:
  TemplateMatcher() : tx_res_(32), ty_res_(24), match_threshold_(5000) {}

  TemplateMatcher(unsigned int txRes, unsigned int tyRes) :
      tx_res_(txRes), ty_res_(tyRes), match_threshold_(5000) {}

  using PlaceMatcher::AddPlace;

  void AddPlace(const ReferenceFrameId& frame_id,
                const cv::Mat &new_place) override;

  virtual void GetPotentialPlaceMatches(const ReferenceFrameId& id,
                                        const cv::Mat& place,
                                        std::vector<PlaceMatchCandidate>& vPlaceMatches);

  void Save(const std::string& filename) const;
  void Load(const std::string& filename);
  bool GetPlace(const ReferenceFrameId& place_id, cv::Mat* out) const;

  void AddTemplate(const ReferenceFrameId& frameId, const cv::Mat &thumb);
  cv::Mat ToTemplate(const cv::Mat& image);

  size_t NumPlaces() const override {
    return corpus_.size();
  }

 private:
  std::map<ReferenceFrameId, cv::Mat> corpus_;
  unsigned int tx_res_, ty_res_;
  double match_threshold_;
};

#endif
