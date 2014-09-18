#include <PlaceMatching/TemplateMatcher/TemplateMatcher.h>
#include <SlamMap/ProtobufIO.h>

void TemplateMatcher::AddPlace(const ReferenceFrameId& frame_id,
                               const cv::Mat &new_place) {
  cv::Mat template_img = ToTemplate(new_place);
  corpus_.insert({frame_id, template_img});
}

void TemplateMatcher::GetPotentialPlaceMatches(
    const ReferenceFrameId& frame_id,
    const cv::Mat &query,
    std::vector<PlaceMatchCandidate> &place_matches) {
  cv::Mat template_img = ToTemplate(query);
  for(const auto& it : corpus_) {
    cv::Mat diffMat = abs(template_img - it.second);
    cv::Scalar result = cv::sum(diffMat);

    if(result[0] < match_threshold_) {
      place_matches.emplace_back(it.first,result[0]);
    }
  }

  if (place_matches.size() > 10) {
    std::partial_sort(place_matches.begin(),
                      place_matches.begin() + 10,
                      place_matches.end());
    place_matches.resize(10);
  } else {
    std::sort(place_matches.begin(), place_matches.end());
  }
}

void TemplateMatcher::Save(const std::string& filename) const {
  std::ofstream fout(filename);

  pb::TemplateCorpusMsg corpus;
  for (const auto& id_img : corpus_) {
    pb::TemplateMsg * template_msg = corpus.add_place();
    pb::ReadCvMat(id_img.second, template_msg->mutable_image());
  }

  corpus.SerializeToOstream(&fout);

  saveFrameIndices(fout);
}

void TemplateMatcher::Load(const std::string& filename) {
  std::ifstream fin(filename);

  pb::TemplateCorpusMsg corpus;
  corpus.ParseFromIstream(&fin);

  {
    ReferenceFrameId frame_id;
    for (auto& place : *corpus.mutable_place()) {
      cv::Mat temp = WriteCvMat(place.image()).clone();

      pb::parse_message(place.id(), &frame_id);
      corpus_.insert({frame_id, temp});
    }
  }
  loadFrameIndices(fin);
}

bool TemplateMatcher::GetPlace(const ReferenceFrameId& place_id,
                               cv::Mat* out) const {
  auto it = corpus_.find(place_id);
  if (it == corpus_.end()) return false;
  it->second.copyTo(*out);
  return true;
}

/** Add an already scaled template to the corpus */
void TemplateMatcher::AddTemplate(const ReferenceFrameId& frame_id,
                                  const cv::Mat &thumb) {
  if (thumb.rows != (int)ty_res_ ||
      thumb.cols != (int)tx_res_ ||
      thumb.type() != CV_8U) {
    std::cerr << "Attempting to add incorrect template: "
              << thumb.rows << " - " << thumb.cols << " - " << thumb.type()
              << std::endl;
    abort();
  }
  corpus_.insert({frame_id, thumb});
}

cv::Mat TemplateMatcher::ToTemplate(const cv::Mat& image) {
  cv::Mat thumb(ty_res_, tx_res_, image.type());
  cv::resize(image, thumb, thumb.size(), 0, 0, cv::INTER_AREA);
  thumb.convertTo(thumb, CV_8U);
  return thumb;
}
