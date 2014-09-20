
#include <sys/stat.h>
#include <fcntl.h>
#include <vector>
#include <stdexcept>

#include <opencv2/opencv.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <miniglog/logging.h>
#include <slam_map/ProtobufIO.h>

#include <place_matching/DBoWMatcher/dbow_protobuf.h>
#include <place_matching/DBoWMatcher/DBoW2/DBoW2.h>
#include <place_matching/DBoWMatcher/DLoopDetector/DLoopDetector.h>

#include <place_matching/DBoWMatcher/FMat8UBinary.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>

DBoWMatcher::DBoWMatcher()
{
}

DBoWMatcher::~DBoWMatcher()
{
}

DBoWMatcher::DBoWMatcher(const std::string &voc_file,
                         const Parameters &params,
                         std::string detector,
                         std::string descriptor)
{
  setVocabulary(voc_file, params, detector, descriptor);
}

void DBoWMatcher::setVocabulary(const std::string &voc_file,
                                const Parameters &params,
                                std::string detector,
                                std::string descriptor)
{
  if(voc_file.empty())
    throw std::invalid_argument("No vocabulary given");

  if(!DUtils::FileFunctions::FileExists(voc_file.c_str()))
    throw std::invalid_argument("Could not find vocabulary file");

  // check if voc file name is formatted as DETECTOR_DESCRIPTOR_...
  std::string voc_path, voc_name, voc_ext;
  DUtils::FileFunctions::FileParts(voc_file, voc_path, voc_name, voc_ext);

  std::string guess_detector, guess_descriptor;
  {
    std::vector<std::string> tokens;
    DUtils::StringFunctions::split(voc_name, tokens, "_");
    if(tokens.size() > 2)
    {
      guess_detector = tokens[0];
      guess_descriptor = tokens[1];
    }
    else if(tokens.size() == 2)
    {
      guess_detector = guess_descriptor = tokens[0];
    }
  }

  if(detector.empty()) detector = guess_detector;
  if(descriptor.empty()) descriptor = guess_descriptor;

  if(detector.empty() && !descriptor.empty()) detector = descriptor;
  else if(!detector.empty() && descriptor.empty()) descriptor = detector;

  // create feature extractors
  cv::Ptr<cv::FeatureDetector> fedet = cv::FeatureDetector::create(detector);
  cv::Ptr<cv::DescriptorExtractor> desex =
      cv::DescriptorExtractor::create(descriptor);

  if(fedet.empty())
    throw std::invalid_argument("Could not create a feature detector for " +
                                detector);
  else if(desex.empty())
    throw std::invalid_argument("Could not create a descriptor extractor for " +
                         descriptor);

  // store the detectors
  m_detector = fedet;
  m_descriptor = desex;

  if(descriptor != guess_descriptor)
  {
    // the user load a vocabulary that seems created for a descriptor but
    // used a different one, that's suspicious
    LOG(WARNING) << "Descriptor " << descriptor << " will be used with the "
                 "vocabulary " << voc_name << "." << voc_ext << ", which "
                 "seems created for descriptor " << guess_descriptor;
  }

  // if the descriptor binary?
  bool binary = descriptor == "ORB" || descriptor == "FREAK" ||
      descriptor == "BRISK" || descriptor == "BRIEF";

  if(!binary)
  {
    throw std::invalid_argument("Only binary descriptors are supported");
  }

  // load the vocabulary and the detector
  DBoW2::TemplatedVocabulary<cv::Mat, FMat8UBinary> voc(voc_file);
  m_loop_detector.reset(new DLoopDetector::TemplatedLoopDetector
                        <cv::Mat, FMat8UBinary>(voc, params));
}

void DBoWMatcher::getFeatures(const cv::Mat &im, std::vector<cv::KeyPoint>& keys,
                 std::vector<cv::Mat>& descs) const
{
  cv::Mat all_descs;
  m_detector->detect(im, keys);
  m_descriptor->compute(im, keys, all_descs);

  // convert NxL mat to a vector or N 1xL
  descs.resize(all_descs.rows);
  for(int i = 0; i < all_descs.rows; ++i)
    descs[i] = all_descs.row(i);
}

void DBoWMatcher::AddPlace(const ReferenceFrameId& frameId,
                           const std::vector<cv::KeyPoint> &keys,
                           const std::vector<cv::Mat> &descs)
{
  unsigned int uid = getNewInternalPlaceId(frameId);
  AddPlace(uid, keys, descs);
}

void DBoWMatcher::AddPlace(const unsigned int uFrameId, const cv::Mat &newPlace)
{
  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  getFeatures(newPlace, keys, descs);
  AddPlace(uFrameId, keys, descs);
}

void DBoWMatcher::AddPlace(const unsigned int uFrameId,
              const std::vector<cv::KeyPoint> &keys,
              const std::vector<cv::Mat> &descs)
{
  // add to database
  DBoW2::EntryId eid = m_loop_detector->addPlace(keys, descs);

  // update maps
  m_entry_to_frame[eid] = uFrameId;
  m_frame_to_entry[uFrameId] = eid;
}

void DBoWMatcher::GetPlace(const unsigned int uFrameId,
              std::vector<cv::KeyPoint> &keys,
              std::vector<cv::Mat> &descs) const
{
  keys.clear();
  descs.clear();

  if (!m_frame_to_entry.count(uFrameId)) return;

  unsigned int eid = m_frame_to_entry.at(uFrameId);
  if(m_loop_detector && eid < m_loop_detector->size())
  {
    auto mit = m_frame_to_entry.find(uFrameId);
    if(mit != m_frame_to_entry.end())
    {
      DBoW2::EntryId eid = mit->second;
      m_loop_detector->getPlace(eid, keys, descs);
    }
    // else: should not happen
  }
}

void DBoWMatcher::GetPotentialPlaceMatches(
  const cv::Mat &queryPlace,
  std::vector<PlaceMatchCandidate> &vPlaceMatches)
{
  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  getFeatures(queryPlace, keys, descs);
  GetPotentialPlaceMatches(keys, descs, queryPlace.cols,
                           queryPlace.rows, vPlaceMatches);
}

void DBoWMatcher::GetPotentialPlaceMatches(
  const std::vector<cv::KeyPoint> &keys,
  const std::vector<cv::Mat> &descs,
  int width, int height,
  std::vector<PlaceMatchCandidate> &vPlaceMatches)
{
  vPlaceMatches.clear();

  // make sure the loop detector is completely configured
  if(!m_loop_detector->imageSizeSet())
    m_loop_detector->setImageSize(width, height);

  DLoopDetector::DetectionResult match;
  if(m_loop_detector->detectLoop(keys, descs, match,
                                 DBoW2::EntryId(this->size()))) {
    unsigned int uFrameId = m_entry_to_frame[match.match];

    // only one image is returned, and without score
    vPlaceMatches.push_back(PlaceMatchCandidate(
                            uFrameId, 1.));
  }
}

void DBoWMatcher::GetPotentialMatchesOrAddPlace(
    const cv::Mat &place,
    std::vector<PlaceMatchCandidate>& vPlaceMatches,
    const unsigned int uFrameId,
    const std::vector<unsigned int> &vSubset)
{
  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  getFeatures(place, keys, descs);
  GetPotentialMatchesOrAddPlace(keys, descs, place.cols, place.rows,
                                vPlaceMatches, uFrameId, vSubset);
}

void DBoWMatcher::GetPotentialMatchesOrAddPlace(
    const std::vector<cv::KeyPoint> &keys,
    const std::vector<cv::Mat> &descs,
    int width, int height,
    std::vector<PlaceMatchCandidate>& vPlaceMatches,
    const unsigned int _uFrameId,
    const std::vector<unsigned int> &vSubset)
{
  vPlaceMatches.clear();

  if(!m_loop_detector->imageSizeSet())
    m_loop_detector->setImageSize(width, height);

  unsigned int uFrameId;
  DBoW2::EntryId eid;

  if(_uFrameId == std::numeric_limits<unsigned int>::max())
  {
    uFrameId = this->size();
    eid = this->size();
  }
  else
  {
    uFrameId = _uFrameId;

    auto mit = m_frame_to_entry.find(_uFrameId);
    if(mit == m_frame_to_entry.end())
      eid = this->size();
    else
      eid = mit->second;
  }

  // detect something
  DLoopDetector::DetectionResult match;
  if(m_loop_detector->detectLoop(keys, descs, match, eid))
  {
    // only one image is returned, and without score
    unsigned int current_ufid = m_entry_to_frame[match.match];

    if(vSubset.empty() ||
       std::find(vSubset.begin(), vSubset.end(), current_ufid) != vSubset.end())
      vPlaceMatches.push_back(PlaceMatchCandidate(current_ufid, 1.));
  }

  if(vPlaceMatches.empty())
  {
    // add
    DBoW2::EntryId eid = m_loop_detector->addPlace(keys, descs);

    // update maps
    m_entry_to_frame[eid] = uFrameId;
    m_frame_to_entry[uFrameId] = eid;
  }
}

size_t DBoWMatcher::size() const
{
  if(m_loop_detector) return m_loop_detector->size();
  else return 0;
}



void DBoWMatcher::GetPotentialPlaceMatches(
        const cv::Mat &queryPlace,
        const std::vector<unsigned int> &vSubset,
        std::vector<PlaceMatchCandidate> &vPlaceMatches)
{
  std::vector<cv::KeyPoint> keys;
  std::vector<cv::Mat> descs;
  getFeatures(queryPlace, keys, descs);
  GetPotentialPlaceMatches(keys, descs, queryPlace.cols,
                           queryPlace.rows, vSubset, vPlaceMatches);
}

void DBoWMatcher::GetPotentialPlaceMatches(
    const std::vector<cv::KeyPoint> &queryKeys,
    const std::vector<cv::Mat> &queryDescs,
    int width, int height,
    const std::vector<unsigned int> &vSubset,
    std::vector<PlaceMatchCandidate> &vPlaceMatches)
{
  // Note: DLoopDetector does not provide a nice way of performing
  // this operation, so we just match all the places in the database
  // and then check if the detected one is in the given subset
  GetPotentialPlaceMatches(queryKeys, queryDescs, width,
                           height, vPlaceMatches);

  if(vPlaceMatches.size() == 1)
  {
    if(vSubset.end() ==
       std::find(vSubset.begin(), vSubset.end(), vPlaceMatches[0].getID()))
      vPlaceMatches.clear();
  }
  else if(!vPlaceMatches.empty())
  {
    // it may be worth it to sort the accepted indices to avoid linear searchs
    std::vector<unsigned int> sorted_subset = vSubset;
    std::sort(sorted_subset.begin(), sorted_subset.end());

    // i_remove contains the indices of items of vPlaceMatches to remove
    std::vector<unsigned int> i_remove;
    if(vPlaceMatches.size() > vSubset.size())
      i_remove.reserve(vPlaceMatches.size() - vSubset.size());

    for(size_t i = 0; i < vPlaceMatches.size(); ++i)
    {
      if(!std::binary_search(sorted_subset.begin(), sorted_subset.end(),
                             vPlaceMatches[i].getID()))
      {
        // if it is not in the subset, flag for removal
        i_remove.push_back(i);
      }
    }

    // real removal
    const bool preserve_order = true;
    DUtils::STL::removeIndices(vPlaceMatches, i_remove, preserve_order);
  }
}

const DBoWMatcher::InternalData DBoWMatcher::exportData() const
{
  DBoWMatcher::InternalData ret;

  ret.frame_indices.reserve(m_entry_to_frame.size());
  for(auto mit = m_entry_to_frame.begin(); mit != m_entry_to_frame.end(); ++mit)
  {
    ret.frame_indices.push_back(mit->second);
  }

  if(m_loop_detector)
  {
    BinaryLoopDetector::InternalData loop_data = m_loop_detector->exportData();
    ret.image_descriptors = loop_data.image_descriptors;
    ret.image_keys = loop_data.image_keys;
    ret.params = loop_data.params;
  }
  else
  {
    ret.image_descriptors = NULL;
    ret.image_keys = NULL;
  }

  return ret;
}

void DBoWMatcher::importData(const DBoWMatcher::InternalData& data)
{
  if(m_loop_detector)
  {
    if(data.image_descriptors != nullptr && data.image_keys != nullptr)
    {
      size_t eid = m_loop_detector->size();
      for(size_t i = 0; i < data.frame_indices.size(); ++i, ++eid)
      {
        const unsigned int uframeid = data.frame_indices[i];
        m_entry_to_frame[eid] = uframeid;
        m_frame_to_entry[uframeid] = eid;
      }
    }

    BinaryLoopDetector::InternalData loop_data;
    loop_data.image_descriptors = data.image_descriptors;
    loop_data.image_keys = data.image_keys;
    loop_data.params = data.params;

    m_loop_detector->importData(loop_data);
  }
}

void DBoWMatcher::Save(const std::string& filename) const {
  using namespace google::protobuf::io;
  if(!m_loop_detector) return;

  LOG(INFO) << "Saving DBoW places to " << filename;

  BinaryLoopDetector::InternalData loop_data = m_loop_detector->exportData();
  CHECK_NOTNULL(loop_data.image_keys);
  CHECK_NOTNULL(loop_data.image_descriptors);

  pb::DBoWPlaceMsg msg;

  int fd = open(filename.c_str(),
                O_WRONLY | O_CREAT | O_TRUNC,
                S_IRUSR | S_IWUSR | S_IRGRP);
  if (fd < 0) {
    LOG(ERROR) << "Could not save DBoW places to " << filename;
    return;
  }

  FileOutputStream* raw_output = new FileOutputStream(fd);
  CodedOutputStream* coded_output = new CodedOutputStream(raw_output);
  for (size_t i = 0; i < m_frames.size(); ++i) {
    msg.Clear();

    unsigned int entry = m_frame_to_entry.at(i);
    pb::fill_message(m_place_ids.at(i), msg.mutable_index());
    pb::fill_message(loop_data.image_keys->at(entry),
                     msg.mutable_keypoint_vector());
    pb::fill_message(loop_data.image_descriptors->at(entry),
                     msg.mutable_descriptor_vector());

    auto msg_size = msg.ByteSize();
    if (msg_size > 64 << 20) {
      LOG(ERROR) << "Message is too large, skipping.";
      continue;
    }
    coded_output->WriteVarint64(msg_size);
    msg.SerializeToCodedStream(coded_output);
  }

  delete coded_output;
  delete raw_output;
  int res = close(fd);
  CHECK_GE(res, 0);
  LOG(INFO) << "Finished saving " << m_entry_to_frame.size() << " DBoW places.";
}

void DBoWMatcher::Load(const std::string& filename) {
  if(!m_loop_detector) return;

  LOG(INFO) << "Loading DBoW places from " << filename;

  std::vector<std::vector<cv::KeyPoint> > image_keys;
  std::vector<std::vector<cv::Mat> > image_descriptors;

  pb::DBoWPlaceMsg place;

  using namespace google::protobuf::io;

  int fd = open(filename.c_str(), O_RDONLY);
  if (fd < 0) {
    LOG(ERROR) << "Could not load DBoW places from " << filename
               << "(" << strerror(fd) << ")";
    return;
  }
  FileInputStream* raw_input = new FileInputStream(fd);
  CodedInputStream* coded_input = new CodedInputStream(raw_input);

  int index = 0;
  uint64_t size = 0;

  // Up overall limit to be 256 MB, 64 MB for warning
  coded_input->SetTotalBytesLimit(64 << 22, 64 << 20);

  while (coded_input->ReadVarint64(&size)) {
    place.Clear();
    CodedInputStream::Limit msgLimit = coded_input->PushLimit(size);
    if (!place.ParseFromCodedStream(coded_input)) {
      break;
    } else if (!place.has_index() ||
               !place.has_keypoint_vector() ||
               !place.has_descriptor_vector()) {
      LOG(ERROR) << "Place is missing pieces";
      break;
    }

    coded_input->PopLimit(msgLimit);

    ReferenceFrameId frame_id;
    pb::parse_message(place.index(), &frame_id);
    m_frames.emplace(frame_id, m_frames.size());
    m_place_ids.emplace(m_frames.size() - 1, frame_id);
    m_entry_to_frame[index] = m_frames.size() - 1;
    m_frame_to_entry[m_frames.size() - 1] = index;
    ++index;

    image_keys.emplace_back();
    image_descriptors.emplace_back();

    if (!pb::parse_message(place.keypoint_vector(),
                           &image_keys.back()) ||
        !pb::parse_message(place.descriptor_vector(),
                           &image_descriptors.back())) {
      image_keys.pop_back();
      image_descriptors.pop_back();
    }
  }

  BinaryLoopDetector::InternalData loop_data;
  loop_data.image_descriptors = &image_descriptors;
  loop_data.image_keys = &image_keys;

  auto params = m_loop_detector->getParameters();
  loop_data.params = &params;

  m_loop_detector->importData(loop_data);

  delete coded_input;
  delete raw_input;
  int res = close(fd);
  CHECK_GE(res, 0);
  LOG(INFO) << "Finished loading " << index << " DBoW places.";
}
