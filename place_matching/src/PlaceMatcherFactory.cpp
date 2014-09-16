// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <place_matching/place_matcher_factory.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>
#include <place_matching/DBoWMatcher/dbow_config.h>

PlaceMatcherFactory::Options::Options() :
    vocab_filename(DBOW_ORB_VOC_FILE) {
}

std::shared_ptr<PlaceMatcher>
PlaceMatcherFactory::Create(const Options& options) {
  auto params = options.parameters;
  auto multi_params = options.multi_parameters;
  switch (options.type) {
    case kTemplateMatcherType:
      return std::make_shared<TemplateMatcher>();
    case kOVVMatcherType:
#ifdef BUILD_OVV
      return std::make_shared<OVVPlaceMatcher>();
#endif
    case kDBoWMatcherType:
      return std::make_shared<DBoWMatcher>(options.vocab_filename,
                                           params,
                                           options.detector,
                                           options.descriptor);
    case kMultiDBoWMatcherType:
      return std::make_shared<MultiDBoWMatcher>(options.map,
                                                options.vocab_filename,
                                                multi_params,
                                                options.descriptor);
    default:
      LOG(FATAL) << "Unknown MatcherType passed to PlaceMatcherFactory";
  }
  return std::shared_ptr<PlaceMatcher>();
}
