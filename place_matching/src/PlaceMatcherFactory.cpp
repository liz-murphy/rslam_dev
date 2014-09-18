// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#include <place_matching/PlaceMatcherFactory.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>


std::shared_ptr<PlaceMatcher>
PlaceMatcherFactory::Create(const Options& options) {
  switch (options.type) {
    case kTemplateMatcherType:
      return std::make_shared<TemplateMatcher>();
    default:
      std::cerr << "Unknown MatcherType passed to PlaceMatcherFactory";
  }
  return std::shared_ptr<PlaceMatcher>();
}
