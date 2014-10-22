// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.
#pragma once

#include <place_matching/PlaceMatcher.h>
#include <place_matching/TemplateMatcher/TemplateMatcher.h>
#include <place_matching/DBoWMatcher/DBoWMatcher.h>
#include <place_matching/MultiDBoWMatcher/MultiDBoWMatcher.h>
#include <memory>

/** Abstract factory for PlaceMatcher creation */
class PlaceMatcherFactory {
 public:
  enum MatcherType {
    kTemplateMatcherType,
    kOVVMatcherType,
    kDBoWMatcherType,
    kMultiDBoWMatcherType
  };

  struct Options {
    Options();
    MatcherType type = kDBoWMatcherType;

    // DBoW and MultiDBoW Options
    std::string vocab_filename;      // DBoW and MultiDBoW. Compile-time default
    std::string descriptor = "";     // DBoW and MultiDBoW
    std::string detector = "";       // DBoW only (MultiDBoW uses tracks)
    DBoWMatcher::Parameters parameters = DLoopDetector::Parameters(1); // DBoW
    MultiDBoWMatcher::Parameters multi_parameters; // MultiDBoW
    std::shared_ptr<SlamMap> map;    // MultiDBoW
  };

  static std::shared_ptr<PlaceMatcher> Create(const Options& options);

 private:
  PlaceMatcherFactory() {}
  virtual ~PlaceMatcherFactory() {}
};
