// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

// one flag in each camera (left and right)
enum MatchFlag {
  /** UNDEFINED state -- this should never happen */
  UnInitialized = 0,

  /** Measurement is the first measurement of a new landmark */
  NewFeature,

  /** Measurement is a good track of the landmark */
  GoodMatch,

  /** Measurement was not tracked but then rethreaded */
  Rethreaded,

  /** Measurement did not match it's landmark in temporal matching */
  NoMatchInRegion,

  /** Measurement did not find corresponding scanline match (multi-view only) */
  NoMatchOnLine,

  /** Measurement failed to match in time because of sub-pixel refinement */
  LowSubPixInRegion,

  /** Measurement failed to match Left-Right because of sub-pixel refinement */
  LowSubPixOnLine,

  /** Measurement failed to match because no FAST features to compare with */
  NoFeaturesToMatch,

  /** Measurement was tagged as an outlier by ransac */
  RansacOutlier,

  /** Measurement was tagged as an outlier after GaussNewton */
  GaussNewtonOutlier,

  /** Measurement is outside of the field of view */
  OutsideFOV,

  /** Measurement is missing */
  MissingFeature,

  /** Measurement matches well to more than one correspondence */
  AmbiguousMatch,

  /** Measurement reverse matching verification failed */
  ReverseMatchFail,

  /** Measurement match score too high */
  BadScoreAfterESM,

  /** mono measurement that's still at infinity due to not enough tracks */
  LandmarkAtInfinity,

  /** Not a real flag. Used to indicate last flag. */
  LastMatchFlag,
};

static const unsigned int g_nNumMatchFlags = LastMatchFlag;

inline const char* MatchStr(MatchFlag e) {
  switch(e){
    case UnInitialized     : return "UnInitialized";
    case NewFeature        : return "NewFeature";
    case GoodMatch         : return "GoodMatch";
    case Rethreaded        : return "Rethreaded";
    case NoMatchInRegion   : return "NoMatchInROI";
    case NoMatchOnLine     : return "NoMatchOnLine";
    case LowSubPixInRegion : return "LowSubPixInROI";
    case LowSubPixOnLine   : return "LowSubPixOnLine";
    case NoFeaturesToMatch : return "NoFeatToMatch";
    case RansacOutlier     : return "RansacOutlier";
    case GaussNewtonOutlier: return "GNOutlier";
    case OutsideFOV        : return "OutsideFOV";
    case MissingFeature    : return "MissingFeature";
    case AmbiguousMatch    : return "AmbiguousMatch";
    case ReverseMatchFail  : return "ReverseMatchFail";
    case BadScoreAfterESM  : return "BadScoreESM";
    case LandmarkAtInfinity: return "Lmk@Infinity";
    case LastMatchFlag     : return "ERROR-LastFlag";
  }
  return "ERROR";
}
