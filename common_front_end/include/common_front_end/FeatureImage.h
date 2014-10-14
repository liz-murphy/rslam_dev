// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <vector>
#include <utility>
#include <memory>

#include <slam_map/PatchHomography.h>
#include <common_front_end/QuadTree.h>
#include <common_front_end/Feature.h>
#include <common_front_end/FeatureHandler.h>
#include <common_front_end/CommonFrontEndParamsConfig.h>
#include <utils/PrintMessage.h>
#include <utils/MathTypes.h>
#include <utils/ESM.h>
#include <utils/PatchUtils.h>
#include <opencv2/highgui/highgui.hpp>
#include <pb_msgs/ImagePyramid.h>

typedef std::shared_ptr<Feature> FeaturePtr;

class FeatureImage {
 public:
  FeatureImage(unsigned int num_pyramid_levels,
               double       level_scaling_factor,
               unsigned int num_quadtree_levels,
               unsigned int num_features_to_track,
               unsigned int max_features_in_cell,
               bool         use_buckets = true
               );

  /// Operate while maintaining a reference to the source pb::Image.
  /// Does not clone the underlying Mat
  void SetImage(const std::shared_ptr<pb::Image>& img);
  void Clear();

  /// copy one feature image into another
  void Copy(const FeatureImage& fromImage);

  ////////////////////////////////////////////////////////////////////////////
  // QUERY FUNCTIONS
  ////////////////////////////////////////////////////////////////////////////
  const unsigned char* ImageData();
  cv::Mat& ImageRef();
  int Width() const;
  int Height() const;
  QuadTree& GetQuadTreeRef();
  QuadTree* GetQuadTreePtr();

  std::vector<FeaturePtr>& RowVectorRef(const int nRow);
  const std::vector<FeaturePtr>& RowVectorRef(const int nRow) const;

  Feature* GetNearbyFeature(const float x, const float y);
  Feature* GetBestFeatureInCell(const unsigned int cell);
  Feature* GetBestFeature(int *pROI = nullptr);

  const std::vector< cv::KeyPoint > GetKeypoints();
  const cv::Mat& GetImageAtLevel(unsigned int uLevel) const;

  float GetFactorAtLevel(unsigned int uLevel) const {
    return factor_at_level_[uLevel];
  }

  float GetLevelFactor() const {
    return level_scale_factor_;
  }

  unsigned int GetNumLevels() const {
    return  num_pyramid_levels_;
  }

  const cv::Mat& GetImageAtScale(double dScale) const;
  bool ExtractFeatures(FeatureHandler& rFeatureHandler);
  void ResetFeatures();

  /// @param [in] H Warping homography in the level 0 image
  template<unsigned int PatchSize>
  bool LoadPatch(const PatchHomography<PatchSize> &H,
                 unsigned char* pPatch) const {
    PatchHomography<PatchSize> SamplingH;
    unsigned int uSamplingLevel;
    H.GetSamplingHomographyAndLevel(
        level_scale_factor_, image_pyramid_->NumLevels(),
        SamplingH, uSamplingLevel);
    return LoadPatchAtLevel<PatchSize>(SamplingH, uSamplingLevel, pPatch);
  }

  /// H: Warping homography in sampling level
  template<unsigned int PatchSize>
  bool LoadPatchAtLevel(const PatchHomography<PatchSize> &H,
                        const unsigned int uLevel,
                        unsigned char* pPatch) const {
    if (!image_pyramid_->Initialized()) return false;

    const cv::Mat& im = image_pyramid_->at(uLevel);
    return LoadWarpedPatch<PatchSize>(
        im.data, im.cols, im.rows, H.matrix(), pPatch);
  }

  /// This thing is hideous.  TODO make H a homography class.  This load
  /// is here in FeatureImage because this is where the image pyramid lives.
  /// The hope is that this one location hides the uglieness.
  template<unsigned int PatchSize>
  double RefineSubPixelXY(const unsigned char* pPatch,
                          const PatchHomography<PatchSize> &H,
                          float& fU,
                          float& fV) const {
    /// The following lines compute SamplingH, the homography to load
    /// from the right octave.
    PatchHomography<PatchSize> SamplingH;
    unsigned int uSamplingLevel;
    H.GetSamplingHomographyAndLevel(
        level_scale_factor_, image_pyramid_->NumLevels(),
        SamplingH, uSamplingLevel);
    const cv::Mat& rIm = image_pyramid_->at(uSamplingLevel);

    CTrackTrackingStats trackingStats;
    TrackingResults     trackingResults;

    if (esm_use_search_roi_) {
      //======================================================================
      // Extract a warped ROI from the search image (using the homography)
      //======================================================================
      const int uMargin = 3;
      const unsigned int uSearchROISize = PatchSize + 2*uMargin;
      std::vector<unsigned char> vSearchROI(uSearchROISize*uSearchROISize);
      // compute homography
      Eigen::Vector3t tl =
          SamplingH.matrix() * Eigen::Vector3t(-uMargin, -uMargin, 1.0);
      Eigen::Vector3t tr =
          SamplingH.matrix() * Eigen::Vector3t(PatchSize + uMargin - 1,
                                               -uMargin, 1.0);
      Eigen::Vector2t c  = SamplingH.CenterPixel();
      tl.template head<2>() /= tl[2];
      tr.template head<2>() /= tr[2];
      PatchHomography<uSearchROISize> searchHomography(
          tl.template head<2>(), tr.template head<2>(), c);

      LoadWarpedPatch<uSearchROISize>(rIm.data, rIm.cols, rIm.rows,
                                      searchHomography.matrix(),
                                      &vSearchROI[0]);

      // now set initial top-left position of search patch
      static TrackingSettings trackingSettings(uMargin, uMargin,
                                               PatchSize, PatchSize, PatchSize);

      trackingSettings.MaxNumIterations(20);
      const ImageHolder refPatchHolder(pPatch, PatchSize, PatchSize, PatchSize);
      const ImageHolder curImageHolder(&vSearchROI[0], uSearchROISize,
                                       uSearchROISize, uSearchROISize);

      TrackPlaneHomography(
          trackingSettings,
          refPatchHolder,
          curImageHolder,
          2,
          &trackingStats,
          &trackingResults);

      // homography position is a Delta from the top-left of the
      // patch. Compute the center.
      Eigen::Vector3t warpedCenter;
      warpedCenter[0] =
          uMargin + trackingResults.GetHomography().Get(0, 2) + (PatchSize-1)/2;
      warpedCenter[1] =
          uMargin + trackingResults.GetHomography().Get(1, 2) + (PatchSize-1)/2;
      warpedCenter[2] = 1.0;

      // apply homography to get correct center position
      Eigen::Vector3t centerPixel = searchHomography.matrix() * warpedCenter;

      // return to base level

      fU = (centerPixel[0]/centerPixel[2] + 0.5) /
          factor_at_level_[uSamplingLevel] - 0.5;
      fV = (centerPixel[1]/centerPixel[2] + 0.5) /
          factor_at_level_[uSamplingLevel] - 0.5;

    } else {
      // Homography top left (NB right now the first two args are not
      // used -- TODO clean this up)
      static TrackingSettings trackingSettings(0, 0, PatchSize,
                                               PatchSize, PatchSize);

      // TODO template CTrackHomography on Scalar type
      trackingResults.SetHomography(
          CTrackHomography(SamplingH.matrix().template cast<double>()));
      trackingSettings.MaxNumIterations(20);

      const ImageHolder refPatchHolder(pPatch, PatchSize, PatchSize, PatchSize);
      const ImageHolder curImageHolder(rIm.data, rIm.cols, rIm.rows, rIm.cols);

      RSLAM_TrackPlaneHomography<Scalar, PatchSize>(
          trackingSettings,
          refPatchHolder,
          curImageHolder,
          2,
          &trackingStats,
          &trackingResults);

      const Eigen::Vector3t centerPixel =
          trackingResults.GetHomography().m_mH.cast<Scalar>() *
          Eigen::Vector3t((PatchSize-1)/2.0, (PatchSize-1)/2.0, 1.0);

      // return to base level
      fU = (centerPixel[0] / centerPixel[2] + 0.5) /
          factor_at_level_[uSamplingLevel] - 0.5;
      fV = (centerPixel[1] / centerPixel[2] + 0.5) /
          factor_at_level_[uSamplingLevel] - 0.5;
    }

    return trackingStats.RMS();
  }

  /// for OVV
  bool GetDescriptors(std::vector<unsigned char *> &descriptors,
                      unsigned int &descsize);

  double                                     harris_score_threshold_;
 private:
  /// Assumes the top level of the pyramid (level 0) is set, and
  /// builds out the rest of the pyramid
  void _BuildPyramid();
  void _AddKeyPoints(std::vector<cv::KeyPoint> &keypoints);

  void _BucketKeyPoints(std::vector<cv::KeyPoint>& keypoints);

 private:
  bool                                       use_buckets_;
  float                                      level_scale_factor_;
  float                                      log_level_scale_factor_;
  unsigned int                               num_pyramid_levels_;
  unsigned int                               num_quadtree_levels_;
  unsigned int                               num_features_to_track_;
  size_t                                     max_features_in_cell_;
  QuadTree                                   quad_tree_;
  std::vector< std::vector< FeaturePtr > >   features_;          // CRS
  std::vector< cv::KeyPoint >                keypoints_;

  // we keep an image pyramid (for some types of feature detectors)
  std::shared_ptr<pb::ImagePyramid>          image_pyramid_;
  std::vector<float>                         factor_at_level_;
  bool                                       esm_use_search_roi_;
};

typedef std::shared_ptr<FeatureImage>                FeatureImagePtr;
typedef std::vector<std::shared_ptr<FeatureImage> >  FeatureImageVector;
