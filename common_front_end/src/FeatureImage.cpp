// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <common_front_end/FeatureImage.h>
#include <algorithm>
#include <utility>
#include <vector>

#include <common_front_end/CommonFrontEndParamsConfig.h>
//#include <common_front_end/FlybyFeatureDetector.h>
#include <miniglog/logging.h>
#include <pb_msgs/Image.h>
#include <pb_msgs/ImagePyramid.h>
#include <utils/ESM.h>

#include <ros/ros.h>

/*static double& g_harris_score_threshold =
    CVarUtils::CreateCVar<>("features.fast.HarrisScoreThrshold", 10000.0, "");
*/

// used for sorting
inline bool _CompareScoreFunc(const FeaturePtr& i, const FeaturePtr& j) {
  return i->response > j->response;
}

inline bool _CompareKeyPoints(const std::pair<int, float>& i,
                              const std::pair<int, float>& j) {
  return i.second > j.second;
}

FeatureImage::FeatureImage(unsigned int num_pyramid_levels,
                           double       level_scaling_factor,
                           unsigned int num_quadtree_levels,
                           unsigned int num_features_to_track,
                           unsigned int max_features_in_cell,
                           bool         use_buckets)
    : use_buckets_(use_buckets),
      level_scale_factor_(level_scaling_factor),
      log_level_scale_factor_(1.0/log(level_scale_factor_)),
      num_pyramid_levels_(num_pyramid_levels),
      num_quadtree_levels_(num_quadtree_levels),
      num_features_to_track_(num_features_to_track),
      max_features_in_cell_(max_features_in_cell),
      image_pyramid_(new pb::ImagePyramid(num_pyramid_levels,
                                          level_scaling_factor))
      {
        std::cout << "FeatureImage.cpp: " <<  num_pyramid_levels << "," << level_scaling_factor << ", " << num_quadtree_levels << ", " << num_features_to_track << ", " << max_features_in_cell << ", " << use_buckets << "\n";
  factor_at_level_.resize(num_pyramid_levels_);
  factor_at_level_[0] = 1.0;
  for (unsigned int ii = 1; ii < num_pyramid_levels_; ++ii) {
    factor_at_level_[ii] = factor_at_level_[ii-1]*level_scale_factor_;
  }
}

void FeatureImage::SetImage(const std::shared_ptr<pb::Image>& img) {
  image_pyramid_->Build(img);
}

/// copy one feature image into another
void FeatureImage::Copy(const FeatureImage& src_image) {
  // copy basic data
  num_pyramid_levels_ = src_image.num_pyramid_levels_;
  num_quadtree_levels_ = src_image.num_quadtree_levels_;
  max_features_in_cell_ = src_image.max_features_in_cell_;
  level_scale_factor_ = src_image.level_scale_factor_;
  log_level_scale_factor_ = src_image.log_level_scale_factor_;
  use_buckets_ = src_image.use_buckets_;
  quad_tree_ = src_image.quad_tree_;
  keypoints_ = src_image.keypoints_;
  factor_at_level_ = src_image.factor_at_level_;

  // copy image pyramid
  image_pyramid_.reset(new pb::ImagePyramid(*src_image.image_pyramid_));

  // copy features
  features_.clear();
  features_.resize(src_image.features_.size());
  for (size_t ii = 0 ; ii < src_image.features_.size() ; ii++) {
    features_[ii].resize(src_image.features_[ii].size());
    for (size_t jj = 0 ; jj < src_image.features_[ii].size() ; jj++) {
      features_[ii][jj] =
          FeaturePtr(new Feature(*src_image.features_[ii][jj]));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// QUERY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
const unsigned char* FeatureImage::ImageData() {
  return image_pyramid_->base()->data();
}

cv::Mat& FeatureImage::ImageRef() {
  return image_pyramid_->at(0);
}

int FeatureImage::Width() const {
  return image_pyramid_->at(0).cols;
}

int FeatureImage::Height() const {
  return image_pyramid_->at(0).rows;
}

QuadTree& FeatureImage::GetQuadTreeRef() {
  return quad_tree_;
}

QuadTree* FeatureImage::GetQuadTreePtr() {
  return &quad_tree_;
}

std::vector< FeaturePtr >& FeatureImage::RowVectorRef(const int nRow) {
  return features_[nRow];
}

const std::vector< FeaturePtr >&
FeatureImage::RowVectorRef(const int nRow) const {
  return features_[nRow];
}

/// Look for features close to this feature in the sparse feature image.
Feature* FeatureImage::GetNearbyFeature(const float x, const float y) {
  std::vector< FeaturePtr >& vRow = features_[round(y)];
  for (FeaturePtr pf : vRow) {
    if (pf->x > x+0.5) {
      return nullptr;
    }
    if (fabs(pf->x - x) < 0.5) {
      return pf.get();
    }
  }
  return nullptr;
}

const std::vector< cv::KeyPoint > FeatureImage::GetKeypoints() {
  return keypoints_;
}

const cv::Mat& FeatureImage::GetImageAtLevel(unsigned int uLevel) const {
  return image_pyramid_->at(uLevel);
}

const cv::Mat& FeatureImage::GetImageAtScale(double scale) const {
  unsigned int level = 0;
  if (scale > 1.0) {
    std::cerr << "WARNING. FeatureImage::GetImageAtScale. scale = "
              << scale << " > 1.0" << std::endl;
  } else if (scale < 1e-3) {
    level = image_pyramid_->NumLevels() - 1;
  } else {
    level = round(log(scale)*log_level_scale_factor_);
  }
  return image_pyramid_->at(level);
}

// Populate the CRS Matrix in the already created Pyramid
bool FeatureImage::ExtractFeatures(FeatureHandler& feature_handler) {
  features_.clear();
  keypoints_.clear();
  quad_tree_.Init(image_pyramid_->at(0).cols,
      image_pyramid_->at(0).rows,
      num_quadtree_levels_,
      num_features_to_track_);

  features_.resize(image_pyramid_->at(0).rows);
  feature_handler.DetectFeatures(image_pyramid_, keypoints_);

  if (use_buckets_) {
    _BucketKeyPoints(keypoints_);
  }

  // Compute descriptors
  cv::Mat descriptors;
  feature_handler.ComputeDescriptors(image_pyramid_, keypoints_, descriptors);

  size_t size = descriptors.cols * descriptors.elemSize();
  for (size_t ii = 0; ii < keypoints_.size(); ++ii) {
    cv::KeyPoint& kp = keypoints_[ii];

    std::vector<FeaturePtr>& row = features_[roundf(kp.pt.y)];
    row.emplace_back(new Feature(kp.class_id, kp.pt.x, kp.pt.y, kp.size,
                                 kp.angle, kp.response,
                                 (size > 0 ? descriptors.ptr(ii) : nullptr),
                                 size));
  }

  // sort features in each row according to their response value
  for (size_t ii = 0; ii < features_.size(); ++ii) {
    std::sort(features_[ii].begin(), features_[ii].end(), _CompareScoreFunc);
  }
  return true;
}

bool FeatureImage::GetDescriptors(std::vector<unsigned char *> &descriptors,
                                  unsigned int &descsize) {
  for (unsigned int i = 0; i < features_.size(); i++) {
    for (unsigned int j = 0; j < features_[i].size(); j++) {
      descriptors.push_back(&(features_[i][j])->descriptor[0]);
      if (features_[i][j]->descsize > 0) {
        descsize = features_[i][j]->descsize;
      }
    }
  }
  return true;
}

Feature* FeatureImage::GetBestFeatureInCell(const unsigned int cell) {
  float best_score = -FLT_MAX;
  Feature* best_feature_ptr = nullptr;
  CellBounds bounds = quad_tree_.GetCellBounds(cell);
  for (int uRow = bounds.t; uRow < bounds.b; ++uRow) {
    if (!features_[uRow].empty()) {
      // since each row is sorted only check first feature
      for (size_t ii = 0; ii < features_[uRow].size(); ++ii) {
        Feature* pF = features_[uRow][ii].get();
        if (pF->x >= bounds.l && pF->x < bounds.r) {
          if (best_score < pF->response && !pF->used) {
            best_score = pF->response;
            best_feature_ptr = pF;
          }
        }
      }
    }
  }
  return best_feature_ptr;
}

Feature* FeatureImage::GetBestFeature(int* pROI) {
  float best_score = std::numeric_limits<float>::lowest();
  Feature* best_feature_ptr = nullptr;

  // look in quad tree for *where* we want to find a feature
  std::vector<int> cells;
  std::vector<int> empty_cells;

  quad_tree_.GetMostlyEmptyCellList(cells, pROI);

  Feature* feature_ptr = nullptr;

  while (!cells.empty()) {
    empty_cells.clear();
    /*
        std::cout << "Searching in: [";
        for ( size_t ii = 0; ii < vCells.size(); ii++) {
            std::cout << " " << vCells[ii] << " ";
        }
        std::cout << "]" << std::endl;
        */

    for (size_t ii = 0; ii < cells.size(); ++ii) {
      feature_ptr = GetBestFeatureInCell(cells[ii]);
      // if best feat score is less than current feat score
      if (feature_ptr && best_score < feature_ptr->response) {
        best_score = feature_ptr->response;
        best_feature_ptr = feature_ptr;
      }

      if (feature_ptr == nullptr) {
        // no more good features in this cell, don't search in it anymore
        empty_cells.push_back(cells[ii]);
      }
    }

    if (!empty_cells.empty()) {
      quad_tree_.DeactivateCells(empty_cells);
    }

    if (best_feature_ptr == nullptr) {
      // try again again
      quad_tree_.GetMostlyEmptyCellList(cells);
    } else {
      // found the best feature, return!
      break;
    }
  }

  return best_feature_ptr;
}

void FeatureImage::ResetFeatures() {
  for (size_t ii = 0; ii < features_.size(); ++ii) {
    for (size_t jj = 0; jj < features_[ii].size(); ++jj) {
      features_[ii][jj]->used = false;
    }
  }

  // RESET QUADTREE!!!!!
}

/// Bucketting is done here to be able to consider all scales at once
void FeatureImage::_BucketKeyPoints(std::vector<cv::KeyPoint>& keypoints) {
  // NAIVE IMPLEMENTATION
  // select a constant number of features from each part of the image
  const size_t rows        = 1 << (quad_tree_.Levels()-1);
  const size_t cols        = 1 << (quad_tree_.Levels()-1);
  const size_t num_buckets = rows*cols;
  const float  x_cell_size =
      std::ceil(static_cast<float>(image_pyramid_->at(0).cols) / cols);
  const float  y_cell_size =
      std::ceil(static_cast<float>(image_pyramid_->at(0).rows) / rows);

  const double roi_top = static_cast<double>(CANONICAL_PATCH_SIZE) / 2.0;
  const double roi_left = static_cast<double>(CANONICAL_PATCH_SIZE) / 2.0;
  const double roi_bot =
      static_cast<double>(image_pyramid_->at(0).rows) - roi_top;
  const double roi_right =
      static_cast<double>(image_pyramid_->at(0).cols) - roi_left;

  std::vector<std::vector<std::pair<int, float> > > buckets(num_buckets);

  for (size_t ii = 0; ii < keypoints.size(); ++ii) {
    double x = keypoints[ii].pt.x;
    double y = keypoints[ii].pt.y;
    int r = static_cast<int>(y / y_cell_size);
    int c = static_cast<int>(x / x_cell_size);
    int b = r * cols + c;

    // filter points outside the allowed ROI
    if (x < roi_left || x > roi_right || y < roi_top || y > roi_bot) {
      continue;
    }

    // filter points according to their response value
    if (keypoints[ii].response > harris_score_threshold_) {
      buckets[b].push_back(std::pair<int, float>(ii, keypoints[ii].response));
    }
  }

  // sort the buckets
  std::vector<cv::KeyPoint> filtered_keypoints;
  for (size_t ii = 0; ii < num_buckets; ++ii) {
    std::sort(buckets[ii].begin(), buckets[ii].end(), _CompareKeyPoints);
    // take new keypoints
    size_t npts = std::min(max_features_in_cell_,
                           buckets[ii].size());
    for (size_t jj = 0; jj < npts; ++jj) {
      filtered_keypoints.push_back(keypoints[buckets[ii][jj].first]);
    }
  }

  keypoints = filtered_keypoints;
}
