#pragma once
#include <stdint.h>
#include <vector>
#include <cstring>
#include <Eigen/Eigen>


namespace sdtrack {

class FeatureMask {
  static const long kFeatureMaskRadius = 5;
  typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> MaskT;
 public:
  FeatureMask() = default;
  FeatureMask(const FeatureMask&) = default;
  ~FeatureMask() = default;

  void AddImage(int width, int height) {
    feature_mask_.push_back(MaskT::Zero(height, width));
  }

  void SetMask(size_t cam, long x, long y) {
    CHECK_LT(cam, feature_mask_.size());

    // set an area around the x/y coordinate as masked
    size_t min_col = std::max(0L, x - kFeatureMaskRadius);
    size_t max_col = std::min(static_cast<long>(feature_mask_[cam].cols() - 1L),
                              x + kFeatureMaskRadius);
    size_t min_row = std::max(0L, y - kFeatureMaskRadius);
    size_t max_row = std::min(static_cast<long>(feature_mask_[cam].rows() - 1L),
                              y + kFeatureMaskRadius);

    feature_mask_[cam].block(min_row, min_col,
                             max_row - min_row,
                             max_col - min_col).setOnes();
  }

  /** Returns true if the point is masked */
  bool GetMask(int cam, int x, int y) {
    return feature_mask_[cam](y, x) == 1;
  }

  void Clear() {
    for (MaskT& mask : feature_mask_) {
      mask.setZero();
    }
  }

 private:
  std::vector<MaskT> feature_mask_;
};
}  // namespace sdtrack
