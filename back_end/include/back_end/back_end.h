// Copyright (c) John Morrison, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <optimization/optimization.h>

namespace rslam {
namespace backend {

/**
 * SLAM BackEnd process encapsulation
 *
 * API:
 * - Adaptive Asynchronous Conditioning Bundle Adjuster
 * - Global pose measurement relaxation
 *
 * Internally:
 * - Runs asynchronous threads which perform smart map optimizations
 *   in response to callbacks from the SlamMap.
 */
class BackEnd {
 public:
  BackEnd();
  ~BackEnd();

  void Init(const std::shared_ptr<SlamMap>& map);
  void Run();
  void Stop();

  bool is_running() const {
    return is_running_;
  }

  /** Perform an adaptive bundle adjustment starting from 'root' */
  void AACBundleAdjustment(const ReferenceFrameId& root);

  /** Perform an intelligent pose graph relaxation starting from 'root' */
  void PoseGraphRelaxation(const ReferenceFrameId& root);

 protected:
  void PGRThread();

 private:
  std::shared_ptr<SlamMap> map_;
  std::atomic<bool> is_running_;

  // Pose Graph Relaxation Optimizer
  optimization::Optimization pgr_optimization_;
  std::thread pgr_thread_;
  mutable std::mutex pgr_mutex_;
};
}  // end namespace backend
}  // end namespace rslam
