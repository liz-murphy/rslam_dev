// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <pb_msgs/ImageArray.h>
#include <rslam_engine/RslamEngineOptions.h>
#include <slam_map/SlamMapFwd.h>
#include <utils/MathTypes.h>

namespace rslam {

class RslamEngine;

/** A simplified interface to the RSLAM system */
class SimpleRslamEngine {
 public:
  SimpleRslamEngine();
  virtual ~SimpleRslamEngine();

  void Init(const RslamEngineOptions& options,
            const calibu::CameraRigT<Scalar>& rig,
            const std::shared_ptr<pb::ImageArray>& images);

  /**
   * Process the given frame with RSLAM
   */
  ReferenceFrameId AddFrame(const std::shared_ptr<pb::ImageArray>& images);

  /**
   * Retrieves the current ID being tracked in RSLAM
   *
   * @returns Whether the system has a current frame, which means it
   *          is initialized and has started tracking.
   */
  bool CurrentFrameId(ReferenceFrameId* id) const;

  /**
   * Finds the pose between the current frame and the given frame
   *
   * Pose is t_current_from_other.
   *
   * @returns False if the pose cannot be found (does not exist or not
   *          initialized)
   */
  bool GetRelativePose(const ReferenceFrameId& other, Sophus::SE3t* t_ab) const;

  /**
   * Finds the pose between the two given frames
   *
   * Pose is t_a_from_b.
   *
   * @returns False if the pose cannot be found (does not exist or not
   *          initialized)
   */
  bool GetRelativePose(const ReferenceFrameId& a, const ReferenceFrameId& b,
                       Sophus::SE3t* t_ab) const;

  /** Is the SLAM system initialized yet? */
  bool IsInitialized() const;

 private:
  std::unique_ptr<RslamEngine> engine_;
};

}  // namespace rslam
