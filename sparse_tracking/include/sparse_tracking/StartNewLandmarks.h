// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#ifndef _START_NEW_LANDMARKS_H_
#define _START_NEW_LANDMARKS_H_

#include <vector>
#include <slam_map/SlamMapFwd.h>
#include <feature_utils/FeatureImage.h>
#include <sparse_tracking/FeatureMask.h>


namespace rslam {
namespace sparse {

///
/// \brief Add new landmarks in image regions where too few are tracked.
/// Attempt to triangulate if possible.
/// \param[in] rig  multi-view camera rig
/// \param[in,out] map pointer
/// \param[in,out] current_frame pointer in the map
/// \param[in,out] images that store the detected features
/// \param[in,out] feature_options
/// \param[out] num_tracked_landmarks
/// \param[out] num_new_landmarks
///
void StartNewLandmarks(const calibu::CameraRigT<Scalar> &rig,
                       const std::shared_ptr<SlamMap>   &map,
                       SlamFramePtr                     current_frame,
                       FeatureImageVector               &images,
                       unsigned int                     &num_tracked_landmarks,
                       unsigned int                     &num_new_landmarks,
                       sparse::FeatureMask              &feature_mask
                       );

#endif	/* STARTNEWLANDMARKS_H */

}
}
