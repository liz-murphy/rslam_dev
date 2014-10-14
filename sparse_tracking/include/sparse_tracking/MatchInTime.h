// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <common_front_end/FeatureImage.h>
#include <slam_map/SlamMapFwd.h>
#include <local_map/LocalMap.h>

///
/// \brief MatchInTime - Find 3D to 2D matches
/// \param[in] images - set of images that will be searched for matches
/// \param[in] working_set - set of 3D landmarks
/// \param[in] feature_options
/// \param[out] new_measurements - matched 2d features in our msr format
/// \param[out] feature_matches - pointers to matched features
/// \return
///
int MatchInTime(const ReferenceFrameId                &frame_id,
                const FeatureImageVector              &images,
                const LocalMap                        &working_set,
                std::vector<MultiViewMeasurement>     &new_measurements,
                std::vector< std::vector<Feature*> >  &feature_matches,
                float                                 search_radius);
