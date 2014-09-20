// Copyright (c) George Washington University, all rights reserved. See the
// accompanying LICENSE file for more information.

#pragma once

#include <stdint.h>
#include <slam_map/SlamMapFwd.h>
#include <map>

namespace pb {
class PlaceMapMsg;
class SlamMapMsg;
}

class PlaceMatcher;
class TemplateMatcher;
class DBoWMatcher;
class MultiDBoWMatcher;

/**
 * Load the nodes, edges and cameras from a message into a map
 *
 * @param map_msg Load the messages from this protobuf message
 * @param slam_map Load into this SlamMap
 * @param zero_modified_time Should the loaded node/edge modification
 *        timestamps be set to zero?
 */
void LoadSlamMapMsg(const pb::SlamMapMsg& map_msg,
                    SlamMap* slam_map,
                    bool zero_modified_time);

/**
 * Load templates into a matcher and ID maps.
 *
 * All parameters except message are optional (can be null)
 */
void LoadTemplateMsg(const pb::PlaceMapMsg& place_map,
                     TemplateMatcher* matcher);

void LoadDBoWMsg(const pb::PlaceMapMsg& place_map,
                 DBoWMatcher* matcher);

void LoadMultiDBoWMsg(const pb::PlaceMapMsg& place_map,
                      MultiDBoWMatcher *matcher);

void LoadPlacesFromMsg(const pb::PlaceMapMsg& place_map,
                       PlaceMatcher* matcher);
