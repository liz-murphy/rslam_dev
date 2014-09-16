// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <slam_map/FrameObject.h>

namespace pb {
void fill_message(const std::shared_ptr<FrameObject>& fo, FrameObjectMsg* msg);
void fill_message(const std::shared_ptr<CubeObject>& fo, CubeObjectMsg* msg);
void fill_message(const std::shared_ptr<TeapotObject>& fo,
                  TeapotObjectMsg* msg);
void fill_message(const std::shared_ptr<TextObject>& fo, TextObjectMsg* msg);
void parse_message(const FrameObjectMsg& msg, std::shared_ptr<FrameObject>* fo);
void parse_message(const CubeObjectMsg& msg, std::shared_ptr<FrameObject>* fo);
void parse_message(const TeapotObjectMsg& msg,
                   std::shared_ptr<FrameObject>* fo);
void parse_message(const TextObjectMsg& msg, std::shared_ptr<FrameObject>* fo);
}  // namespace pb
