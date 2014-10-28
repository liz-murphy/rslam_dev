// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#include <slam_map/FrameObjectProtobufIO.h>
#include <slam_map/ProtobufIO.h>
#include <pb_msgs/Matrix.h>
#include <pb_msgs/Pose.h>

namespace pb {

void fill_message(const std::shared_ptr<FrameObject>& fo,
                  FrameObjectMsg* msg) {
  msg->set_uuid(rslam::uuid::uuid_begin(fo->id),
                rslam::uuid::uuid_size(fo->id));

  switch (fo->type) {
    case FrameObject::Type::Cube:
      msg->set_type(FrameObjectMsg::Cube);
      fill_message(std::static_pointer_cast<CubeObject>(fo),
                   msg->MutableExtension(CubeObjectMsg::object));
      break;
    case FrameObject::Type::Teapot:
      msg->set_type(FrameObjectMsg::Teapot);
      fill_message(std::static_pointer_cast<TeapotObject>(fo),
                   msg->MutableExtension(TeapotObjectMsg::object));
      break;
    case FrameObject::Type::Text:
      msg->set_type(FrameObjectMsg::Text);
      fill_message(std::static_pointer_cast<TextObject>(fo),
                   msg->MutableExtension(TextObjectMsg::object));
      break;
    default:
      ROS_ERROR("Unknown FrameObject type being serialized");
  }
}

void fill_message(const std::shared_ptr<CubeObject>& fo,
                  CubeObjectMsg* msg) {

  WritePoseSE3(fo->t_po, msg->mutable_t_po());
  fill_message(fo->scale, msg->mutable_scale());
}

void fill_message(const std::shared_ptr<TextObject>& fo,
                  TextObjectMsg* msg) {

  WritePoseSE3(fo->t_po, msg->mutable_t_po());
  fill_message(fo->scale, msg->mutable_scale());
  msg->set_text(fo->text);
  fill_message(fo->color, msg->mutable_color());
}

void fill_message(const std::shared_ptr<TeapotObject>& fo,
                  TeapotObjectMsg* msg) {

  WritePoseSE3(fo->t_po, msg->mutable_t_po());
  fill_message(fo->scale, msg->mutable_scale());
}

void parse_message(const FrameObjectMsg& msg,
                   std::shared_ptr<FrameObject>* fo) {

  switch (msg.type()) {
    case FrameObjectMsg::Cube:
      parse_message(msg.GetExtension(CubeObjectMsg::object), fo);
      break;
    case FrameObjectMsg::Teapot:
      parse_message(msg.GetExtension(TeapotObjectMsg::object), fo);
      break;
    case FrameObjectMsg::Text:
      parse_message(msg.GetExtension(TextObjectMsg::object), fo);
      break;
    default:
      ROS_ERROR("Unknown FrameObject type being serialized");
  }

  std::copy(msg.uuid().begin(), msg.uuid().end(),
            rslam::uuid::uuid_begin((*fo)->id));
}

void parse_message(const CubeObjectMsg& msg,
                   std::shared_ptr<FrameObject>* fo) {

  CubeObject* cube = new CubeObject;
  cube->type = FrameObject::Type::Cube;

  ReadPoseSE3(msg.t_po(), &cube->t_po);
  parse_message(msg.scale(), &cube->scale);
  fo->reset(cube);
}

void parse_message(const TeapotObjectMsg& msg,
                   std::shared_ptr<FrameObject>* fo) {

  TeapotObject* teapot = new TeapotObject;
  teapot->type = FrameObject::Type::Teapot;

  ReadPoseSE3(msg.t_po(), &teapot->t_po);
  parse_message(msg.scale(), &teapot->scale);
  fo->reset(teapot);
}

void parse_message(const TextObjectMsg& msg,
                   std::shared_ptr<FrameObject>* fo) {

  TextObject* text = new TextObject;
  text->type = FrameObject::Type::Text;

  ReadPoseSE3(msg.t_po(), &text->t_po);
  parse_message(msg.scale(), &text->scale);
  parse_message(msg.color(), &text->color);
  text->text = msg.text();
  fo->reset(text);
}
}  // namespace pb
