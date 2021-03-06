// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: template.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "template.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace pb {

namespace {

const ::google::protobuf::Descriptor* TemplateMsg_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  TemplateMsg_reflection_ = NULL;
const ::google::protobuf::Descriptor* TemplateCorpusMsg_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  TemplateCorpusMsg_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_template_2eproto() {
  protobuf_AddDesc_template_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "template.proto");
  GOOGLE_CHECK(file != NULL);
  TemplateMsg_descriptor_ = file->message_type(0);
  static const int TemplateMsg_offsets_[2] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateMsg, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateMsg, image_),
  };
  TemplateMsg_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      TemplateMsg_descriptor_,
      TemplateMsg::default_instance_,
      TemplateMsg_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateMsg, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateMsg, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(TemplateMsg));
  TemplateCorpusMsg_descriptor_ = file->message_type(1);
  static const int TemplateCorpusMsg_offsets_[1] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateCorpusMsg, place_),
  };
  TemplateCorpusMsg_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      TemplateCorpusMsg_descriptor_,
      TemplateCorpusMsg::default_instance_,
      TemplateCorpusMsg_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateCorpusMsg, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TemplateCorpusMsg, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(TemplateCorpusMsg));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_template_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    TemplateMsg_descriptor_, &TemplateMsg::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    TemplateCorpusMsg_descriptor_, &TemplateCorpusMsg::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_template_2eproto() {
  delete TemplateMsg::default_instance_;
  delete TemplateMsg_reflection_;
  delete TemplateCorpusMsg::default_instance_;
  delete TemplateCorpusMsg_reflection_;
}

void protobuf_AddDesc_template_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::pb::protobuf_AddDesc_Image_2eproto();
  ::pb::protobuf_AddDesc_rslam_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\016template.proto\022\002pb\032\013Image.proto\032\013rslam"
    ".proto\"O\n\013TemplateMsg\022#\n\002id\030\001 \001(\0132\027.pb.R"
    "eferenceFrameIdMsg\022\033\n\005image\030\002 \001(\0132\014.pb.I"
    "mageMsg\"3\n\021TemplateCorpusMsg\022\036\n\005place\030\001 "
    "\003(\0132\017.pb.TemplateMsg", 180);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "template.proto", &protobuf_RegisterTypes);
  TemplateMsg::default_instance_ = new TemplateMsg();
  TemplateCorpusMsg::default_instance_ = new TemplateCorpusMsg();
  TemplateMsg::default_instance_->InitAsDefaultInstance();
  TemplateCorpusMsg::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_template_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_template_2eproto {
  StaticDescriptorInitializer_template_2eproto() {
    protobuf_AddDesc_template_2eproto();
  }
} static_descriptor_initializer_template_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int TemplateMsg::kIdFieldNumber;
const int TemplateMsg::kImageFieldNumber;
#endif  // !_MSC_VER

TemplateMsg::TemplateMsg()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void TemplateMsg::InitAsDefaultInstance() {
  id_ = const_cast< ::pb::ReferenceFrameIdMsg*>(&::pb::ReferenceFrameIdMsg::default_instance());
  image_ = const_cast< ::pb::ImageMsg*>(&::pb::ImageMsg::default_instance());
}

TemplateMsg::TemplateMsg(const TemplateMsg& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void TemplateMsg::SharedCtor() {
  _cached_size_ = 0;
  id_ = NULL;
  image_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

TemplateMsg::~TemplateMsg() {
  SharedDtor();
}

void TemplateMsg::SharedDtor() {
  if (this != default_instance_) {
    delete id_;
    delete image_;
  }
}

void TemplateMsg::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* TemplateMsg::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return TemplateMsg_descriptor_;
}

const TemplateMsg& TemplateMsg::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_template_2eproto();
  return *default_instance_;
}

TemplateMsg* TemplateMsg::default_instance_ = NULL;

TemplateMsg* TemplateMsg::New() const {
  return new TemplateMsg;
}

void TemplateMsg::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (has_id()) {
      if (id_ != NULL) id_->::pb::ReferenceFrameIdMsg::Clear();
    }
    if (has_image()) {
      if (image_ != NULL) image_->::pb::ImageMsg::Clear();
    }
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool TemplateMsg::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .pb.ReferenceFrameIdMsg id = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_id()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(18)) goto parse_image;
        break;
      }

      // optional .pb.ImageMsg image = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_image:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_image()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void TemplateMsg::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional .pb.ReferenceFrameIdMsg id = 1;
  if (has_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->id(), output);
  }

  // optional .pb.ImageMsg image = 2;
  if (has_image()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->image(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* TemplateMsg::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // optional .pb.ReferenceFrameIdMsg id = 1;
  if (has_id()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->id(), target);
  }

  // optional .pb.ImageMsg image = 2;
  if (has_image()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        2, this->image(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int TemplateMsg::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional .pb.ReferenceFrameIdMsg id = 1;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->id());
    }

    // optional .pb.ImageMsg image = 2;
    if (has_image()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->image());
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void TemplateMsg::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const TemplateMsg* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const TemplateMsg*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void TemplateMsg::MergeFrom(const TemplateMsg& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_id()) {
      mutable_id()->::pb::ReferenceFrameIdMsg::MergeFrom(from.id());
    }
    if (from.has_image()) {
      mutable_image()->::pb::ImageMsg::MergeFrom(from.image());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void TemplateMsg::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TemplateMsg::CopyFrom(const TemplateMsg& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TemplateMsg::IsInitialized() const {

  return true;
}

void TemplateMsg::Swap(TemplateMsg* other) {
  if (other != this) {
    std::swap(id_, other->id_);
    std::swap(image_, other->image_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata TemplateMsg::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = TemplateMsg_descriptor_;
  metadata.reflection = TemplateMsg_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int TemplateCorpusMsg::kPlaceFieldNumber;
#endif  // !_MSC_VER

TemplateCorpusMsg::TemplateCorpusMsg()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void TemplateCorpusMsg::InitAsDefaultInstance() {
}

TemplateCorpusMsg::TemplateCorpusMsg(const TemplateCorpusMsg& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void TemplateCorpusMsg::SharedCtor() {
  _cached_size_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

TemplateCorpusMsg::~TemplateCorpusMsg() {
  SharedDtor();
}

void TemplateCorpusMsg::SharedDtor() {
  if (this != default_instance_) {
  }
}

void TemplateCorpusMsg::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* TemplateCorpusMsg::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return TemplateCorpusMsg_descriptor_;
}

const TemplateCorpusMsg& TemplateCorpusMsg::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_template_2eproto();
  return *default_instance_;
}

TemplateCorpusMsg* TemplateCorpusMsg::default_instance_ = NULL;

TemplateCorpusMsg* TemplateCorpusMsg::New() const {
  return new TemplateCorpusMsg;
}

void TemplateCorpusMsg::Clear() {
  place_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool TemplateCorpusMsg::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .pb.TemplateMsg place = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_place:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_place()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(10)) goto parse_place;
        if (input->ExpectAtEnd()) return true;
        break;
      }

      default: {
      handle_uninterpreted:
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          return true;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
  return true;
#undef DO_
}

void TemplateCorpusMsg::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // repeated .pb.TemplateMsg place = 1;
  for (int i = 0; i < this->place_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->place(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* TemplateCorpusMsg::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // repeated .pb.TemplateMsg place = 1;
  for (int i = 0; i < this->place_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->place(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int TemplateCorpusMsg::ByteSize() const {
  int total_size = 0;

  // repeated .pb.TemplateMsg place = 1;
  total_size += 1 * this->place_size();
  for (int i = 0; i < this->place_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->place(i));
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void TemplateCorpusMsg::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const TemplateCorpusMsg* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const TemplateCorpusMsg*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void TemplateCorpusMsg::MergeFrom(const TemplateCorpusMsg& from) {
  GOOGLE_CHECK_NE(&from, this);
  place_.MergeFrom(from.place_);
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void TemplateCorpusMsg::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TemplateCorpusMsg::CopyFrom(const TemplateCorpusMsg& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TemplateCorpusMsg::IsInitialized() const {

  return true;
}

void TemplateCorpusMsg::Swap(TemplateCorpusMsg* other) {
  if (other != this) {
    place_.Swap(&other->place_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata TemplateCorpusMsg::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = TemplateCorpusMsg_descriptor_;
  metadata.reflection = TemplateCorpusMsg_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

// @@protoc_insertion_point(global_scope)
