// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Camera.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Camera.pb.h"

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

const ::google::protobuf::Descriptor* CameraMsg_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  CameraMsg_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Camera_2eproto() {
  protobuf_AddDesc_Camera_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Camera.proto");
  GOOGLE_CHECK(file != NULL);
  CameraMsg_descriptor_ = file->message_type(0);
  static const int CameraMsg_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraMsg, id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraMsg, device_time_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraMsg, image_),
  };
  CameraMsg_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      CameraMsg_descriptor_,
      CameraMsg::default_instance_,
      CameraMsg_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraMsg, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CameraMsg, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(CameraMsg));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Camera_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    CameraMsg_descriptor_, &CameraMsg::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Camera_2eproto() {
  delete CameraMsg::default_instance_;
  delete CameraMsg_reflection_;
}

void protobuf_AddDesc_Camera_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::pb::protobuf_AddDesc_Image_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\014Camera.proto\022\002pb\032\013Image.proto\"I\n\tCamer"
    "aMsg\022\n\n\002id\030\001 \001(\005\022\023\n\013device_time\030\002 \001(\001\022\033\n"
    "\005image\030\003 \003(\0132\014.pb.ImageMsg", 106);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Camera.proto", &protobuf_RegisterTypes);
  CameraMsg::default_instance_ = new CameraMsg();
  CameraMsg::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Camera_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Camera_2eproto {
  StaticDescriptorInitializer_Camera_2eproto() {
    protobuf_AddDesc_Camera_2eproto();
  }
} static_descriptor_initializer_Camera_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int CameraMsg::kIdFieldNumber;
const int CameraMsg::kDeviceTimeFieldNumber;
const int CameraMsg::kImageFieldNumber;
#endif  // !_MSC_VER

CameraMsg::CameraMsg()
  : ::google::protobuf::Message() {
  SharedCtor();
}

void CameraMsg::InitAsDefaultInstance() {
}

CameraMsg::CameraMsg(const CameraMsg& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
}

void CameraMsg::SharedCtor() {
  _cached_size_ = 0;
  id_ = 0;
  device_time_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

CameraMsg::~CameraMsg() {
  SharedDtor();
}

void CameraMsg::SharedDtor() {
  if (this != default_instance_) {
  }
}

void CameraMsg::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* CameraMsg::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return CameraMsg_descriptor_;
}

const CameraMsg& CameraMsg::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Camera_2eproto();
  return *default_instance_;
}

CameraMsg* CameraMsg::default_instance_ = NULL;

CameraMsg* CameraMsg::New() const {
  return new CameraMsg;
}

void CameraMsg::Clear() {
  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    id_ = 0;
    device_time_ = 0;
  }
  image_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool CameraMsg::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) return false
  ::google::protobuf::uint32 tag;
  while ((tag = input->ReadTag()) != 0) {
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 id = 1;
      case 1: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_VARINT) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &id_)));
          set_has_id();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(17)) goto parse_device_time;
        break;
      }

      // optional double device_time = 2;
      case 2: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_FIXED64) {
         parse_device_time:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &device_time_)));
          set_has_device_time();
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(26)) goto parse_image;
        break;
      }

      // repeated .pb.ImageMsg image = 3;
      case 3: {
        if (::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED) {
         parse_image:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_image()));
        } else {
          goto handle_uninterpreted;
        }
        if (input->ExpectTag(26)) goto parse_image;
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

void CameraMsg::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // optional int32 id = 1;
  if (has_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(1, this->id(), output);
  }

  // optional double device_time = 2;
  if (has_device_time()) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->device_time(), output);
  }

  // repeated .pb.ImageMsg image = 3;
  for (int i = 0; i < this->image_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->image(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
}

::google::protobuf::uint8* CameraMsg::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // optional int32 id = 1;
  if (has_id()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(1, this->id(), target);
  }

  // optional double device_time = 2;
  if (has_device_time()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->device_time(), target);
  }

  // repeated .pb.ImageMsg image = 3;
  for (int i = 0; i < this->image_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->image(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  return target;
}

int CameraMsg::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional int32 id = 1;
    if (has_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->id());
    }

    // optional double device_time = 2;
    if (has_device_time()) {
      total_size += 1 + 8;
    }

  }
  // repeated .pb.ImageMsg image = 3;
  total_size += 1 * this->image_size();
  for (int i = 0; i < this->image_size(); i++) {
    total_size +=
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        this->image(i));
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

void CameraMsg::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const CameraMsg* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const CameraMsg*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void CameraMsg::MergeFrom(const CameraMsg& from) {
  GOOGLE_CHECK_NE(&from, this);
  image_.MergeFrom(from.image_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_id()) {
      set_id(from.id());
    }
    if (from.has_device_time()) {
      set_device_time(from.device_time());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void CameraMsg::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CameraMsg::CopyFrom(const CameraMsg& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CameraMsg::IsInitialized() const {

  return true;
}

void CameraMsg::Swap(CameraMsg* other) {
  if (other != this) {
    std::swap(id_, other->id_);
    std::swap(device_time_, other->device_time_);
    image_.Swap(&other->image_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata CameraMsg::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = CameraMsg_descriptor_;
  metadata.reflection = CameraMsg_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

// @@protoc_insertion_point(global_scope)
