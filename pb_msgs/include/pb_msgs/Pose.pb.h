// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Pose.proto

#ifndef PROTOBUF_Pose_2eproto__INCLUDED
#define PROTOBUF_Pose_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "Matrix.pb.h"
// @@protoc_insertion_point(includes)

namespace pb {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Pose_2eproto();
void protobuf_AssignDesc_Pose_2eproto();
void protobuf_ShutdownFile_Pose_2eproto();

class PoseMsg;

enum PoseMsg_Type {
  PoseMsg_Type_SO2 = 1,
  PoseMsg_Type_SE2 = 2,
  PoseMsg_Type_SO3 = 3,
  PoseMsg_Type_SE3 = 4,
  PoseMsg_Type_LatLongAlt = 5,
  PoseMsg_Type_Euler = 6,
  PoseMsg_Type_RAW = 9
};
bool PoseMsg_Type_IsValid(int value);
const PoseMsg_Type PoseMsg_Type_Type_MIN = PoseMsg_Type_SO2;
const PoseMsg_Type PoseMsg_Type_Type_MAX = PoseMsg_Type_RAW;
const int PoseMsg_Type_Type_ARRAYSIZE = PoseMsg_Type_Type_MAX + 1;

const ::google::protobuf::EnumDescriptor* PoseMsg_Type_descriptor();
inline const ::std::string& PoseMsg_Type_Name(PoseMsg_Type value) {
  return ::google::protobuf::internal::NameOfEnum(
    PoseMsg_Type_descriptor(), value);
}
inline bool PoseMsg_Type_Parse(
    const ::std::string& name, PoseMsg_Type* value) {
  return ::google::protobuf::internal::ParseNamedEnum<PoseMsg_Type>(
    PoseMsg_Type_descriptor(), name, value);
}
// ===================================================================

class PoseMsg : public ::google::protobuf::Message {
 public:
  PoseMsg();
  virtual ~PoseMsg();

  PoseMsg(const PoseMsg& from);

  inline PoseMsg& operator=(const PoseMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PoseMsg& default_instance();

  void Swap(PoseMsg* other);

  // implements Message ----------------------------------------------

  PoseMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PoseMsg& from);
  void MergeFrom(const PoseMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:

  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  typedef PoseMsg_Type Type;
  static const Type SO2 = PoseMsg_Type_SO2;
  static const Type SE2 = PoseMsg_Type_SE2;
  static const Type SO3 = PoseMsg_Type_SO3;
  static const Type SE3 = PoseMsg_Type_SE3;
  static const Type LatLongAlt = PoseMsg_Type_LatLongAlt;
  static const Type Euler = PoseMsg_Type_Euler;
  static const Type RAW = PoseMsg_Type_RAW;
  static inline bool Type_IsValid(int value) {
    return PoseMsg_Type_IsValid(value);
  }
  static const Type Type_MIN =
    PoseMsg_Type_Type_MIN;
  static const Type Type_MAX =
    PoseMsg_Type_Type_MAX;
  static const int Type_ARRAYSIZE =
    PoseMsg_Type_Type_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  Type_descriptor() {
    return PoseMsg_Type_descriptor();
  }
  static inline const ::std::string& Type_Name(Type value) {
    return PoseMsg_Type_Name(value);
  }
  static inline bool Type_Parse(const ::std::string& name,
      Type* value) {
    return PoseMsg_Type_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional int32 id = 1;
  inline bool has_id() const;
  inline void clear_id();
  static const int kIdFieldNumber = 1;
  inline ::google::protobuf::int32 id() const;
  inline void set_id(::google::protobuf::int32 value);

  // optional double device_time = 2;
  inline bool has_device_time() const;
  inline void clear_device_time();
  static const int kDeviceTimeFieldNumber = 2;
  inline double device_time() const;
  inline void set_device_time(double value);

  // required .pb.PoseMsg.Type type = 3;
  inline bool has_type() const;
  inline void clear_type();
  static const int kTypeFieldNumber = 3;
  inline ::pb::PoseMsg_Type type() const;
  inline void set_type(::pb::PoseMsg_Type value);

  // optional .pb.VectorMsg pose = 4;
  inline bool has_pose() const;
  inline void clear_pose();
  static const int kPoseFieldNumber = 4;
  inline const ::pb::VectorMsg& pose() const;
  inline ::pb::VectorMsg* mutable_pose();
  inline ::pb::VectorMsg* release_pose();
  inline void set_allocated_pose(::pb::VectorMsg* pose);

  // optional .pb.MatrixMsg covariance = 5;
  inline bool has_covariance() const;
  inline void clear_covariance();
  static const int kCovarianceFieldNumber = 5;
  inline const ::pb::MatrixMsg& covariance() const;
  inline ::pb::MatrixMsg* mutable_covariance();
  inline ::pb::MatrixMsg* release_covariance();
  inline void set_allocated_covariance(::pb::MatrixMsg* covariance);

  // @@protoc_insertion_point(class_scope:pb.PoseMsg)
 private:
  inline void set_has_id();
  inline void clear_has_id();
  inline void set_has_device_time();
  inline void clear_has_device_time();
  inline void set_has_type();
  inline void clear_has_type();
  inline void set_has_pose();
  inline void clear_has_pose();
  inline void set_has_covariance();
  inline void clear_has_covariance();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  double device_time_;
  ::google::protobuf::int32 id_;
  int type_;
  ::pb::VectorMsg* pose_;
  ::pb::MatrixMsg* covariance_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];

  friend void  protobuf_AddDesc_Pose_2eproto();
  friend void protobuf_AssignDesc_Pose_2eproto();
  friend void protobuf_ShutdownFile_Pose_2eproto();

  void InitAsDefaultInstance();
  static PoseMsg* default_instance_;
};
// ===================================================================


// ===================================================================

// PoseMsg

// optional int32 id = 1;
inline bool PoseMsg::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void PoseMsg::set_has_id() {
  _has_bits_[0] |= 0x00000001u;
}
inline void PoseMsg::clear_has_id() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void PoseMsg::clear_id() {
  id_ = 0;
  clear_has_id();
}
inline ::google::protobuf::int32 PoseMsg::id() const {
  return id_;
}
inline void PoseMsg::set_id(::google::protobuf::int32 value) {
  set_has_id();
  id_ = value;
}

// optional double device_time = 2;
inline bool PoseMsg::has_device_time() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void PoseMsg::set_has_device_time() {
  _has_bits_[0] |= 0x00000002u;
}
inline void PoseMsg::clear_has_device_time() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void PoseMsg::clear_device_time() {
  device_time_ = 0;
  clear_has_device_time();
}
inline double PoseMsg::device_time() const {
  return device_time_;
}
inline void PoseMsg::set_device_time(double value) {
  set_has_device_time();
  device_time_ = value;
}

// required .pb.PoseMsg.Type type = 3;
inline bool PoseMsg::has_type() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void PoseMsg::set_has_type() {
  _has_bits_[0] |= 0x00000004u;
}
inline void PoseMsg::clear_has_type() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void PoseMsg::clear_type() {
  type_ = 1;
  clear_has_type();
}
inline ::pb::PoseMsg_Type PoseMsg::type() const {
  return static_cast< ::pb::PoseMsg_Type >(type_);
}
inline void PoseMsg::set_type(::pb::PoseMsg_Type value) {
  assert(::pb::PoseMsg_Type_IsValid(value));
  set_has_type();
  type_ = value;
}

// optional .pb.VectorMsg pose = 4;
inline bool PoseMsg::has_pose() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void PoseMsg::set_has_pose() {
  _has_bits_[0] |= 0x00000008u;
}
inline void PoseMsg::clear_has_pose() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void PoseMsg::clear_pose() {
  if (pose_ != NULL) pose_->::pb::VectorMsg::Clear();
  clear_has_pose();
}
inline const ::pb::VectorMsg& PoseMsg::pose() const {
  return pose_ != NULL ? *pose_ : *default_instance_->pose_;
}
inline ::pb::VectorMsg* PoseMsg::mutable_pose() {
  set_has_pose();
  if (pose_ == NULL) pose_ = new ::pb::VectorMsg;
  return pose_;
}
inline ::pb::VectorMsg* PoseMsg::release_pose() {
  clear_has_pose();
  ::pb::VectorMsg* temp = pose_;
  pose_ = NULL;
  return temp;
}
inline void PoseMsg::set_allocated_pose(::pb::VectorMsg* pose) {
  delete pose_;
  pose_ = pose;
  if (pose) {
    set_has_pose();
  } else {
    clear_has_pose();
  }
}

// optional .pb.MatrixMsg covariance = 5;
inline bool PoseMsg::has_covariance() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void PoseMsg::set_has_covariance() {
  _has_bits_[0] |= 0x00000010u;
}
inline void PoseMsg::clear_has_covariance() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void PoseMsg::clear_covariance() {
  if (covariance_ != NULL) covariance_->::pb::MatrixMsg::Clear();
  clear_has_covariance();
}
inline const ::pb::MatrixMsg& PoseMsg::covariance() const {
  return covariance_ != NULL ? *covariance_ : *default_instance_->covariance_;
}
inline ::pb::MatrixMsg* PoseMsg::mutable_covariance() {
  set_has_covariance();
  if (covariance_ == NULL) covariance_ = new ::pb::MatrixMsg;
  return covariance_;
}
inline ::pb::MatrixMsg* PoseMsg::release_covariance() {
  clear_has_covariance();
  ::pb::MatrixMsg* temp = covariance_;
  covariance_ = NULL;
  return temp;
}
inline void PoseMsg::set_allocated_covariance(::pb::MatrixMsg* covariance) {
  delete covariance_;
  covariance_ = covariance;
  if (covariance) {
    set_has_covariance();
  } else {
    clear_has_covariance();
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::pb::PoseMsg_Type>() {
  return ::pb::PoseMsg_Type_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Pose_2eproto__INCLUDED
