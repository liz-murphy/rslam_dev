// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Header.proto

#ifndef PROTOBUF_Header_2eproto__INCLUDED
#define PROTOBUF_Header_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#include "CameraModel.pb.h"
// @@protoc_insertion_point(includes)

namespace pb {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Header_2eproto();
void protobuf_AssignDesc_Header_2eproto();
void protobuf_ShutdownFile_Header_2eproto();

class Header;

// ===================================================================

class Header : public ::google::protobuf::Message {
 public:
  Header();
  virtual ~Header();

  Header(const Header& from);

  inline Header& operator=(const Header& from) {
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
  static const Header& default_instance();

  void Swap(Header* other);

  // implements Message ----------------------------------------------

  Header* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
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

  // accessors -------------------------------------------------------

  // required uint32 version = 1;
  inline bool has_version() const;
  inline void clear_version();
  static const int kVersionFieldNumber = 1;
  inline ::google::protobuf::uint32 version() const;
  inline void set_version(::google::protobuf::uint32 value);

  // optional double date = 2;
  inline bool has_date() const;
  inline void clear_date();
  static const int kDateFieldNumber = 2;
  inline double date() const;
  inline void set_date(double value);

  // optional string description = 3;
  inline bool has_description() const;
  inline void clear_description();
  static const int kDescriptionFieldNumber = 3;
  inline const ::std::string& description() const;
  inline void set_description(const ::std::string& value);
  inline void set_description(const char* value);
  inline void set_description(const char* value, size_t size);
  inline ::std::string* mutable_description();
  inline ::std::string* release_description();
  inline void set_allocated_description(::std::string* description);

  // optional .pb.CameraModelMsg camera_model = 5;
  inline bool has_camera_model() const;
  inline void clear_camera_model();
  static const int kCameraModelFieldNumber = 5;
  inline const ::pb::CameraModelMsg& camera_model() const;
  inline ::pb::CameraModelMsg* mutable_camera_model();
  inline ::pb::CameraModelMsg* release_camera_model();
  inline void set_allocated_camera_model(::pb::CameraModelMsg* camera_model);

  // @@protoc_insertion_point(class_scope:pb.Header)
 private:
  inline void set_has_version();
  inline void clear_has_version();
  inline void set_has_date();
  inline void clear_has_date();
  inline void set_has_description();
  inline void clear_has_description();
  inline void set_has_camera_model();
  inline void clear_has_camera_model();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  double date_;
  ::std::string* description_;
  ::pb::CameraModelMsg* camera_model_;
  ::google::protobuf::uint32 version_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];

  friend void  protobuf_AddDesc_Header_2eproto();
  friend void protobuf_AssignDesc_Header_2eproto();
  friend void protobuf_ShutdownFile_Header_2eproto();

  void InitAsDefaultInstance();
  static Header* default_instance_;
};
// ===================================================================


// ===================================================================

// Header

// required uint32 version = 1;
inline bool Header::has_version() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Header::set_has_version() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Header::clear_has_version() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Header::clear_version() {
  version_ = 0u;
  clear_has_version();
}
inline ::google::protobuf::uint32 Header::version() const {
  return version_;
}
inline void Header::set_version(::google::protobuf::uint32 value) {
  set_has_version();
  version_ = value;
}

// optional double date = 2;
inline bool Header::has_date() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Header::set_has_date() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Header::clear_has_date() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Header::clear_date() {
  date_ = 0;
  clear_has_date();
}
inline double Header::date() const {
  return date_;
}
inline void Header::set_date(double value) {
  set_has_date();
  date_ = value;
}

// optional string description = 3;
inline bool Header::has_description() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Header::set_has_description() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Header::clear_has_description() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Header::clear_description() {
  if (description_ != &::google::protobuf::internal::kEmptyString) {
    description_->clear();
  }
  clear_has_description();
}
inline const ::std::string& Header::description() const {
  return *description_;
}
inline void Header::set_description(const ::std::string& value) {
  set_has_description();
  if (description_ == &::google::protobuf::internal::kEmptyString) {
    description_ = new ::std::string;
  }
  description_->assign(value);
}
inline void Header::set_description(const char* value) {
  set_has_description();
  if (description_ == &::google::protobuf::internal::kEmptyString) {
    description_ = new ::std::string;
  }
  description_->assign(value);
}
inline void Header::set_description(const char* value, size_t size) {
  set_has_description();
  if (description_ == &::google::protobuf::internal::kEmptyString) {
    description_ = new ::std::string;
  }
  description_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* Header::mutable_description() {
  set_has_description();
  if (description_ == &::google::protobuf::internal::kEmptyString) {
    description_ = new ::std::string;
  }
  return description_;
}
inline ::std::string* Header::release_description() {
  clear_has_description();
  if (description_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = description_;
    description_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void Header::set_allocated_description(::std::string* description) {
  if (description_ != &::google::protobuf::internal::kEmptyString) {
    delete description_;
  }
  if (description) {
    set_has_description();
    description_ = description;
  } else {
    clear_has_description();
    description_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional .pb.CameraModelMsg camera_model = 5;
inline bool Header::has_camera_model() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Header::set_has_camera_model() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Header::clear_has_camera_model() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Header::clear_camera_model() {
  if (camera_model_ != NULL) camera_model_->::pb::CameraModelMsg::Clear();
  clear_has_camera_model();
}
inline const ::pb::CameraModelMsg& Header::camera_model() const {
  return camera_model_ != NULL ? *camera_model_ : *default_instance_->camera_model_;
}
inline ::pb::CameraModelMsg* Header::mutable_camera_model() {
  set_has_camera_model();
  if (camera_model_ == NULL) camera_model_ = new ::pb::CameraModelMsg;
  return camera_model_;
}
inline ::pb::CameraModelMsg* Header::release_camera_model() {
  clear_has_camera_model();
  ::pb::CameraModelMsg* temp = camera_model_;
  camera_model_ = NULL;
  return temp;
}
inline void Header::set_allocated_camera_model(::pb::CameraModelMsg* camera_model) {
  delete camera_model_;
  camera_model_ = camera_model;
  if (camera_model) {
    set_has_camera_model();
  } else {
    clear_has_camera_model();
  }
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace pb

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_Header_2eproto__INCLUDED
