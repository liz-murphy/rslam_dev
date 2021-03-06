// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: NodeCamMessage.proto

#ifndef PROTOBUF_NodeCamMessage_2eproto__INCLUDED
#define PROTOBUF_NodeCamMessage_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_NodeCamMessage_2eproto();
void protobuf_AssignDesc_NodeCamMessage_2eproto();
void protobuf_ShutdownFile_NodeCamMessage_2eproto();

class RegisterNodeCamReqMsg;
class RegisterNodeCamRepMsg;
class CamMsg;
class ImageMsg;

// ===================================================================

class RegisterNodeCamReqMsg : public ::google::protobuf::Message {
 public:
  RegisterNodeCamReqMsg();
  virtual ~RegisterNodeCamReqMsg();

  RegisterNodeCamReqMsg(const RegisterNodeCamReqMsg& from);

  inline RegisterNodeCamReqMsg& operator=(const RegisterNodeCamReqMsg& from) {
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
  static const RegisterNodeCamReqMsg& default_instance();

  void Swap(RegisterNodeCamReqMsg* other);

  // implements Message ----------------------------------------------

  RegisterNodeCamReqMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RegisterNodeCamReqMsg& from);
  void MergeFrom(const RegisterNodeCamReqMsg& from);
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

  // optional bytes uri = 1;
  inline bool has_uri() const;
  inline void clear_uri();
  static const int kUriFieldNumber = 1;
  inline const ::std::string& uri() const;
  inline void set_uri(const ::std::string& value);
  inline void set_uri(const char* value);
  inline void set_uri(const void* value, size_t size);
  inline ::std::string* mutable_uri();
  inline ::std::string* release_uri();
  inline void set_allocated_uri(::std::string* uri);

  // @@protoc_insertion_point(class_scope:RegisterNodeCamReqMsg)
 private:
  inline void set_has_uri();
  inline void clear_has_uri();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* uri_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(1 + 31) / 32];

  friend void  protobuf_AddDesc_NodeCamMessage_2eproto();
  friend void protobuf_AssignDesc_NodeCamMessage_2eproto();
  friend void protobuf_ShutdownFile_NodeCamMessage_2eproto();

  void InitAsDefaultInstance();
  static RegisterNodeCamReqMsg* default_instance_;
};
// -------------------------------------------------------------------

class RegisterNodeCamRepMsg : public ::google::protobuf::Message {
 public:
  RegisterNodeCamRepMsg();
  virtual ~RegisterNodeCamRepMsg();

  RegisterNodeCamRepMsg(const RegisterNodeCamRepMsg& from);

  inline RegisterNodeCamRepMsg& operator=(const RegisterNodeCamRepMsg& from) {
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
  static const RegisterNodeCamRepMsg& default_instance();

  void Swap(RegisterNodeCamRepMsg* other);

  // implements Message ----------------------------------------------

  RegisterNodeCamRepMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RegisterNodeCamRepMsg& from);
  void MergeFrom(const RegisterNodeCamRepMsg& from);
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

  // optional int32 time_step = 1;
  inline bool has_time_step() const;
  inline void clear_time_step();
  static const int kTimeStepFieldNumber = 1;
  inline ::google::protobuf::int32 time_step() const;
  inline void set_time_step(::google::protobuf::int32 value);

  // optional int32 regsiter_flag = 2;
  inline bool has_regsiter_flag() const;
  inline void clear_regsiter_flag();
  static const int kRegsiterFlagFieldNumber = 2;
  inline ::google::protobuf::int32 regsiter_flag() const;
  inline void set_regsiter_flag(::google::protobuf::int32 value);

  // optional int32 width = 3;
  inline bool has_width() const;
  inline void clear_width();
  static const int kWidthFieldNumber = 3;
  inline ::google::protobuf::int32 width() const;
  inline void set_width(::google::protobuf::int32 value);

  // optional int32 height = 4;
  inline bool has_height() const;
  inline void clear_height();
  static const int kHeightFieldNumber = 4;
  inline ::google::protobuf::int32 height() const;
  inline void set_height(::google::protobuf::int32 value);

  // optional int32 channels = 5;
  inline bool has_channels() const;
  inline void clear_channels();
  static const int kChannelsFieldNumber = 5;
  inline ::google::protobuf::int32 channels() const;
  inline void set_channels(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:RegisterNodeCamRepMsg)
 private:
  inline void set_has_time_step();
  inline void clear_has_time_step();
  inline void set_has_regsiter_flag();
  inline void clear_has_regsiter_flag();
  inline void set_has_width();
  inline void clear_has_width();
  inline void set_has_height();
  inline void clear_has_height();
  inline void set_has_channels();
  inline void clear_has_channels();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::int32 time_step_;
  ::google::protobuf::int32 regsiter_flag_;
  ::google::protobuf::int32 width_;
  ::google::protobuf::int32 height_;
  ::google::protobuf::int32 channels_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(5 + 31) / 32];

  friend void  protobuf_AddDesc_NodeCamMessage_2eproto();
  friend void protobuf_AssignDesc_NodeCamMessage_2eproto();
  friend void protobuf_ShutdownFile_NodeCamMessage_2eproto();

  void InitAsDefaultInstance();
  static RegisterNodeCamRepMsg* default_instance_;
};
// -------------------------------------------------------------------

class CamMsg : public ::google::protobuf::Message {
 public:
  CamMsg();
  virtual ~CamMsg();

  CamMsg(const CamMsg& from);

  inline CamMsg& operator=(const CamMsg& from) {
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
  static const CamMsg& default_instance();

  void Swap(CamMsg* other);

  // implements Message ----------------------------------------------

  CamMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const CamMsg& from);
  void MergeFrom(const CamMsg& from);
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

  // repeated .ImageMsg image = 1;
  inline int image_size() const;
  inline void clear_image();
  static const int kImageFieldNumber = 1;
  inline const ::ImageMsg& image(int index) const;
  inline ::ImageMsg* mutable_image(int index);
  inline ::ImageMsg* add_image();
  inline const ::google::protobuf::RepeatedPtrField< ::ImageMsg >&
      image() const;
  inline ::google::protobuf::RepeatedPtrField< ::ImageMsg >*
      mutable_image();

  // required int32 time_step = 2;
  inline bool has_time_step() const;
  inline void clear_time_step();
  static const int kTimeStepFieldNumber = 2;
  inline ::google::protobuf::int32 time_step() const;
  inline void set_time_step(::google::protobuf::int32 value);

  // optional int32 size = 3;
  inline bool has_size() const;
  inline void clear_size();
  static const int kSizeFieldNumber = 3;
  inline ::google::protobuf::int32 size() const;
  inline void set_size(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:CamMsg)
 private:
  inline void set_has_time_step();
  inline void clear_has_time_step();
  inline void set_has_size();
  inline void clear_has_size();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::RepeatedPtrField< ::ImageMsg > image_;
  ::google::protobuf::int32 time_step_;
  ::google::protobuf::int32 size_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(3 + 31) / 32];

  friend void  protobuf_AddDesc_NodeCamMessage_2eproto();
  friend void protobuf_AssignDesc_NodeCamMessage_2eproto();
  friend void protobuf_ShutdownFile_NodeCamMessage_2eproto();

  void InitAsDefaultInstance();
  static CamMsg* default_instance_;
};
// -------------------------------------------------------------------

class ImageMsg : public ::google::protobuf::Message {
 public:
  ImageMsg();
  virtual ~ImageMsg();

  ImageMsg(const ImageMsg& from);

  inline ImageMsg& operator=(const ImageMsg& from) {
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
  static const ImageMsg& default_instance();

  void Swap(ImageMsg* other);

  // implements Message ----------------------------------------------

  ImageMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const ImageMsg& from);
  void MergeFrom(const ImageMsg& from);
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

  // optional bytes image = 1;
  inline bool has_image() const;
  inline void clear_image();
  static const int kImageFieldNumber = 1;
  inline const ::std::string& image() const;
  inline void set_image(const ::std::string& value);
  inline void set_image(const char* value);
  inline void set_image(const void* value, size_t size);
  inline ::std::string* mutable_image();
  inline ::std::string* release_image();
  inline void set_allocated_image(::std::string* image);

  // optional int32 image_type = 2;
  inline bool has_image_type() const;
  inline void clear_image_type();
  static const int kImageTypeFieldNumber = 2;
  inline ::google::protobuf::int32 image_type() const;
  inline void set_image_type(::google::protobuf::int32 value);

  // optional int32 image_height = 3;
  inline bool has_image_height() const;
  inline void clear_image_height();
  static const int kImageHeightFieldNumber = 3;
  inline ::google::protobuf::int32 image_height() const;
  inline void set_image_height(::google::protobuf::int32 value);

  // optional int32 image_width = 4;
  inline bool has_image_width() const;
  inline void clear_image_width();
  static const int kImageWidthFieldNumber = 4;
  inline ::google::protobuf::int32 image_width() const;
  inline void set_image_width(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:ImageMsg)
 private:
  inline void set_has_image();
  inline void clear_has_image();
  inline void set_has_image_type();
  inline void clear_has_image_type();
  inline void set_has_image_height();
  inline void clear_has_image_height();
  inline void set_has_image_width();
  inline void clear_has_image_width();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::std::string* image_;
  ::google::protobuf::int32 image_type_;
  ::google::protobuf::int32 image_height_;
  ::google::protobuf::int32 image_width_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(4 + 31) / 32];

  friend void  protobuf_AddDesc_NodeCamMessage_2eproto();
  friend void protobuf_AssignDesc_NodeCamMessage_2eproto();
  friend void protobuf_ShutdownFile_NodeCamMessage_2eproto();

  void InitAsDefaultInstance();
  static ImageMsg* default_instance_;
};
// ===================================================================


// ===================================================================

// RegisterNodeCamReqMsg

// optional bytes uri = 1;
inline bool RegisterNodeCamReqMsg::has_uri() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RegisterNodeCamReqMsg::set_has_uri() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RegisterNodeCamReqMsg::clear_has_uri() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RegisterNodeCamReqMsg::clear_uri() {
  if (uri_ != &::google::protobuf::internal::kEmptyString) {
    uri_->clear();
  }
  clear_has_uri();
}
inline const ::std::string& RegisterNodeCamReqMsg::uri() const {
  return *uri_;
}
inline void RegisterNodeCamReqMsg::set_uri(const ::std::string& value) {
  set_has_uri();
  if (uri_ == &::google::protobuf::internal::kEmptyString) {
    uri_ = new ::std::string;
  }
  uri_->assign(value);
}
inline void RegisterNodeCamReqMsg::set_uri(const char* value) {
  set_has_uri();
  if (uri_ == &::google::protobuf::internal::kEmptyString) {
    uri_ = new ::std::string;
  }
  uri_->assign(value);
}
inline void RegisterNodeCamReqMsg::set_uri(const void* value, size_t size) {
  set_has_uri();
  if (uri_ == &::google::protobuf::internal::kEmptyString) {
    uri_ = new ::std::string;
  }
  uri_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* RegisterNodeCamReqMsg::mutable_uri() {
  set_has_uri();
  if (uri_ == &::google::protobuf::internal::kEmptyString) {
    uri_ = new ::std::string;
  }
  return uri_;
}
inline ::std::string* RegisterNodeCamReqMsg::release_uri() {
  clear_has_uri();
  if (uri_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = uri_;
    uri_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void RegisterNodeCamReqMsg::set_allocated_uri(::std::string* uri) {
  if (uri_ != &::google::protobuf::internal::kEmptyString) {
    delete uri_;
  }
  if (uri) {
    set_has_uri();
    uri_ = uri;
  } else {
    clear_has_uri();
    uri_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// -------------------------------------------------------------------

// RegisterNodeCamRepMsg

// optional int32 time_step = 1;
inline bool RegisterNodeCamRepMsg::has_time_step() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void RegisterNodeCamRepMsg::set_has_time_step() {
  _has_bits_[0] |= 0x00000001u;
}
inline void RegisterNodeCamRepMsg::clear_has_time_step() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void RegisterNodeCamRepMsg::clear_time_step() {
  time_step_ = 0;
  clear_has_time_step();
}
inline ::google::protobuf::int32 RegisterNodeCamRepMsg::time_step() const {
  return time_step_;
}
inline void RegisterNodeCamRepMsg::set_time_step(::google::protobuf::int32 value) {
  set_has_time_step();
  time_step_ = value;
}

// optional int32 regsiter_flag = 2;
inline bool RegisterNodeCamRepMsg::has_regsiter_flag() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void RegisterNodeCamRepMsg::set_has_regsiter_flag() {
  _has_bits_[0] |= 0x00000002u;
}
inline void RegisterNodeCamRepMsg::clear_has_regsiter_flag() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void RegisterNodeCamRepMsg::clear_regsiter_flag() {
  regsiter_flag_ = 0;
  clear_has_regsiter_flag();
}
inline ::google::protobuf::int32 RegisterNodeCamRepMsg::regsiter_flag() const {
  return regsiter_flag_;
}
inline void RegisterNodeCamRepMsg::set_regsiter_flag(::google::protobuf::int32 value) {
  set_has_regsiter_flag();
  regsiter_flag_ = value;
}

// optional int32 width = 3;
inline bool RegisterNodeCamRepMsg::has_width() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void RegisterNodeCamRepMsg::set_has_width() {
  _has_bits_[0] |= 0x00000004u;
}
inline void RegisterNodeCamRepMsg::clear_has_width() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void RegisterNodeCamRepMsg::clear_width() {
  width_ = 0;
  clear_has_width();
}
inline ::google::protobuf::int32 RegisterNodeCamRepMsg::width() const {
  return width_;
}
inline void RegisterNodeCamRepMsg::set_width(::google::protobuf::int32 value) {
  set_has_width();
  width_ = value;
}

// optional int32 height = 4;
inline bool RegisterNodeCamRepMsg::has_height() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void RegisterNodeCamRepMsg::set_has_height() {
  _has_bits_[0] |= 0x00000008u;
}
inline void RegisterNodeCamRepMsg::clear_has_height() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void RegisterNodeCamRepMsg::clear_height() {
  height_ = 0;
  clear_has_height();
}
inline ::google::protobuf::int32 RegisterNodeCamRepMsg::height() const {
  return height_;
}
inline void RegisterNodeCamRepMsg::set_height(::google::protobuf::int32 value) {
  set_has_height();
  height_ = value;
}

// optional int32 channels = 5;
inline bool RegisterNodeCamRepMsg::has_channels() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void RegisterNodeCamRepMsg::set_has_channels() {
  _has_bits_[0] |= 0x00000010u;
}
inline void RegisterNodeCamRepMsg::clear_has_channels() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void RegisterNodeCamRepMsg::clear_channels() {
  channels_ = 0;
  clear_has_channels();
}
inline ::google::protobuf::int32 RegisterNodeCamRepMsg::channels() const {
  return channels_;
}
inline void RegisterNodeCamRepMsg::set_channels(::google::protobuf::int32 value) {
  set_has_channels();
  channels_ = value;
}

// -------------------------------------------------------------------

// CamMsg

// repeated .ImageMsg image = 1;
inline int CamMsg::image_size() const {
  return image_.size();
}
inline void CamMsg::clear_image() {
  image_.Clear();
}
inline const ::ImageMsg& CamMsg::image(int index) const {
  return image_.Get(index);
}
inline ::ImageMsg* CamMsg::mutable_image(int index) {
  return image_.Mutable(index);
}
inline ::ImageMsg* CamMsg::add_image() {
  return image_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::ImageMsg >&
CamMsg::image() const {
  return image_;
}
inline ::google::protobuf::RepeatedPtrField< ::ImageMsg >*
CamMsg::mutable_image() {
  return &image_;
}

// required int32 time_step = 2;
inline bool CamMsg::has_time_step() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CamMsg::set_has_time_step() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CamMsg::clear_has_time_step() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CamMsg::clear_time_step() {
  time_step_ = 0;
  clear_has_time_step();
}
inline ::google::protobuf::int32 CamMsg::time_step() const {
  return time_step_;
}
inline void CamMsg::set_time_step(::google::protobuf::int32 value) {
  set_has_time_step();
  time_step_ = value;
}

// optional int32 size = 3;
inline bool CamMsg::has_size() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CamMsg::set_has_size() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CamMsg::clear_has_size() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CamMsg::clear_size() {
  size_ = 0;
  clear_has_size();
}
inline ::google::protobuf::int32 CamMsg::size() const {
  return size_;
}
inline void CamMsg::set_size(::google::protobuf::int32 value) {
  set_has_size();
  size_ = value;
}

// -------------------------------------------------------------------

// ImageMsg

// optional bytes image = 1;
inline bool ImageMsg::has_image() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ImageMsg::set_has_image() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ImageMsg::clear_has_image() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ImageMsg::clear_image() {
  if (image_ != &::google::protobuf::internal::kEmptyString) {
    image_->clear();
  }
  clear_has_image();
}
inline const ::std::string& ImageMsg::image() const {
  return *image_;
}
inline void ImageMsg::set_image(const ::std::string& value) {
  set_has_image();
  if (image_ == &::google::protobuf::internal::kEmptyString) {
    image_ = new ::std::string;
  }
  image_->assign(value);
}
inline void ImageMsg::set_image(const char* value) {
  set_has_image();
  if (image_ == &::google::protobuf::internal::kEmptyString) {
    image_ = new ::std::string;
  }
  image_->assign(value);
}
inline void ImageMsg::set_image(const void* value, size_t size) {
  set_has_image();
  if (image_ == &::google::protobuf::internal::kEmptyString) {
    image_ = new ::std::string;
  }
  image_->assign(reinterpret_cast<const char*>(value), size);
}
inline ::std::string* ImageMsg::mutable_image() {
  set_has_image();
  if (image_ == &::google::protobuf::internal::kEmptyString) {
    image_ = new ::std::string;
  }
  return image_;
}
inline ::std::string* ImageMsg::release_image() {
  clear_has_image();
  if (image_ == &::google::protobuf::internal::kEmptyString) {
    return NULL;
  } else {
    ::std::string* temp = image_;
    image_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
    return temp;
  }
}
inline void ImageMsg::set_allocated_image(::std::string* image) {
  if (image_ != &::google::protobuf::internal::kEmptyString) {
    delete image_;
  }
  if (image) {
    set_has_image();
    image_ = image;
  } else {
    clear_has_image();
    image_ = const_cast< ::std::string*>(&::google::protobuf::internal::kEmptyString);
  }
}

// optional int32 image_type = 2;
inline bool ImageMsg::has_image_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ImageMsg::set_has_image_type() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ImageMsg::clear_has_image_type() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ImageMsg::clear_image_type() {
  image_type_ = 0;
  clear_has_image_type();
}
inline ::google::protobuf::int32 ImageMsg::image_type() const {
  return image_type_;
}
inline void ImageMsg::set_image_type(::google::protobuf::int32 value) {
  set_has_image_type();
  image_type_ = value;
}

// optional int32 image_height = 3;
inline bool ImageMsg::has_image_height() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ImageMsg::set_has_image_height() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ImageMsg::clear_has_image_height() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ImageMsg::clear_image_height() {
  image_height_ = 0;
  clear_has_image_height();
}
inline ::google::protobuf::int32 ImageMsg::image_height() const {
  return image_height_;
}
inline void ImageMsg::set_image_height(::google::protobuf::int32 value) {
  set_has_image_height();
  image_height_ = value;
}

// optional int32 image_width = 4;
inline bool ImageMsg::has_image_width() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void ImageMsg::set_has_image_width() {
  _has_bits_[0] |= 0x00000008u;
}
inline void ImageMsg::clear_has_image_width() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void ImageMsg::clear_image_width() {
  image_width_ = 0;
  clear_has_image_width();
}
inline ::google::protobuf::int32 ImageMsg::image_width() const {
  return image_width_;
}
inline void ImageMsg::set_image_width(::google::protobuf::int32 value) {
  set_has_image_width();
  image_width_ = value;
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_NodeCamMessage_2eproto__INCLUDED
