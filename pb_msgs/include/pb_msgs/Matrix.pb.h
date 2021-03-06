// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Matrix.proto

#ifndef PROTOBUF_Matrix_2eproto__INCLUDED
#define PROTOBUF_Matrix_2eproto__INCLUDED

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

namespace pb {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_Matrix_2eproto();
void protobuf_AssignDesc_Matrix_2eproto();
void protobuf_ShutdownFile_Matrix_2eproto();

class MatrixMsg;
class VectorMsg;

// ===================================================================

class MatrixMsg : public ::google::protobuf::Message {
 public:
  MatrixMsg();
  virtual ~MatrixMsg();

  MatrixMsg(const MatrixMsg& from);

  inline MatrixMsg& operator=(const MatrixMsg& from) {
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
  static const MatrixMsg& default_instance();

  void Swap(MatrixMsg* other);

  // implements Message ----------------------------------------------

  MatrixMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const MatrixMsg& from);
  void MergeFrom(const MatrixMsg& from);
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

  // required uint32 rows = 1;
  inline bool has_rows() const;
  inline void clear_rows();
  static const int kRowsFieldNumber = 1;
  inline ::google::protobuf::uint32 rows() const;
  inline void set_rows(::google::protobuf::uint32 value);

  // repeated double data = 2 [packed = true];
  inline int data_size() const;
  inline void clear_data();
  static const int kDataFieldNumber = 2;
  inline double data(int index) const;
  inline void set_data(int index, double value);
  inline void add_data(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      data() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_data();

  // @@protoc_insertion_point(class_scope:pb.MatrixMsg)
 private:
  inline void set_has_rows();
  inline void clear_has_rows();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::RepeatedField< double > data_;
  mutable int _data_cached_byte_size_;
  ::google::protobuf::uint32 rows_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(2 + 31) / 32];

  friend void  protobuf_AddDesc_Matrix_2eproto();
  friend void protobuf_AssignDesc_Matrix_2eproto();
  friend void protobuf_ShutdownFile_Matrix_2eproto();

  void InitAsDefaultInstance();
  static MatrixMsg* default_instance_;
};
// -------------------------------------------------------------------

class VectorMsg : public ::google::protobuf::Message {
 public:
  VectorMsg();
  virtual ~VectorMsg();

  VectorMsg(const VectorMsg& from);

  inline VectorMsg& operator=(const VectorMsg& from) {
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
  static const VectorMsg& default_instance();

  void Swap(VectorMsg* other);

  // implements Message ----------------------------------------------

  VectorMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const VectorMsg& from);
  void MergeFrom(const VectorMsg& from);
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

  // repeated double data = 1 [packed = true];
  inline int data_size() const;
  inline void clear_data();
  static const int kDataFieldNumber = 1;
  inline double data(int index) const;
  inline void set_data(int index, double value);
  inline void add_data(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      data() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_data();

  // @@protoc_insertion_point(class_scope:pb.VectorMsg)
 private:

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::RepeatedField< double > data_;
  mutable int _data_cached_byte_size_;

  mutable int _cached_size_;
  ::google::protobuf::uint32 _has_bits_[(1 + 31) / 32];

  friend void  protobuf_AddDesc_Matrix_2eproto();
  friend void protobuf_AssignDesc_Matrix_2eproto();
  friend void protobuf_ShutdownFile_Matrix_2eproto();

  void InitAsDefaultInstance();
  static VectorMsg* default_instance_;
};
// ===================================================================


// ===================================================================

// MatrixMsg

// required uint32 rows = 1;
inline bool MatrixMsg::has_rows() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MatrixMsg::set_has_rows() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MatrixMsg::clear_has_rows() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void MatrixMsg::clear_rows() {
  rows_ = 0u;
  clear_has_rows();
}
inline ::google::protobuf::uint32 MatrixMsg::rows() const {
  return rows_;
}
inline void MatrixMsg::set_rows(::google::protobuf::uint32 value) {
  set_has_rows();
  rows_ = value;
}

// repeated double data = 2 [packed = true];
inline int MatrixMsg::data_size() const {
  return data_.size();
}
inline void MatrixMsg::clear_data() {
  data_.Clear();
}
inline double MatrixMsg::data(int index) const {
  return data_.Get(index);
}
inline void MatrixMsg::set_data(int index, double value) {
  data_.Set(index, value);
}
inline void MatrixMsg::add_data(double value) {
  data_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
MatrixMsg::data() const {
  return data_;
}
inline ::google::protobuf::RepeatedField< double >*
MatrixMsg::mutable_data() {
  return &data_;
}

// -------------------------------------------------------------------

// VectorMsg

// repeated double data = 1 [packed = true];
inline int VectorMsg::data_size() const {
  return data_.size();
}
inline void VectorMsg::clear_data() {
  data_.Clear();
}
inline double VectorMsg::data(int index) const {
  return data_.Get(index);
}
inline void VectorMsg::set_data(int index, double value) {
  data_.Set(index, value);
}
inline void VectorMsg::add_data(double value) {
  data_.Add(value);
}
inline const ::google::protobuf::RepeatedField< double >&
VectorMsg::data() const {
  return data_;
}
inline ::google::protobuf::RepeatedField< double >*
VectorMsg::mutable_data() {
  return &data_;
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

#endif  // PROTOBUF_Matrix_2eproto__INCLUDED
