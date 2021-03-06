// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: double.proto

#ifndef PROTOBUF_INCLUDED_double_2eproto
#define PROTOBUF_INCLUDED_double_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_double_2eproto 

namespace protobuf_double_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_double_2eproto
namespace gazebo {
namespace msgs {
class Double;
class DoubleDefaultTypeInternal;
extern DoubleDefaultTypeInternal _Double_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::Double* Arena::CreateMaybeMessage<::gazebo::msgs::Double>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

// ===================================================================

class Double : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Double) */ {
 public:
  Double();
  virtual ~Double();

  Double(const Double& from);

  inline Double& operator=(const Double& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Double(Double&& from) noexcept
    : Double() {
    *this = ::std::move(from);
  }

  inline Double& operator=(Double&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Double& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Double* internal_default_instance() {
    return reinterpret_cast<const Double*>(
               &_Double_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Double* other);
  friend void swap(Double& a, Double& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Double* New() const final {
    return CreateMaybeMessage<Double>(NULL);
  }

  Double* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Double>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Double& from);
  void MergeFrom(const Double& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Double* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required double data = 1;
  bool has_data() const;
  void clear_data();
  static const int kDataFieldNumber = 1;
  double data() const;
  void set_data(double value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Double)
 private:
  void set_has_data();
  void clear_has_data();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  double data_;
  friend struct ::protobuf_double_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Double

// required double data = 1;
inline bool Double::has_data() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Double::set_has_data() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Double::clear_has_data() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Double::clear_data() {
  data_ = 0;
  clear_has_data();
}
inline double Double::data() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Double.data)
  return data_;
}
inline void Double::set_data(double value) {
  set_has_data();
  data_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Double.data)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_double_2eproto
