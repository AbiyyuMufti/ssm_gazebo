// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: color.proto

#ifndef PROTOBUF_INCLUDED_color_2eproto
#define PROTOBUF_INCLUDED_color_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_color_2eproto 

namespace protobuf_color_2eproto {
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
}  // namespace protobuf_color_2eproto
namespace gazebo {
namespace msgs {
class Color;
class ColorDefaultTypeInternal;
extern ColorDefaultTypeInternal _Color_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::Color* Arena::CreateMaybeMessage<::gazebo::msgs::Color>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

// ===================================================================

class Color : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Color) */ {
 public:
  Color();
  virtual ~Color();

  Color(const Color& from);

  inline Color& operator=(const Color& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Color(Color&& from) noexcept
    : Color() {
    *this = ::std::move(from);
  }

  inline Color& operator=(Color&& from) noexcept {
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
  static const Color& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Color* internal_default_instance() {
    return reinterpret_cast<const Color*>(
               &_Color_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Color* other);
  friend void swap(Color& a, Color& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Color* New() const final {
    return CreateMaybeMessage<Color>(NULL);
  }

  Color* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Color>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Color& from);
  void MergeFrom(const Color& from);
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
  void InternalSwap(Color* other);
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

  // required float r = 2;
  bool has_r() const;
  void clear_r();
  static const int kRFieldNumber = 2;
  float r() const;
  void set_r(float value);

  // required float g = 3;
  bool has_g() const;
  void clear_g();
  static const int kGFieldNumber = 3;
  float g() const;
  void set_g(float value);

  // required float b = 4;
  bool has_b() const;
  void clear_b();
  static const int kBFieldNumber = 4;
  float b() const;
  void set_b(float value);

  // optional float a = 5 [default = 1];
  bool has_a() const;
  void clear_a();
  static const int kAFieldNumber = 5;
  float a() const;
  void set_a(float value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Color)
 private:
  void set_has_r();
  void clear_has_r();
  void set_has_g();
  void clear_has_g();
  void set_has_b();
  void clear_has_b();
  void set_has_a();
  void clear_has_a();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  float r_;
  float g_;
  float b_;
  float a_;
  friend struct ::protobuf_color_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Color

// required float r = 2;
inline bool Color::has_r() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Color::set_has_r() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Color::clear_has_r() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Color::clear_r() {
  r_ = 0;
  clear_has_r();
}
inline float Color::r() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Color.r)
  return r_;
}
inline void Color::set_r(float value) {
  set_has_r();
  r_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Color.r)
}

// required float g = 3;
inline bool Color::has_g() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Color::set_has_g() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Color::clear_has_g() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Color::clear_g() {
  g_ = 0;
  clear_has_g();
}
inline float Color::g() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Color.g)
  return g_;
}
inline void Color::set_g(float value) {
  set_has_g();
  g_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Color.g)
}

// required float b = 4;
inline bool Color::has_b() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Color::set_has_b() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Color::clear_has_b() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Color::clear_b() {
  b_ = 0;
  clear_has_b();
}
inline float Color::b() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Color.b)
  return b_;
}
inline void Color::set_b(float value) {
  set_has_b();
  b_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Color.b)
}

// optional float a = 5 [default = 1];
inline bool Color::has_a() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Color::set_has_a() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Color::clear_has_a() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Color::clear_a() {
  a_ = 1;
  clear_has_a();
}
inline float Color::a() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Color.a)
  return a_;
}
inline void Color::set_a(float value) {
  set_has_a();
  a_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Color.a)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_color_2eproto
