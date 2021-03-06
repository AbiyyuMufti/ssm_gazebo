// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: imagegeom.proto

#ifndef PROTOBUF_INCLUDED_imagegeom_2eproto
#define PROTOBUF_INCLUDED_imagegeom_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_imagegeom_2eproto 

namespace protobuf_imagegeom_2eproto {
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
}  // namespace protobuf_imagegeom_2eproto
namespace gazebo {
namespace msgs {
class ImageGeom;
class ImageGeomDefaultTypeInternal;
extern ImageGeomDefaultTypeInternal _ImageGeom_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::ImageGeom* Arena::CreateMaybeMessage<::gazebo::msgs::ImageGeom>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

// ===================================================================

class ImageGeom : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.ImageGeom) */ {
 public:
  ImageGeom();
  virtual ~ImageGeom();

  ImageGeom(const ImageGeom& from);

  inline ImageGeom& operator=(const ImageGeom& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ImageGeom(ImageGeom&& from) noexcept
    : ImageGeom() {
    *this = ::std::move(from);
  }

  inline ImageGeom& operator=(ImageGeom&& from) noexcept {
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
  static const ImageGeom& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ImageGeom* internal_default_instance() {
    return reinterpret_cast<const ImageGeom*>(
               &_ImageGeom_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(ImageGeom* other);
  friend void swap(ImageGeom& a, ImageGeom& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ImageGeom* New() const final {
    return CreateMaybeMessage<ImageGeom>(NULL);
  }

  ImageGeom* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<ImageGeom>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const ImageGeom& from);
  void MergeFrom(const ImageGeom& from);
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
  void InternalSwap(ImageGeom* other);
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

  // required string uri = 1;
  bool has_uri() const;
  void clear_uri();
  static const int kUriFieldNumber = 1;
  const ::std::string& uri() const;
  void set_uri(const ::std::string& value);
  #if LANG_CXX11
  void set_uri(::std::string&& value);
  #endif
  void set_uri(const char* value);
  void set_uri(const char* value, size_t size);
  ::std::string* mutable_uri();
  ::std::string* release_uri();
  void set_allocated_uri(::std::string* uri);

  // optional double scale = 2;
  bool has_scale() const;
  void clear_scale();
  static const int kScaleFieldNumber = 2;
  double scale() const;
  void set_scale(double value);

  // optional double height = 4;
  bool has_height() const;
  void clear_height();
  static const int kHeightFieldNumber = 4;
  double height() const;
  void set_height(double value);

  // optional int32 granularity = 5;
  bool has_granularity() const;
  void clear_granularity();
  static const int kGranularityFieldNumber = 5;
  ::google::protobuf::int32 granularity() const;
  void set_granularity(::google::protobuf::int32 value);

  // optional int32 threshold = 3 [default = 255];
  bool has_threshold() const;
  void clear_threshold();
  static const int kThresholdFieldNumber = 3;
  ::google::protobuf::int32 threshold() const;
  void set_threshold(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.ImageGeom)
 private:
  void set_has_uri();
  void clear_has_uri();
  void set_has_scale();
  void clear_has_scale();
  void set_has_threshold();
  void clear_has_threshold();
  void set_has_height();
  void clear_has_height();
  void set_has_granularity();
  void clear_has_granularity();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr uri_;
  double scale_;
  double height_;
  ::google::protobuf::int32 granularity_;
  ::google::protobuf::int32 threshold_;
  friend struct ::protobuf_imagegeom_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ImageGeom

// required string uri = 1;
inline bool ImageGeom::has_uri() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ImageGeom::set_has_uri() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ImageGeom::clear_has_uri() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ImageGeom::clear_uri() {
  uri_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_uri();
}
inline const ::std::string& ImageGeom::uri() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageGeom.uri)
  return uri_.GetNoArena();
}
inline void ImageGeom::set_uri(const ::std::string& value) {
  set_has_uri();
  uri_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.ImageGeom.uri)
}
#if LANG_CXX11
inline void ImageGeom::set_uri(::std::string&& value) {
  set_has_uri();
  uri_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.ImageGeom.uri)
}
#endif
inline void ImageGeom::set_uri(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_uri();
  uri_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.ImageGeom.uri)
}
inline void ImageGeom::set_uri(const char* value, size_t size) {
  set_has_uri();
  uri_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.ImageGeom.uri)
}
inline ::std::string* ImageGeom::mutable_uri() {
  set_has_uri();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.ImageGeom.uri)
  return uri_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* ImageGeom::release_uri() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.ImageGeom.uri)
  if (!has_uri()) {
    return NULL;
  }
  clear_has_uri();
  return uri_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void ImageGeom::set_allocated_uri(::std::string* uri) {
  if (uri != NULL) {
    set_has_uri();
  } else {
    clear_has_uri();
  }
  uri_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), uri);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.ImageGeom.uri)
}

// optional double scale = 2;
inline bool ImageGeom::has_scale() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ImageGeom::set_has_scale() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ImageGeom::clear_has_scale() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ImageGeom::clear_scale() {
  scale_ = 0;
  clear_has_scale();
}
inline double ImageGeom::scale() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageGeom.scale)
  return scale_;
}
inline void ImageGeom::set_scale(double value) {
  set_has_scale();
  scale_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.ImageGeom.scale)
}

// optional int32 threshold = 3 [default = 255];
inline bool ImageGeom::has_threshold() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void ImageGeom::set_has_threshold() {
  _has_bits_[0] |= 0x00000010u;
}
inline void ImageGeom::clear_has_threshold() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void ImageGeom::clear_threshold() {
  threshold_ = 255;
  clear_has_threshold();
}
inline ::google::protobuf::int32 ImageGeom::threshold() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageGeom.threshold)
  return threshold_;
}
inline void ImageGeom::set_threshold(::google::protobuf::int32 value) {
  set_has_threshold();
  threshold_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.ImageGeom.threshold)
}

// optional double height = 4;
inline bool ImageGeom::has_height() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ImageGeom::set_has_height() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ImageGeom::clear_has_height() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void ImageGeom::clear_height() {
  height_ = 0;
  clear_has_height();
}
inline double ImageGeom::height() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageGeom.height)
  return height_;
}
inline void ImageGeom::set_height(double value) {
  set_has_height();
  height_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.ImageGeom.height)
}

// optional int32 granularity = 5;
inline bool ImageGeom::has_granularity() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void ImageGeom::set_has_granularity() {
  _has_bits_[0] |= 0x00000008u;
}
inline void ImageGeom::clear_has_granularity() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void ImageGeom::clear_granularity() {
  granularity_ = 0;
  clear_has_granularity();
}
inline ::google::protobuf::int32 ImageGeom::granularity() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.ImageGeom.granularity)
  return granularity_;
}
inline void ImageGeom::set_granularity(::google::protobuf::int32 value) {
  set_has_granularity();
  granularity_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.ImageGeom.granularity)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_imagegeom_2eproto
