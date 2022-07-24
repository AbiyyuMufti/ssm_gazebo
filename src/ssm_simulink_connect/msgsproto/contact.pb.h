// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: contact.proto

#ifndef PROTOBUF_INCLUDED_contact_2eproto
#define PROTOBUF_INCLUDED_contact_2eproto

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
#include "vector3d.pb.h"
#include "time.pb.h"
#include "joint_wrench.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_contact_2eproto 

namespace protobuf_contact_2eproto {
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
}  // namespace protobuf_contact_2eproto
namespace gazebo {
namespace msgs {
class Contact;
class ContactDefaultTypeInternal;
extern ContactDefaultTypeInternal _Contact_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::Contact* Arena::CreateMaybeMessage<::gazebo::msgs::Contact>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

// ===================================================================

class Contact : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Contact) */ {
 public:
  Contact();
  virtual ~Contact();

  Contact(const Contact& from);

  inline Contact& operator=(const Contact& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Contact(Contact&& from) noexcept
    : Contact() {
    *this = ::std::move(from);
  }

  inline Contact& operator=(Contact&& from) noexcept {
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
  static const Contact& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Contact* internal_default_instance() {
    return reinterpret_cast<const Contact*>(
               &_Contact_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Contact* other);
  friend void swap(Contact& a, Contact& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Contact* New() const final {
    return CreateMaybeMessage<Contact>(NULL);
  }

  Contact* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Contact>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Contact& from);
  void MergeFrom(const Contact& from);
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
  void InternalSwap(Contact* other);
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

  // repeated .gazebo.msgs.Vector3d position = 3;
  int position_size() const;
  void clear_position();
  static const int kPositionFieldNumber = 3;
  ::gazebo::msgs::Vector3d* mutable_position(int index);
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_position();
  const ::gazebo::msgs::Vector3d& position(int index) const;
  ::gazebo::msgs::Vector3d* add_position();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      position() const;

  // repeated .gazebo.msgs.Vector3d normal = 4;
  int normal_size() const;
  void clear_normal();
  static const int kNormalFieldNumber = 4;
  ::gazebo::msgs::Vector3d* mutable_normal(int index);
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
      mutable_normal();
  const ::gazebo::msgs::Vector3d& normal(int index) const;
  ::gazebo::msgs::Vector3d* add_normal();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
      normal() const;

  // repeated double depth = 5;
  int depth_size() const;
  void clear_depth();
  static const int kDepthFieldNumber = 5;
  double depth(int index) const;
  void set_depth(int index, double value);
  void add_depth(double value);
  const ::google::protobuf::RepeatedField< double >&
      depth() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_depth();

  // repeated .gazebo.msgs.JointWrench wrench = 6;
  int wrench_size() const;
  void clear_wrench();
  static const int kWrenchFieldNumber = 6;
  ::gazebo::msgs::JointWrench* mutable_wrench(int index);
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::JointWrench >*
      mutable_wrench();
  const ::gazebo::msgs::JointWrench& wrench(int index) const;
  ::gazebo::msgs::JointWrench* add_wrench();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::JointWrench >&
      wrench() const;

  // required string collision1 = 1;
  bool has_collision1() const;
  void clear_collision1();
  static const int kCollision1FieldNumber = 1;
  const ::std::string& collision1() const;
  void set_collision1(const ::std::string& value);
  #if LANG_CXX11
  void set_collision1(::std::string&& value);
  #endif
  void set_collision1(const char* value);
  void set_collision1(const char* value, size_t size);
  ::std::string* mutable_collision1();
  ::std::string* release_collision1();
  void set_allocated_collision1(::std::string* collision1);

  // required string collision2 = 2;
  bool has_collision2() const;
  void clear_collision2();
  static const int kCollision2FieldNumber = 2;
  const ::std::string& collision2() const;
  void set_collision2(const ::std::string& value);
  #if LANG_CXX11
  void set_collision2(::std::string&& value);
  #endif
  void set_collision2(const char* value);
  void set_collision2(const char* value, size_t size);
  ::std::string* mutable_collision2();
  ::std::string* release_collision2();
  void set_allocated_collision2(::std::string* collision2);

  // required string world = 8;
  bool has_world() const;
  void clear_world();
  static const int kWorldFieldNumber = 8;
  const ::std::string& world() const;
  void set_world(const ::std::string& value);
  #if LANG_CXX11
  void set_world(::std::string&& value);
  #endif
  void set_world(const char* value);
  void set_world(const char* value, size_t size);
  ::std::string* mutable_world();
  ::std::string* release_world();
  void set_allocated_world(::std::string* world);

  // required .gazebo.msgs.Time time = 7;
  bool has_time() const;
  void clear_time();
  static const int kTimeFieldNumber = 7;
  private:
  const ::gazebo::msgs::Time& _internal_time() const;
  public:
  const ::gazebo::msgs::Time& time() const;
  ::gazebo::msgs::Time* release_time();
  ::gazebo::msgs::Time* mutable_time();
  void set_allocated_time(::gazebo::msgs::Time* time);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Contact)
 private:
  void set_has_collision1();
  void clear_has_collision1();
  void set_has_collision2();
  void clear_has_collision2();
  void set_has_time();
  void clear_has_time();
  void set_has_world();
  void clear_has_world();

  // helper for ByteSizeLong()
  size_t RequiredFieldsByteSizeFallback() const;

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > position_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d > normal_;
  ::google::protobuf::RepeatedField< double > depth_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::JointWrench > wrench_;
  ::google::protobuf::internal::ArenaStringPtr collision1_;
  ::google::protobuf::internal::ArenaStringPtr collision2_;
  ::google::protobuf::internal::ArenaStringPtr world_;
  ::gazebo::msgs::Time* time_;
  friend struct ::protobuf_contact_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Contact

// required string collision1 = 1;
inline bool Contact::has_collision1() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Contact::set_has_collision1() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Contact::clear_has_collision1() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Contact::clear_collision1() {
  collision1_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_collision1();
}
inline const ::std::string& Contact::collision1() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.collision1)
  return collision1_.GetNoArena();
}
inline void Contact::set_collision1(const ::std::string& value) {
  set_has_collision1();
  collision1_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Contact.collision1)
}
#if LANG_CXX11
inline void Contact::set_collision1(::std::string&& value) {
  set_has_collision1();
  collision1_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.Contact.collision1)
}
#endif
inline void Contact::set_collision1(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_collision1();
  collision1_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Contact.collision1)
}
inline void Contact::set_collision1(const char* value, size_t size) {
  set_has_collision1();
  collision1_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Contact.collision1)
}
inline ::std::string* Contact::mutable_collision1() {
  set_has_collision1();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.collision1)
  return collision1_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Contact::release_collision1() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Contact.collision1)
  if (!has_collision1()) {
    return NULL;
  }
  clear_has_collision1();
  return collision1_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Contact::set_allocated_collision1(::std::string* collision1) {
  if (collision1 != NULL) {
    set_has_collision1();
  } else {
    clear_has_collision1();
  }
  collision1_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), collision1);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Contact.collision1)
}

// required string collision2 = 2;
inline bool Contact::has_collision2() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Contact::set_has_collision2() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Contact::clear_has_collision2() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Contact::clear_collision2() {
  collision2_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_collision2();
}
inline const ::std::string& Contact::collision2() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.collision2)
  return collision2_.GetNoArena();
}
inline void Contact::set_collision2(const ::std::string& value) {
  set_has_collision2();
  collision2_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Contact.collision2)
}
#if LANG_CXX11
inline void Contact::set_collision2(::std::string&& value) {
  set_has_collision2();
  collision2_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.Contact.collision2)
}
#endif
inline void Contact::set_collision2(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_collision2();
  collision2_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Contact.collision2)
}
inline void Contact::set_collision2(const char* value, size_t size) {
  set_has_collision2();
  collision2_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Contact.collision2)
}
inline ::std::string* Contact::mutable_collision2() {
  set_has_collision2();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.collision2)
  return collision2_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Contact::release_collision2() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Contact.collision2)
  if (!has_collision2()) {
    return NULL;
  }
  clear_has_collision2();
  return collision2_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Contact::set_allocated_collision2(::std::string* collision2) {
  if (collision2 != NULL) {
    set_has_collision2();
  } else {
    clear_has_collision2();
  }
  collision2_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), collision2);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Contact.collision2)
}

// repeated .gazebo.msgs.Vector3d position = 3;
inline int Contact::position_size() const {
  return position_.size();
}
inline ::gazebo::msgs::Vector3d* Contact::mutable_position(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.position)
  return position_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
Contact::mutable_position() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Contact.position)
  return &position_;
}
inline const ::gazebo::msgs::Vector3d& Contact::position(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.position)
  return position_.Get(index);
}
inline ::gazebo::msgs::Vector3d* Contact::add_position() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Contact.position)
  return position_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
Contact::position() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Contact.position)
  return position_;
}

// repeated .gazebo.msgs.Vector3d normal = 4;
inline int Contact::normal_size() const {
  return normal_.size();
}
inline ::gazebo::msgs::Vector3d* Contact::mutable_normal(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.normal)
  return normal_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >*
Contact::mutable_normal() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Contact.normal)
  return &normal_;
}
inline const ::gazebo::msgs::Vector3d& Contact::normal(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.normal)
  return normal_.Get(index);
}
inline ::gazebo::msgs::Vector3d* Contact::add_normal() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Contact.normal)
  return normal_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Vector3d >&
Contact::normal() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Contact.normal)
  return normal_;
}

// repeated double depth = 5;
inline int Contact::depth_size() const {
  return depth_.size();
}
inline void Contact::clear_depth() {
  depth_.Clear();
}
inline double Contact::depth(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.depth)
  return depth_.Get(index);
}
inline void Contact::set_depth(int index, double value) {
  depth_.Set(index, value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Contact.depth)
}
inline void Contact::add_depth(double value) {
  depth_.Add(value);
  // @@protoc_insertion_point(field_add:gazebo.msgs.Contact.depth)
}
inline const ::google::protobuf::RepeatedField< double >&
Contact::depth() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Contact.depth)
  return depth_;
}
inline ::google::protobuf::RepeatedField< double >*
Contact::mutable_depth() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Contact.depth)
  return &depth_;
}

// repeated .gazebo.msgs.JointWrench wrench = 6;
inline int Contact::wrench_size() const {
  return wrench_.size();
}
inline ::gazebo::msgs::JointWrench* Contact::mutable_wrench(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.wrench)
  return wrench_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::JointWrench >*
Contact::mutable_wrench() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Contact.wrench)
  return &wrench_;
}
inline const ::gazebo::msgs::JointWrench& Contact::wrench(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.wrench)
  return wrench_.Get(index);
}
inline ::gazebo::msgs::JointWrench* Contact::add_wrench() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Contact.wrench)
  return wrench_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::JointWrench >&
Contact::wrench() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Contact.wrench)
  return wrench_;
}

// required .gazebo.msgs.Time time = 7;
inline bool Contact::has_time() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Contact::set_has_time() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Contact::clear_has_time() {
  _has_bits_[0] &= ~0x00000008u;
}
inline const ::gazebo::msgs::Time& Contact::_internal_time() const {
  return *time_;
}
inline const ::gazebo::msgs::Time& Contact::time() const {
  const ::gazebo::msgs::Time* p = time_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.time)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Time*>(
      &::gazebo::msgs::_Time_default_instance_);
}
inline ::gazebo::msgs::Time* Contact::release_time() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Contact.time)
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Time* Contact::mutable_time() {
  set_has_time();
  if (time_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Time>(GetArenaNoVirtual());
    time_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.time)
  return time_;
}
inline void Contact::set_allocated_time(::gazebo::msgs::Time* time) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(time_);
  }
  if (time) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      time = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, time, submessage_arena);
    }
    set_has_time();
  } else {
    clear_has_time();
  }
  time_ = time;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Contact.time)
}

// required string world = 8;
inline bool Contact::has_world() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Contact::set_has_world() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Contact::clear_has_world() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Contact::clear_world() {
  world_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_world();
}
inline const ::std::string& Contact::world() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contact.world)
  return world_.GetNoArena();
}
inline void Contact::set_world(const ::std::string& value) {
  set_has_world();
  world_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Contact.world)
}
#if LANG_CXX11
inline void Contact::set_world(::std::string&& value) {
  set_has_world();
  world_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.Contact.world)
}
#endif
inline void Contact::set_world(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_world();
  world_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Contact.world)
}
inline void Contact::set_world(const char* value, size_t size) {
  set_has_world();
  world_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Contact.world)
}
inline ::std::string* Contact::mutable_world() {
  set_has_world();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contact.world)
  return world_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Contact::release_world() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Contact.world)
  if (!has_world()) {
    return NULL;
  }
  clear_has_world();
  return world_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Contact::set_allocated_world(::std::string* world) {
  if (world != NULL) {
    set_has_world();
  } else {
    clear_has_world();
  }
  world_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), world);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Contact.world)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_contact_2eproto