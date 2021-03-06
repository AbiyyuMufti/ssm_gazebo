// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: contacts.proto

#ifndef PROTOBUF_INCLUDED_contacts_2eproto
#define PROTOBUF_INCLUDED_contacts_2eproto

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
#include "contact.pb.h"
#include "time.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_contacts_2eproto 

namespace protobuf_contacts_2eproto {
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
}  // namespace protobuf_contacts_2eproto
namespace gazebo {
namespace msgs {
class Contacts;
class ContactsDefaultTypeInternal;
extern ContactsDefaultTypeInternal _Contacts_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::Contacts* Arena::CreateMaybeMessage<::gazebo::msgs::Contacts>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

// ===================================================================

class Contacts : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Contacts) */ {
 public:
  Contacts();
  virtual ~Contacts();

  Contacts(const Contacts& from);

  inline Contacts& operator=(const Contacts& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Contacts(Contacts&& from) noexcept
    : Contacts() {
    *this = ::std::move(from);
  }

  inline Contacts& operator=(Contacts&& from) noexcept {
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
  static const Contacts& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Contacts* internal_default_instance() {
    return reinterpret_cast<const Contacts*>(
               &_Contacts_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Contacts* other);
  friend void swap(Contacts& a, Contacts& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Contacts* New() const final {
    return CreateMaybeMessage<Contacts>(NULL);
  }

  Contacts* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Contacts>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Contacts& from);
  void MergeFrom(const Contacts& from);
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
  void InternalSwap(Contacts* other);
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

  // repeated .gazebo.msgs.Contact contact = 1;
  int contact_size() const;
  void clear_contact();
  static const int kContactFieldNumber = 1;
  ::gazebo::msgs::Contact* mutable_contact(int index);
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Contact >*
      mutable_contact();
  const ::gazebo::msgs::Contact& contact(int index) const;
  ::gazebo::msgs::Contact* add_contact();
  const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Contact >&
      contact() const;

  // required .gazebo.msgs.Time time = 2;
  bool has_time() const;
  void clear_time();
  static const int kTimeFieldNumber = 2;
  private:
  const ::gazebo::msgs::Time& _internal_time() const;
  public:
  const ::gazebo::msgs::Time& time() const;
  ::gazebo::msgs::Time* release_time();
  ::gazebo::msgs::Time* mutable_time();
  void set_allocated_time(::gazebo::msgs::Time* time);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Contacts)
 private:
  void set_has_time();
  void clear_has_time();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Contact > contact_;
  ::gazebo::msgs::Time* time_;
  friend struct ::protobuf_contacts_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Contacts

// repeated .gazebo.msgs.Contact contact = 1;
inline int Contacts::contact_size() const {
  return contact_.size();
}
inline ::gazebo::msgs::Contact* Contacts::mutable_contact(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contacts.contact)
  return contact_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Contact >*
Contacts::mutable_contact() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Contacts.contact)
  return &contact_;
}
inline const ::gazebo::msgs::Contact& Contacts::contact(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contacts.contact)
  return contact_.Get(index);
}
inline ::gazebo::msgs::Contact* Contacts::add_contact() {
  // @@protoc_insertion_point(field_add:gazebo.msgs.Contacts.contact)
  return contact_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::gazebo::msgs::Contact >&
Contacts::contact() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Contacts.contact)
  return contact_;
}

// required .gazebo.msgs.Time time = 2;
inline bool Contacts::has_time() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Contacts::set_has_time() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Contacts::clear_has_time() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::gazebo::msgs::Time& Contacts::_internal_time() const {
  return *time_;
}
inline const ::gazebo::msgs::Time& Contacts::time() const {
  const ::gazebo::msgs::Time* p = time_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Contacts.time)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Time*>(
      &::gazebo::msgs::_Time_default_instance_);
}
inline ::gazebo::msgs::Time* Contacts::release_time() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Contacts.time)
  clear_has_time();
  ::gazebo::msgs::Time* temp = time_;
  time_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Time* Contacts::mutable_time() {
  set_has_time();
  if (time_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Time>(GetArenaNoVirtual());
    time_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Contacts.time)
  return time_;
}
inline void Contacts::set_allocated_time(::gazebo::msgs::Time* time) {
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
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Contacts.time)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_contacts_2eproto
