// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: contact.proto

#include "contact.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace protobuf_joint_5fwrench_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_joint_5fwrench_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_JointWrench;
}  // namespace protobuf_joint_5fwrench_2eproto
namespace protobuf_time_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_time_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Time;
}  // namespace protobuf_time_2eproto
namespace protobuf_vector3d_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_vector3d_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Vector3d;
}  // namespace protobuf_vector3d_2eproto
namespace gazebo {
namespace msgs {
class ContactDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Contact>
      _instance;
} _Contact_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace protobuf_contact_2eproto {
static void InitDefaultsContact() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::gazebo::msgs::_Contact_default_instance_;
    new (ptr) ::gazebo::msgs::Contact();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::gazebo::msgs::Contact::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<3> scc_info_Contact =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 3, InitDefaultsContact}, {
      &protobuf_vector3d_2eproto::scc_info_Vector3d.base,
      &protobuf_joint_5fwrench_2eproto::scc_info_JointWrench.base,
      &protobuf_time_2eproto::scc_info_Time.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Contact.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, collision1_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, collision2_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, position_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, normal_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, depth_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, wrench_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, time_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Contact, world_),
  0,
  1,
  ~0u,
  ~0u,
  ~0u,
  ~0u,
  3,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 13, sizeof(::gazebo::msgs::Contact)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::gazebo::msgs::_Contact_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "contact.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\rcontact.proto\022\013gazebo.msgs\032\016vector3d.p"
      "roto\032\ntime.proto\032\022joint_wrench.proto\"\352\001\n"
      "\007Contact\022\022\n\ncollision1\030\001 \002(\t\022\022\n\ncollisio"
      "n2\030\002 \002(\t\022\'\n\010position\030\003 \003(\0132\025.gazebo.msgs"
      ".Vector3d\022%\n\006normal\030\004 \003(\0132\025.gazebo.msgs."
      "Vector3d\022\r\n\005depth\030\005 \003(\001\022(\n\006wrench\030\006 \003(\0132"
      "\030.gazebo.msgs.JointWrench\022\037\n\004time\030\007 \002(\0132"
      "\021.gazebo.msgs.Time\022\r\n\005world\030\010 \002(\t"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 313);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "contact.proto", &protobuf_RegisterTypes);
  ::protobuf_vector3d_2eproto::AddDescriptors();
  ::protobuf_time_2eproto::AddDescriptors();
  ::protobuf_joint_5fwrench_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_contact_2eproto
namespace gazebo {
namespace msgs {

// ===================================================================

void Contact::InitAsDefaultInstance() {
  ::gazebo::msgs::_Contact_default_instance_._instance.get_mutable()->time_ = const_cast< ::gazebo::msgs::Time*>(
      ::gazebo::msgs::Time::internal_default_instance());
}
void Contact::clear_position() {
  position_.Clear();
}
void Contact::clear_normal() {
  normal_.Clear();
}
void Contact::clear_wrench() {
  wrench_.Clear();
}
void Contact::clear_time() {
  if (time_ != NULL) time_->Clear();
  clear_has_time();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Contact::kCollision1FieldNumber;
const int Contact::kCollision2FieldNumber;
const int Contact::kPositionFieldNumber;
const int Contact::kNormalFieldNumber;
const int Contact::kDepthFieldNumber;
const int Contact::kWrenchFieldNumber;
const int Contact::kTimeFieldNumber;
const int Contact::kWorldFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Contact::Contact()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_contact_2eproto::scc_info_Contact.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:gazebo.msgs.Contact)
}
Contact::Contact(const Contact& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      position_(from.position_),
      normal_(from.normal_),
      depth_(from.depth_),
      wrench_(from.wrench_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  collision1_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_collision1()) {
    collision1_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.collision1_);
  }
  collision2_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_collision2()) {
    collision2_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.collision2_);
  }
  world_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_world()) {
    world_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.world_);
  }
  if (from.has_time()) {
    time_ = new ::gazebo::msgs::Time(*from.time_);
  } else {
    time_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:gazebo.msgs.Contact)
}

void Contact::SharedCtor() {
  collision1_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  collision2_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  world_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  time_ = NULL;
}

Contact::~Contact() {
  // @@protoc_insertion_point(destructor:gazebo.msgs.Contact)
  SharedDtor();
}

void Contact::SharedDtor() {
  collision1_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  collision2_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  world_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete time_;
}

void Contact::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Contact::descriptor() {
  ::protobuf_contact_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_contact_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Contact& Contact::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_contact_2eproto::scc_info_Contact.base);
  return *internal_default_instance();
}


void Contact::Clear() {
// @@protoc_insertion_point(message_clear_start:gazebo.msgs.Contact)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  position_.Clear();
  normal_.Clear();
  depth_.Clear();
  wrench_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      collision1_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      collision2_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000004u) {
      world_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(time_ != NULL);
      time_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Contact::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gazebo.msgs.Contact)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required string collision1 = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_collision1()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->collision1().data(), static_cast<int>(this->collision1().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gazebo.msgs.Contact.collision1");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string collision2 = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_collision2()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->collision2().data(), static_cast<int>(this->collision2().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gazebo.msgs.Contact.collision2");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.Vector3d position = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_position()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.Vector3d normal = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_normal()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated double depth = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(41u /* 41 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 41u, input, this->mutable_depth())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_depth())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.JointWrench wrench = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_wrench()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required .gazebo.msgs.Time time = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(58u /* 58 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_time()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required string world = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(66u /* 66 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_world()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->world().data(), static_cast<int>(this->world().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "gazebo.msgs.Contact.world");
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:gazebo.msgs.Contact)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gazebo.msgs.Contact)
  return false;
#undef DO_
}

void Contact::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gazebo.msgs.Contact)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string collision1 = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->collision1().data(), static_cast<int>(this->collision1().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.collision1");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->collision1(), output);
  }

  // required string collision2 = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->collision2().data(), static_cast<int>(this->collision2().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.collision2");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->collision2(), output);
  }

  // repeated .gazebo.msgs.Vector3d position = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->position_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3,
      this->position(static_cast<int>(i)),
      output);
  }

  // repeated .gazebo.msgs.Vector3d normal = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->normal_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4,
      this->normal(static_cast<int>(i)),
      output);
  }

  // repeated double depth = 5;
  for (int i = 0, n = this->depth_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      5, this->depth(i), output);
  }

  // repeated .gazebo.msgs.JointWrench wrench = 6;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->wrench_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      6,
      this->wrench(static_cast<int>(i)),
      output);
  }

  // required .gazebo.msgs.Time time = 7;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      7, this->_internal_time(), output);
  }

  // required string world = 8;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->world().data(), static_cast<int>(this->world().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.world");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      8, this->world(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gazebo.msgs.Contact)
}

::google::protobuf::uint8* Contact::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gazebo.msgs.Contact)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string collision1 = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->collision1().data(), static_cast<int>(this->collision1().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.collision1");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        1, this->collision1(), target);
  }

  // required string collision2 = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->collision2().data(), static_cast<int>(this->collision2().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.collision2");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        2, this->collision2(), target);
  }

  // repeated .gazebo.msgs.Vector3d position = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->position_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->position(static_cast<int>(i)), deterministic, target);
  }

  // repeated .gazebo.msgs.Vector3d normal = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->normal_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->normal(static_cast<int>(i)), deterministic, target);
  }

  // repeated double depth = 5;
  target = ::google::protobuf::internal::WireFormatLite::
    WriteDoubleToArray(5, this->depth_, target);

  // repeated .gazebo.msgs.JointWrench wrench = 6;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->wrench_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        6, this->wrench(static_cast<int>(i)), deterministic, target);
  }

  // required .gazebo.msgs.Time time = 7;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        7, this->_internal_time(), deterministic, target);
  }

  // required string world = 8;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->world().data(), static_cast<int>(this->world().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "gazebo.msgs.Contact.world");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        8, this->world(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gazebo.msgs.Contact)
  return target;
}

size_t Contact::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:gazebo.msgs.Contact)
  size_t total_size = 0;

  if (has_collision1()) {
    // required string collision1 = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->collision1());
  }

  if (has_collision2()) {
    // required string collision2 = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->collision2());
  }

  if (has_world()) {
    // required string world = 8;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->world());
  }

  if (has_time()) {
    // required .gazebo.msgs.Time time = 7;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *time_);
  }

  return total_size;
}
size_t Contact::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gazebo.msgs.Contact)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x0000000f) ^ 0x0000000f) == 0) {  // All required fields are present.
    // required string collision1 = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->collision1());

    // required string collision2 = 2;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->collision2());

    // required string world = 8;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::StringSize(
        this->world());

    // required .gazebo.msgs.Time time = 7;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *time_);

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  // repeated .gazebo.msgs.Vector3d position = 3;
  {
    unsigned int count = static_cast<unsigned int>(this->position_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->position(static_cast<int>(i)));
    }
  }

  // repeated .gazebo.msgs.Vector3d normal = 4;
  {
    unsigned int count = static_cast<unsigned int>(this->normal_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->normal(static_cast<int>(i)));
    }
  }

  // repeated double depth = 5;
  {
    unsigned int count = static_cast<unsigned int>(this->depth_size());
    size_t data_size = 8UL * count;
    total_size += 1 *
                  ::google::protobuf::internal::FromIntSize(this->depth_size());
    total_size += data_size;
  }

  // repeated .gazebo.msgs.JointWrench wrench = 6;
  {
    unsigned int count = static_cast<unsigned int>(this->wrench_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->wrench(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Contact::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gazebo.msgs.Contact)
  GOOGLE_DCHECK_NE(&from, this);
  const Contact* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Contact>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gazebo.msgs.Contact)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gazebo.msgs.Contact)
    MergeFrom(*source);
  }
}

void Contact::MergeFrom(const Contact& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gazebo.msgs.Contact)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  position_.MergeFrom(from.position_);
  normal_.MergeFrom(from.normal_);
  depth_.MergeFrom(from.depth_);
  wrench_.MergeFrom(from.wrench_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 15u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_collision1();
      collision1_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.collision1_);
    }
    if (cached_has_bits & 0x00000002u) {
      set_has_collision2();
      collision2_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.collision2_);
    }
    if (cached_has_bits & 0x00000004u) {
      set_has_world();
      world_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.world_);
    }
    if (cached_has_bits & 0x00000008u) {
      mutable_time()->::gazebo::msgs::Time::MergeFrom(from.time());
    }
  }
}

void Contact::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gazebo.msgs.Contact)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Contact::CopyFrom(const Contact& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gazebo.msgs.Contact)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Contact::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000f) != 0x0000000f) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->position())) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->normal())) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->wrench())) return false;
  if (has_time()) {
    if (!this->time_->IsInitialized()) return false;
  }
  return true;
}

void Contact::Swap(Contact* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Contact::InternalSwap(Contact* other) {
  using std::swap;
  CastToBase(&position_)->InternalSwap(CastToBase(&other->position_));
  CastToBase(&normal_)->InternalSwap(CastToBase(&other->normal_));
  depth_.InternalSwap(&other->depth_);
  CastToBase(&wrench_)->InternalSwap(CastToBase(&other->wrench_));
  collision1_.Swap(&other->collision1_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  collision2_.Swap(&other->collision2_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  world_.Swap(&other->world_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(time_, other->time_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Contact::GetMetadata() const {
  protobuf_contact_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_contact_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::gazebo::msgs::Contact* Arena::CreateMaybeMessage< ::gazebo::msgs::Contact >(Arena* arena) {
  return Arena::CreateInternal< ::gazebo::msgs::Contact >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
