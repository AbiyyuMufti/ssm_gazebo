// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: SphericalVector.proto

#include "SphericalVector.pb.h"

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

namespace ssm_msgs {
namespace msgs {
class SphericalVectorDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<SphericalVector>
      _instance;
} _SphericalVector_default_instance_;
}  // namespace msgs
}  // namespace ssm_msgs
namespace protobuf_SphericalVector_2eproto {
static void InitDefaultsSphericalVector() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::ssm_msgs::msgs::_SphericalVector_default_instance_;
    new (ptr) ::ssm_msgs::msgs::SphericalVector();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::ssm_msgs::msgs::SphericalVector::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_SphericalVector =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsSphericalVector}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_SphericalVector.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ssm_msgs::msgs::SphericalVector, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ssm_msgs::msgs::SphericalVector, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ssm_msgs::msgs::SphericalVector, r_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ssm_msgs::msgs::SphericalVector, theta_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::ssm_msgs::msgs::SphericalVector, phi_),
  0,
  1,
  2,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::ssm_msgs::msgs::SphericalVector)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::ssm_msgs::msgs::_SphericalVector_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "SphericalVector.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\025SphericalVector.proto\022\rssm_msgs.msgs\"8"
      "\n\017SphericalVector\022\t\n\001r\030\001 \002(\001\022\r\n\005theta\030\002 "
      "\002(\001\022\013\n\003phi\030\003 \002(\001"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 96);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "SphericalVector.proto", &protobuf_RegisterTypes);
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
}  // namespace protobuf_SphericalVector_2eproto
namespace ssm_msgs {
namespace msgs {

// ===================================================================

void SphericalVector::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int SphericalVector::kRFieldNumber;
const int SphericalVector::kThetaFieldNumber;
const int SphericalVector::kPhiFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

SphericalVector::SphericalVector()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_SphericalVector_2eproto::scc_info_SphericalVector.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:ssm_msgs.msgs.SphericalVector)
}
SphericalVector::SphericalVector(const SphericalVector& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&r_, &from.r_,
    static_cast<size_t>(reinterpret_cast<char*>(&phi_) -
    reinterpret_cast<char*>(&r_)) + sizeof(phi_));
  // @@protoc_insertion_point(copy_constructor:ssm_msgs.msgs.SphericalVector)
}

void SphericalVector::SharedCtor() {
  ::memset(&r_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&phi_) -
      reinterpret_cast<char*>(&r_)) + sizeof(phi_));
}

SphericalVector::~SphericalVector() {
  // @@protoc_insertion_point(destructor:ssm_msgs.msgs.SphericalVector)
  SharedDtor();
}

void SphericalVector::SharedDtor() {
}

void SphericalVector::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* SphericalVector::descriptor() {
  ::protobuf_SphericalVector_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_SphericalVector_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const SphericalVector& SphericalVector::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_SphericalVector_2eproto::scc_info_SphericalVector.base);
  return *internal_default_instance();
}


void SphericalVector::Clear() {
// @@protoc_insertion_point(message_clear_start:ssm_msgs.msgs.SphericalVector)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 7u) {
    ::memset(&r_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&phi_) -
        reinterpret_cast<char*>(&r_)) + sizeof(phi_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool SphericalVector::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:ssm_msgs.msgs.SphericalVector)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required double r = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_r();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &r_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double theta = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {
          set_has_theta();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // required double phi = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {
          set_has_phi();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &phi_)));
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
  // @@protoc_insertion_point(parse_success:ssm_msgs.msgs.SphericalVector)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:ssm_msgs.msgs.SphericalVector)
  return false;
#undef DO_
}

void SphericalVector::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:ssm_msgs.msgs.SphericalVector)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double r = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->r(), output);
  }

  // required double theta = 2;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->theta(), output);
  }

  // required double phi = 3;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->phi(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:ssm_msgs.msgs.SphericalVector)
}

::google::protobuf::uint8* SphericalVector::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:ssm_msgs.msgs.SphericalVector)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required double r = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->r(), target);
  }

  // required double theta = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->theta(), target);
  }

  // required double phi = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->phi(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:ssm_msgs.msgs.SphericalVector)
  return target;
}

size_t SphericalVector::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:ssm_msgs.msgs.SphericalVector)
  size_t total_size = 0;

  if (has_r()) {
    // required double r = 1;
    total_size += 1 + 8;
  }

  if (has_theta()) {
    // required double theta = 2;
    total_size += 1 + 8;
  }

  if (has_phi()) {
    // required double phi = 3;
    total_size += 1 + 8;
  }

  return total_size;
}
size_t SphericalVector::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:ssm_msgs.msgs.SphericalVector)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (((_has_bits_[0] & 0x00000007) ^ 0x00000007) == 0) {  // All required fields are present.
    // required double r = 1;
    total_size += 1 + 8;

    // required double theta = 2;
    total_size += 1 + 8;

    // required double phi = 3;
    total_size += 1 + 8;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void SphericalVector::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:ssm_msgs.msgs.SphericalVector)
  GOOGLE_DCHECK_NE(&from, this);
  const SphericalVector* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const SphericalVector>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:ssm_msgs.msgs.SphericalVector)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:ssm_msgs.msgs.SphericalVector)
    MergeFrom(*source);
  }
}

void SphericalVector::MergeFrom(const SphericalVector& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:ssm_msgs.msgs.SphericalVector)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      r_ = from.r_;
    }
    if (cached_has_bits & 0x00000002u) {
      theta_ = from.theta_;
    }
    if (cached_has_bits & 0x00000004u) {
      phi_ = from.phi_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void SphericalVector::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:ssm_msgs.msgs.SphericalVector)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SphericalVector::CopyFrom(const SphericalVector& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:ssm_msgs.msgs.SphericalVector)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SphericalVector::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000007) != 0x00000007) return false;
  return true;
}

void SphericalVector::Swap(SphericalVector* other) {
  if (other == this) return;
  InternalSwap(other);
}
void SphericalVector::InternalSwap(SphericalVector* other) {
  using std::swap;
  swap(r_, other->r_);
  swap(theta_, other->theta_);
  swap(phi_, other->phi_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata SphericalVector::GetMetadata() const {
  protobuf_SphericalVector_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_SphericalVector_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace ssm_msgs
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::ssm_msgs::msgs::SphericalVector* Arena::CreateMaybeMessage< ::ssm_msgs::msgs::SphericalVector >(Arena* arena) {
  return Arena::CreateInternal< ::ssm_msgs::msgs::SphericalVector >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
