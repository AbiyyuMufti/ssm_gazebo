// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: images_stamped.proto

#include "images_stamped.pb.h"

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

namespace protobuf_image_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_image_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Image;
}  // namespace protobuf_image_2eproto
namespace protobuf_time_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_time_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Time;
}  // namespace protobuf_time_2eproto
namespace gazebo {
namespace msgs {
class ImagesStampedDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<ImagesStamped>
      _instance;
} _ImagesStamped_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace protobuf_images_5fstamped_2eproto {
static void InitDefaultsImagesStamped() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::gazebo::msgs::_ImagesStamped_default_instance_;
    new (ptr) ::gazebo::msgs::ImagesStamped();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::gazebo::msgs::ImagesStamped::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<2> scc_info_ImagesStamped =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsImagesStamped}, {
      &protobuf_time_2eproto::scc_info_Time.base,
      &protobuf_image_2eproto::scc_info_Image.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_ImagesStamped.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::ImagesStamped, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::ImagesStamped, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::ImagesStamped, time_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::ImagesStamped, image_),
  0,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::gazebo::msgs::ImagesStamped)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::gazebo::msgs::_ImagesStamped_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "images_stamped.proto", schemas, file_default_instances, TableStruct::offsets,
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
      "\n\024images_stamped.proto\022\013gazebo.msgs\032\ntim"
      "e.proto\032\013image.proto\"S\n\rImagesStamped\022\037\n"
      "\004time\030\001 \002(\0132\021.gazebo.msgs.Time\022!\n\005image\030"
      "\002 \003(\0132\022.gazebo.msgs.Image"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 145);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "images_stamped.proto", &protobuf_RegisterTypes);
  ::protobuf_time_2eproto::AddDescriptors();
  ::protobuf_image_2eproto::AddDescriptors();
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
}  // namespace protobuf_images_5fstamped_2eproto
namespace gazebo {
namespace msgs {

// ===================================================================

void ImagesStamped::InitAsDefaultInstance() {
  ::gazebo::msgs::_ImagesStamped_default_instance_._instance.get_mutable()->time_ = const_cast< ::gazebo::msgs::Time*>(
      ::gazebo::msgs::Time::internal_default_instance());
}
void ImagesStamped::clear_time() {
  if (time_ != NULL) time_->Clear();
  clear_has_time();
}
void ImagesStamped::clear_image() {
  image_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ImagesStamped::kTimeFieldNumber;
const int ImagesStamped::kImageFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ImagesStamped::ImagesStamped()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_images_5fstamped_2eproto::scc_info_ImagesStamped.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:gazebo.msgs.ImagesStamped)
}
ImagesStamped::ImagesStamped(const ImagesStamped& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      image_(from.image_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_time()) {
    time_ = new ::gazebo::msgs::Time(*from.time_);
  } else {
    time_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:gazebo.msgs.ImagesStamped)
}

void ImagesStamped::SharedCtor() {
  time_ = NULL;
}

ImagesStamped::~ImagesStamped() {
  // @@protoc_insertion_point(destructor:gazebo.msgs.ImagesStamped)
  SharedDtor();
}

void ImagesStamped::SharedDtor() {
  if (this != internal_default_instance()) delete time_;
}

void ImagesStamped::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* ImagesStamped::descriptor() {
  ::protobuf_images_5fstamped_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_images_5fstamped_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ImagesStamped& ImagesStamped::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_images_5fstamped_2eproto::scc_info_ImagesStamped.base);
  return *internal_default_instance();
}


void ImagesStamped::Clear() {
// @@protoc_insertion_point(message_clear_start:gazebo.msgs.ImagesStamped)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  image_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    GOOGLE_DCHECK(time_ != NULL);
    time_->Clear();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool ImagesStamped::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gazebo.msgs.ImagesStamped)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required .gazebo.msgs.Time time = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_time()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.Image image = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_image()));
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
  // @@protoc_insertion_point(parse_success:gazebo.msgs.ImagesStamped)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gazebo.msgs.ImagesStamped)
  return false;
#undef DO_
}

void ImagesStamped::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gazebo.msgs.ImagesStamped)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gazebo.msgs.Time time = 1;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->_internal_time(), output);
  }

  // repeated .gazebo.msgs.Image image = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->image_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2,
      this->image(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gazebo.msgs.ImagesStamped)
}

::google::protobuf::uint8* ImagesStamped::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gazebo.msgs.ImagesStamped)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required .gazebo.msgs.Time time = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->_internal_time(), deterministic, target);
  }

  // repeated .gazebo.msgs.Image image = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->image_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->image(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gazebo.msgs.ImagesStamped)
  return target;
}

size_t ImagesStamped::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gazebo.msgs.ImagesStamped)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // required .gazebo.msgs.Time time = 1;
  if (has_time()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *time_);
  }
  // repeated .gazebo.msgs.Image image = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->image_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->image(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ImagesStamped::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gazebo.msgs.ImagesStamped)
  GOOGLE_DCHECK_NE(&from, this);
  const ImagesStamped* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ImagesStamped>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gazebo.msgs.ImagesStamped)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gazebo.msgs.ImagesStamped)
    MergeFrom(*source);
  }
}

void ImagesStamped::MergeFrom(const ImagesStamped& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gazebo.msgs.ImagesStamped)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  image_.MergeFrom(from.image_);
  if (from.has_time()) {
    mutable_time()->::gazebo::msgs::Time::MergeFrom(from.time());
  }
}

void ImagesStamped::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gazebo.msgs.ImagesStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ImagesStamped::CopyFrom(const ImagesStamped& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gazebo.msgs.ImagesStamped)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ImagesStamped::IsInitialized() const {
  if ((_has_bits_[0] & 0x00000001) != 0x00000001) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->image())) return false;
  if (has_time()) {
    if (!this->time_->IsInitialized()) return false;
  }
  return true;
}

void ImagesStamped::Swap(ImagesStamped* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ImagesStamped::InternalSwap(ImagesStamped* other) {
  using std::swap;
  CastToBase(&image_)->InternalSwap(CastToBase(&other->image_));
  swap(time_, other->time_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata ImagesStamped::GetMetadata() const {
  protobuf_images_5fstamped_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_images_5fstamped_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::gazebo::msgs::ImagesStamped* Arena::CreateMaybeMessage< ::gazebo::msgs::ImagesStamped >(Arena* arena) {
  return Arena::CreateInternal< ::gazebo::msgs::ImagesStamped >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
