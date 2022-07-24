// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: material.proto

#ifndef PROTOBUF_INCLUDED_material_2eproto
#define PROTOBUF_INCLUDED_material_2eproto

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "color.pb.h"
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_material_2eproto 

namespace protobuf_material_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_material_2eproto
namespace gazebo {
namespace msgs {
class Material;
class MaterialDefaultTypeInternal;
extern MaterialDefaultTypeInternal _Material_default_instance_;
class Material_Script;
class Material_ScriptDefaultTypeInternal;
extern Material_ScriptDefaultTypeInternal _Material_Script_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> ::gazebo::msgs::Material* Arena::CreateMaybeMessage<::gazebo::msgs::Material>(Arena*);
template<> ::gazebo::msgs::Material_Script* Arena::CreateMaybeMessage<::gazebo::msgs::Material_Script>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace gazebo {
namespace msgs {

enum Material_ShaderType {
  Material_ShaderType_VERTEX = 1,
  Material_ShaderType_PIXEL = 2,
  Material_ShaderType_NORMAL_MAP_OBJECT_SPACE = 3,
  Material_ShaderType_NORMAL_MAP_TANGENT_SPACE = 4
};
bool Material_ShaderType_IsValid(int value);
const Material_ShaderType Material_ShaderType_ShaderType_MIN = Material_ShaderType_VERTEX;
const Material_ShaderType Material_ShaderType_ShaderType_MAX = Material_ShaderType_NORMAL_MAP_TANGENT_SPACE;
const int Material_ShaderType_ShaderType_ARRAYSIZE = Material_ShaderType_ShaderType_MAX + 1;

const ::google::protobuf::EnumDescriptor* Material_ShaderType_descriptor();
inline const ::std::string& Material_ShaderType_Name(Material_ShaderType value) {
  return ::google::protobuf::internal::NameOfEnum(
    Material_ShaderType_descriptor(), value);
}
inline bool Material_ShaderType_Parse(
    const ::std::string& name, Material_ShaderType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<Material_ShaderType>(
    Material_ShaderType_descriptor(), name, value);
}
// ===================================================================

class Material_Script : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Material.Script) */ {
 public:
  Material_Script();
  virtual ~Material_Script();

  Material_Script(const Material_Script& from);

  inline Material_Script& operator=(const Material_Script& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Material_Script(Material_Script&& from) noexcept
    : Material_Script() {
    *this = ::std::move(from);
  }

  inline Material_Script& operator=(Material_Script&& from) noexcept {
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
  static const Material_Script& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Material_Script* internal_default_instance() {
    return reinterpret_cast<const Material_Script*>(
               &_Material_Script_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Material_Script* other);
  friend void swap(Material_Script& a, Material_Script& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Material_Script* New() const final {
    return CreateMaybeMessage<Material_Script>(NULL);
  }

  Material_Script* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Material_Script>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Material_Script& from);
  void MergeFrom(const Material_Script& from);
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
  void InternalSwap(Material_Script* other);
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

  // repeated string uri = 1;
  int uri_size() const;
  void clear_uri();
  static const int kUriFieldNumber = 1;
  const ::std::string& uri(int index) const;
  ::std::string* mutable_uri(int index);
  void set_uri(int index, const ::std::string& value);
  #if LANG_CXX11
  void set_uri(int index, ::std::string&& value);
  #endif
  void set_uri(int index, const char* value);
  void set_uri(int index, const char* value, size_t size);
  ::std::string* add_uri();
  void add_uri(const ::std::string& value);
  #if LANG_CXX11
  void add_uri(::std::string&& value);
  #endif
  void add_uri(const char* value);
  void add_uri(const char* value, size_t size);
  const ::google::protobuf::RepeatedPtrField< ::std::string>& uri() const;
  ::google::protobuf::RepeatedPtrField< ::std::string>* mutable_uri();

  // required string name = 2;
  bool has_name() const;
  void clear_name();
  static const int kNameFieldNumber = 2;
  const ::std::string& name() const;
  void set_name(const ::std::string& value);
  #if LANG_CXX11
  void set_name(::std::string&& value);
  #endif
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  ::std::string* mutable_name();
  ::std::string* release_name();
  void set_allocated_name(::std::string* name);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Material.Script)
 private:
  void set_has_name();
  void clear_has_name();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::RepeatedPtrField< ::std::string> uri_;
  ::google::protobuf::internal::ArenaStringPtr name_;
  friend struct ::protobuf_material_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Material : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gazebo.msgs.Material) */ {
 public:
  Material();
  virtual ~Material();

  Material(const Material& from);

  inline Material& operator=(const Material& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Material(Material&& from) noexcept
    : Material() {
    *this = ::std::move(from);
  }

  inline Material& operator=(Material&& from) noexcept {
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
  static const Material& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Material* internal_default_instance() {
    return reinterpret_cast<const Material*>(
               &_Material_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Material* other);
  friend void swap(Material& a, Material& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Material* New() const final {
    return CreateMaybeMessage<Material>(NULL);
  }

  Material* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Material>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Material& from);
  void MergeFrom(const Material& from);
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
  void InternalSwap(Material* other);
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

  typedef Material_Script Script;

  typedef Material_ShaderType ShaderType;
  static const ShaderType VERTEX =
    Material_ShaderType_VERTEX;
  static const ShaderType PIXEL =
    Material_ShaderType_PIXEL;
  static const ShaderType NORMAL_MAP_OBJECT_SPACE =
    Material_ShaderType_NORMAL_MAP_OBJECT_SPACE;
  static const ShaderType NORMAL_MAP_TANGENT_SPACE =
    Material_ShaderType_NORMAL_MAP_TANGENT_SPACE;
  static inline bool ShaderType_IsValid(int value) {
    return Material_ShaderType_IsValid(value);
  }
  static const ShaderType ShaderType_MIN =
    Material_ShaderType_ShaderType_MIN;
  static const ShaderType ShaderType_MAX =
    Material_ShaderType_ShaderType_MAX;
  static const int ShaderType_ARRAYSIZE =
    Material_ShaderType_ShaderType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  ShaderType_descriptor() {
    return Material_ShaderType_descriptor();
  }
  static inline const ::std::string& ShaderType_Name(ShaderType value) {
    return Material_ShaderType_Name(value);
  }
  static inline bool ShaderType_Parse(const ::std::string& name,
      ShaderType* value) {
    return Material_ShaderType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // optional string normal_map = 3;
  bool has_normal_map() const;
  void clear_normal_map();
  static const int kNormalMapFieldNumber = 3;
  const ::std::string& normal_map() const;
  void set_normal_map(const ::std::string& value);
  #if LANG_CXX11
  void set_normal_map(::std::string&& value);
  #endif
  void set_normal_map(const char* value);
  void set_normal_map(const char* value, size_t size);
  ::std::string* mutable_normal_map();
  ::std::string* release_normal_map();
  void set_allocated_normal_map(::std::string* normal_map);

  // optional .gazebo.msgs.Material.Script script = 1;
  bool has_script() const;
  void clear_script();
  static const int kScriptFieldNumber = 1;
  private:
  const ::gazebo::msgs::Material_Script& _internal_script() const;
  public:
  const ::gazebo::msgs::Material_Script& script() const;
  ::gazebo::msgs::Material_Script* release_script();
  ::gazebo::msgs::Material_Script* mutable_script();
  void set_allocated_script(::gazebo::msgs::Material_Script* script);

  // optional .gazebo.msgs.Color ambient = 4;
  bool has_ambient() const;
  void clear_ambient();
  static const int kAmbientFieldNumber = 4;
  private:
  const ::gazebo::msgs::Color& _internal_ambient() const;
  public:
  const ::gazebo::msgs::Color& ambient() const;
  ::gazebo::msgs::Color* release_ambient();
  ::gazebo::msgs::Color* mutable_ambient();
  void set_allocated_ambient(::gazebo::msgs::Color* ambient);

  // optional .gazebo.msgs.Color diffuse = 5;
  bool has_diffuse() const;
  void clear_diffuse();
  static const int kDiffuseFieldNumber = 5;
  private:
  const ::gazebo::msgs::Color& _internal_diffuse() const;
  public:
  const ::gazebo::msgs::Color& diffuse() const;
  ::gazebo::msgs::Color* release_diffuse();
  ::gazebo::msgs::Color* mutable_diffuse();
  void set_allocated_diffuse(::gazebo::msgs::Color* diffuse);

  // optional .gazebo.msgs.Color specular = 6;
  bool has_specular() const;
  void clear_specular();
  static const int kSpecularFieldNumber = 6;
  private:
  const ::gazebo::msgs::Color& _internal_specular() const;
  public:
  const ::gazebo::msgs::Color& specular() const;
  ::gazebo::msgs::Color* release_specular();
  ::gazebo::msgs::Color* mutable_specular();
  void set_allocated_specular(::gazebo::msgs::Color* specular);

  // optional .gazebo.msgs.Color emissive = 7;
  bool has_emissive() const;
  void clear_emissive();
  static const int kEmissiveFieldNumber = 7;
  private:
  const ::gazebo::msgs::Color& _internal_emissive() const;
  public:
  const ::gazebo::msgs::Color& emissive() const;
  ::gazebo::msgs::Color* release_emissive();
  ::gazebo::msgs::Color* mutable_emissive();
  void set_allocated_emissive(::gazebo::msgs::Color* emissive);

  // optional bool lighting = 8;
  bool has_lighting() const;
  void clear_lighting();
  static const int kLightingFieldNumber = 8;
  bool lighting() const;
  void set_lighting(bool value);

  // optional .gazebo.msgs.Material.ShaderType shader_type = 2;
  bool has_shader_type() const;
  void clear_shader_type();
  static const int kShaderTypeFieldNumber = 2;
  ::gazebo::msgs::Material_ShaderType shader_type() const;
  void set_shader_type(::gazebo::msgs::Material_ShaderType value);

  // @@protoc_insertion_point(class_scope:gazebo.msgs.Material)
 private:
  void set_has_script();
  void clear_has_script();
  void set_has_shader_type();
  void clear_has_shader_type();
  void set_has_normal_map();
  void clear_has_normal_map();
  void set_has_ambient();
  void clear_has_ambient();
  void set_has_diffuse();
  void clear_has_diffuse();
  void set_has_specular();
  void clear_has_specular();
  void set_has_emissive();
  void clear_has_emissive();
  void set_has_lighting();
  void clear_has_lighting();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr normal_map_;
  ::gazebo::msgs::Material_Script* script_;
  ::gazebo::msgs::Color* ambient_;
  ::gazebo::msgs::Color* diffuse_;
  ::gazebo::msgs::Color* specular_;
  ::gazebo::msgs::Color* emissive_;
  bool lighting_;
  int shader_type_;
  friend struct ::protobuf_material_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Material_Script

// repeated string uri = 1;
inline int Material_Script::uri_size() const {
  return uri_.size();
}
inline void Material_Script::clear_uri() {
  uri_.Clear();
}
inline const ::std::string& Material_Script::uri(int index) const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.Script.uri)
  return uri_.Get(index);
}
inline ::std::string* Material_Script::mutable_uri(int index) {
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.Script.uri)
  return uri_.Mutable(index);
}
inline void Material_Script::set_uri(int index, const ::std::string& value) {
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.Script.uri)
  uri_.Mutable(index)->assign(value);
}
#if LANG_CXX11
inline void Material_Script::set_uri(int index, ::std::string&& value) {
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.Script.uri)
  uri_.Mutable(index)->assign(std::move(value));
}
#endif
inline void Material_Script::set_uri(int index, const char* value) {
  GOOGLE_DCHECK(value != NULL);
  uri_.Mutable(index)->assign(value);
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Material.Script.uri)
}
inline void Material_Script::set_uri(int index, const char* value, size_t size) {
  uri_.Mutable(index)->assign(
    reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Material.Script.uri)
}
inline ::std::string* Material_Script::add_uri() {
  // @@protoc_insertion_point(field_add_mutable:gazebo.msgs.Material.Script.uri)
  return uri_.Add();
}
inline void Material_Script::add_uri(const ::std::string& value) {
  uri_.Add()->assign(value);
  // @@protoc_insertion_point(field_add:gazebo.msgs.Material.Script.uri)
}
#if LANG_CXX11
inline void Material_Script::add_uri(::std::string&& value) {
  uri_.Add(std::move(value));
  // @@protoc_insertion_point(field_add:gazebo.msgs.Material.Script.uri)
}
#endif
inline void Material_Script::add_uri(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  uri_.Add()->assign(value);
  // @@protoc_insertion_point(field_add_char:gazebo.msgs.Material.Script.uri)
}
inline void Material_Script::add_uri(const char* value, size_t size) {
  uri_.Add()->assign(reinterpret_cast<const char*>(value), size);
  // @@protoc_insertion_point(field_add_pointer:gazebo.msgs.Material.Script.uri)
}
inline const ::google::protobuf::RepeatedPtrField< ::std::string>&
Material_Script::uri() const {
  // @@protoc_insertion_point(field_list:gazebo.msgs.Material.Script.uri)
  return uri_;
}
inline ::google::protobuf::RepeatedPtrField< ::std::string>*
Material_Script::mutable_uri() {
  // @@protoc_insertion_point(field_mutable_list:gazebo.msgs.Material.Script.uri)
  return &uri_;
}

// required string name = 2;
inline bool Material_Script::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Material_Script::set_has_name() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Material_Script::clear_has_name() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Material_Script::clear_name() {
  name_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_name();
}
inline const ::std::string& Material_Script::name() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.Script.name)
  return name_.GetNoArena();
}
inline void Material_Script::set_name(const ::std::string& value) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.Script.name)
}
#if LANG_CXX11
inline void Material_Script::set_name(::std::string&& value) {
  set_has_name();
  name_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.Material.Script.name)
}
#endif
inline void Material_Script::set_name(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Material.Script.name)
}
inline void Material_Script::set_name(const char* value, size_t size) {
  set_has_name();
  name_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Material.Script.name)
}
inline ::std::string* Material_Script::mutable_name() {
  set_has_name();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.Script.name)
  return name_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Material_Script::release_name() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.Script.name)
  if (!has_name()) {
    return NULL;
  }
  clear_has_name();
  return name_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Material_Script::set_allocated_name(::std::string* name) {
  if (name != NULL) {
    set_has_name();
  } else {
    clear_has_name();
  }
  name_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.Script.name)
}

// -------------------------------------------------------------------

// Material

// optional .gazebo.msgs.Material.Script script = 1;
inline bool Material::has_script() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Material::set_has_script() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Material::clear_has_script() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Material::clear_script() {
  if (script_ != NULL) script_->Clear();
  clear_has_script();
}
inline const ::gazebo::msgs::Material_Script& Material::_internal_script() const {
  return *script_;
}
inline const ::gazebo::msgs::Material_Script& Material::script() const {
  const ::gazebo::msgs::Material_Script* p = script_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.script)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Material_Script*>(
      &::gazebo::msgs::_Material_Script_default_instance_);
}
inline ::gazebo::msgs::Material_Script* Material::release_script() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.script)
  clear_has_script();
  ::gazebo::msgs::Material_Script* temp = script_;
  script_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Material_Script* Material::mutable_script() {
  set_has_script();
  if (script_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Material_Script>(GetArenaNoVirtual());
    script_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.script)
  return script_;
}
inline void Material::set_allocated_script(::gazebo::msgs::Material_Script* script) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete script_;
  }
  if (script) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      script = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, script, submessage_arena);
    }
    set_has_script();
  } else {
    clear_has_script();
  }
  script_ = script;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.script)
}

// optional .gazebo.msgs.Material.ShaderType shader_type = 2;
inline bool Material::has_shader_type() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void Material::set_has_shader_type() {
  _has_bits_[0] |= 0x00000080u;
}
inline void Material::clear_has_shader_type() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void Material::clear_shader_type() {
  shader_type_ = 1;
  clear_has_shader_type();
}
inline ::gazebo::msgs::Material_ShaderType Material::shader_type() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.shader_type)
  return static_cast< ::gazebo::msgs::Material_ShaderType >(shader_type_);
}
inline void Material::set_shader_type(::gazebo::msgs::Material_ShaderType value) {
  assert(::gazebo::msgs::Material_ShaderType_IsValid(value));
  set_has_shader_type();
  shader_type_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.shader_type)
}

// optional string normal_map = 3;
inline bool Material::has_normal_map() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Material::set_has_normal_map() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Material::clear_has_normal_map() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Material::clear_normal_map() {
  normal_map_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_normal_map();
}
inline const ::std::string& Material::normal_map() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.normal_map)
  return normal_map_.GetNoArena();
}
inline void Material::set_normal_map(const ::std::string& value) {
  set_has_normal_map();
  normal_map_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.normal_map)
}
#if LANG_CXX11
inline void Material::set_normal_map(::std::string&& value) {
  set_has_normal_map();
  normal_map_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:gazebo.msgs.Material.normal_map)
}
#endif
inline void Material::set_normal_map(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_normal_map();
  normal_map_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:gazebo.msgs.Material.normal_map)
}
inline void Material::set_normal_map(const char* value, size_t size) {
  set_has_normal_map();
  normal_map_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:gazebo.msgs.Material.normal_map)
}
inline ::std::string* Material::mutable_normal_map() {
  set_has_normal_map();
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.normal_map)
  return normal_map_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Material::release_normal_map() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.normal_map)
  if (!has_normal_map()) {
    return NULL;
  }
  clear_has_normal_map();
  return normal_map_.ReleaseNonDefaultNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Material::set_allocated_normal_map(::std::string* normal_map) {
  if (normal_map != NULL) {
    set_has_normal_map();
  } else {
    clear_has_normal_map();
  }
  normal_map_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), normal_map);
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.normal_map)
}

// optional .gazebo.msgs.Color ambient = 4;
inline bool Material::has_ambient() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Material::set_has_ambient() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Material::clear_has_ambient() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::gazebo::msgs::Color& Material::_internal_ambient() const {
  return *ambient_;
}
inline const ::gazebo::msgs::Color& Material::ambient() const {
  const ::gazebo::msgs::Color* p = ambient_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.ambient)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Color*>(
      &::gazebo::msgs::_Color_default_instance_);
}
inline ::gazebo::msgs::Color* Material::release_ambient() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.ambient)
  clear_has_ambient();
  ::gazebo::msgs::Color* temp = ambient_;
  ambient_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Color* Material::mutable_ambient() {
  set_has_ambient();
  if (ambient_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Color>(GetArenaNoVirtual());
    ambient_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.ambient)
  return ambient_;
}
inline void Material::set_allocated_ambient(::gazebo::msgs::Color* ambient) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(ambient_);
  }
  if (ambient) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      ambient = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, ambient, submessage_arena);
    }
    set_has_ambient();
  } else {
    clear_has_ambient();
  }
  ambient_ = ambient;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.ambient)
}

// optional .gazebo.msgs.Color diffuse = 5;
inline bool Material::has_diffuse() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Material::set_has_diffuse() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Material::clear_has_diffuse() {
  _has_bits_[0] &= ~0x00000008u;
}
inline const ::gazebo::msgs::Color& Material::_internal_diffuse() const {
  return *diffuse_;
}
inline const ::gazebo::msgs::Color& Material::diffuse() const {
  const ::gazebo::msgs::Color* p = diffuse_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.diffuse)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Color*>(
      &::gazebo::msgs::_Color_default_instance_);
}
inline ::gazebo::msgs::Color* Material::release_diffuse() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.diffuse)
  clear_has_diffuse();
  ::gazebo::msgs::Color* temp = diffuse_;
  diffuse_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Color* Material::mutable_diffuse() {
  set_has_diffuse();
  if (diffuse_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Color>(GetArenaNoVirtual());
    diffuse_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.diffuse)
  return diffuse_;
}
inline void Material::set_allocated_diffuse(::gazebo::msgs::Color* diffuse) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(diffuse_);
  }
  if (diffuse) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      diffuse = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, diffuse, submessage_arena);
    }
    set_has_diffuse();
  } else {
    clear_has_diffuse();
  }
  diffuse_ = diffuse;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.diffuse)
}

// optional .gazebo.msgs.Color specular = 6;
inline bool Material::has_specular() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Material::set_has_specular() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Material::clear_has_specular() {
  _has_bits_[0] &= ~0x00000010u;
}
inline const ::gazebo::msgs::Color& Material::_internal_specular() const {
  return *specular_;
}
inline const ::gazebo::msgs::Color& Material::specular() const {
  const ::gazebo::msgs::Color* p = specular_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.specular)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Color*>(
      &::gazebo::msgs::_Color_default_instance_);
}
inline ::gazebo::msgs::Color* Material::release_specular() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.specular)
  clear_has_specular();
  ::gazebo::msgs::Color* temp = specular_;
  specular_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Color* Material::mutable_specular() {
  set_has_specular();
  if (specular_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Color>(GetArenaNoVirtual());
    specular_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.specular)
  return specular_;
}
inline void Material::set_allocated_specular(::gazebo::msgs::Color* specular) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(specular_);
  }
  if (specular) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      specular = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, specular, submessage_arena);
    }
    set_has_specular();
  } else {
    clear_has_specular();
  }
  specular_ = specular;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.specular)
}

// optional .gazebo.msgs.Color emissive = 7;
inline bool Material::has_emissive() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Material::set_has_emissive() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Material::clear_has_emissive() {
  _has_bits_[0] &= ~0x00000020u;
}
inline const ::gazebo::msgs::Color& Material::_internal_emissive() const {
  return *emissive_;
}
inline const ::gazebo::msgs::Color& Material::emissive() const {
  const ::gazebo::msgs::Color* p = emissive_;
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.emissive)
  return p != NULL ? *p : *reinterpret_cast<const ::gazebo::msgs::Color*>(
      &::gazebo::msgs::_Color_default_instance_);
}
inline ::gazebo::msgs::Color* Material::release_emissive() {
  // @@protoc_insertion_point(field_release:gazebo.msgs.Material.emissive)
  clear_has_emissive();
  ::gazebo::msgs::Color* temp = emissive_;
  emissive_ = NULL;
  return temp;
}
inline ::gazebo::msgs::Color* Material::mutable_emissive() {
  set_has_emissive();
  if (emissive_ == NULL) {
    auto* p = CreateMaybeMessage<::gazebo::msgs::Color>(GetArenaNoVirtual());
    emissive_ = p;
  }
  // @@protoc_insertion_point(field_mutable:gazebo.msgs.Material.emissive)
  return emissive_;
}
inline void Material::set_allocated_emissive(::gazebo::msgs::Color* emissive) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(emissive_);
  }
  if (emissive) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      emissive = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, emissive, submessage_arena);
    }
    set_has_emissive();
  } else {
    clear_has_emissive();
  }
  emissive_ = emissive;
  // @@protoc_insertion_point(field_set_allocated:gazebo.msgs.Material.emissive)
}

// optional bool lighting = 8;
inline bool Material::has_lighting() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Material::set_has_lighting() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Material::clear_has_lighting() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Material::clear_lighting() {
  lighting_ = false;
  clear_has_lighting();
}
inline bool Material::lighting() const {
  // @@protoc_insertion_point(field_get:gazebo.msgs.Material.lighting)
  return lighting_;
}
inline void Material::set_lighting(bool value) {
  set_has_lighting();
  lighting_ = value;
  // @@protoc_insertion_point(field_set:gazebo.msgs.Material.lighting)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace gazebo

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::gazebo::msgs::Material_ShaderType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::gazebo::msgs::Material_ShaderType>() {
  return ::gazebo::msgs::Material_ShaderType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_material_2eproto