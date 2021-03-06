// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: geometry.proto

#include "geometry.pb.h"

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

namespace protobuf_boxgeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_boxgeom_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_BoxGeom;
}  // namespace protobuf_boxgeom_2eproto
namespace protobuf_cylindergeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_cylindergeom_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_CylinderGeom;
}  // namespace protobuf_cylindergeom_2eproto
namespace protobuf_heightmapgeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_heightmapgeom_2eproto ::google::protobuf::internal::SCCInfo<4> scc_info_HeightmapGeom;
}  // namespace protobuf_heightmapgeom_2eproto
namespace protobuf_imagegeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_imagegeom_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_ImageGeom;
}  // namespace protobuf_imagegeom_2eproto
namespace protobuf_meshgeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_meshgeom_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_MeshGeom;
}  // namespace protobuf_meshgeom_2eproto
namespace protobuf_planegeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_planegeom_2eproto ::google::protobuf::internal::SCCInfo<2> scc_info_PlaneGeom;
}  // namespace protobuf_planegeom_2eproto
namespace protobuf_polylinegeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_polylinegeom_2eproto ::google::protobuf::internal::SCCInfo<1> scc_info_Polyline;
}  // namespace protobuf_polylinegeom_2eproto
namespace protobuf_spheregeom_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_spheregeom_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_SphereGeom;
}  // namespace protobuf_spheregeom_2eproto
namespace protobuf_vector3d_2eproto {
extern PROTOBUF_INTERNAL_EXPORT_protobuf_vector3d_2eproto ::google::protobuf::internal::SCCInfo<0> scc_info_Vector3d;
}  // namespace protobuf_vector3d_2eproto
namespace gazebo {
namespace msgs {
class GeometryDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<Geometry>
      _instance;
} _Geometry_default_instance_;
}  // namespace msgs
}  // namespace gazebo
namespace protobuf_geometry_2eproto {
static void InitDefaultsGeometry() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::gazebo::msgs::_Geometry_default_instance_;
    new (ptr) ::gazebo::msgs::Geometry();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::gazebo::msgs::Geometry::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<9> scc_info_Geometry =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 9, InitDefaultsGeometry}, {
      &protobuf_boxgeom_2eproto::scc_info_BoxGeom.base,
      &protobuf_cylindergeom_2eproto::scc_info_CylinderGeom.base,
      &protobuf_planegeom_2eproto::scc_info_PlaneGeom.base,
      &protobuf_spheregeom_2eproto::scc_info_SphereGeom.base,
      &protobuf_imagegeom_2eproto::scc_info_ImageGeom.base,
      &protobuf_heightmapgeom_2eproto::scc_info_HeightmapGeom.base,
      &protobuf_meshgeom_2eproto::scc_info_MeshGeom.base,
      &protobuf_vector3d_2eproto::scc_info_Vector3d.base,
      &protobuf_polylinegeom_2eproto::scc_info_Polyline.base,}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_Geometry.base);
}

::google::protobuf::Metadata file_level_metadata[1];
const ::google::protobuf::EnumDescriptor* file_level_enum_descriptors[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, type_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, box_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, cylinder_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, plane_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, sphere_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, image_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, heightmap_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, mesh_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, points_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::gazebo::msgs::Geometry, polyline_),
  7,
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  ~0u,
  ~0u,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 15, sizeof(::gazebo::msgs::Geometry)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::gazebo::msgs::_Geometry_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "geometry.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, file_level_enum_descriptors, NULL);
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
      "\n\016geometry.proto\022\013gazebo.msgs\032\rboxgeom.p"
      "roto\032\022cylindergeom.proto\032\020spheregeom.pro"
      "to\032\017planegeom.proto\032\017imagegeom.proto\032\023he"
      "ightmapgeom.proto\032\016meshgeom.proto\032\016vecto"
      "r3d.proto\032\022polylinegeom.proto\"\265\004\n\010Geomet"
      "ry\022(\n\004type\030\001 \001(\0162\032.gazebo.msgs.Geometry."
      "Type\022!\n\003box\030\002 \001(\0132\024.gazebo.msgs.BoxGeom\022"
      "+\n\010cylinder\030\003 \001(\0132\031.gazebo.msgs.Cylinder"
      "Geom\022%\n\005plane\030\004 \001(\0132\026.gazebo.msgs.PlaneG"
      "eom\022\'\n\006sphere\030\005 \001(\0132\027.gazebo.msgs.Sphere"
      "Geom\022%\n\005image\030\006 \001(\0132\026.gazebo.msgs.ImageG"
      "eom\022-\n\theightmap\030\007 \001(\0132\032.gazebo.msgs.Hei"
      "ghtmapGeom\022#\n\004mesh\030\010 \001(\0132\025.gazebo.msgs.M"
      "eshGeom\022%\n\006points\030\t \003(\0132\025.gazebo.msgs.Ve"
      "ctor3d\022\'\n\010polyline\030\n \003(\0132\025.gazebo.msgs.P"
      "olyline\"\223\001\n\004Type\022\007\n\003BOX\020\001\022\014\n\010CYLINDER\020\002\022"
      "\n\n\006SPHERE\020\003\022\t\n\005PLANE\020\004\022\t\n\005IMAGE\020\005\022\r\n\tHEI"
      "GHTMAP\020\006\022\010\n\004MESH\020\007\022\020\n\014TRIANGLE_FAN\020\010\022\016\n\n"
      "LINE_STRIP\020\t\022\014\n\010POLYLINE\020\n\022\t\n\005EMPTY\020\013"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 757);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "geometry.proto", &protobuf_RegisterTypes);
  ::protobuf_boxgeom_2eproto::AddDescriptors();
  ::protobuf_cylindergeom_2eproto::AddDescriptors();
  ::protobuf_spheregeom_2eproto::AddDescriptors();
  ::protobuf_planegeom_2eproto::AddDescriptors();
  ::protobuf_imagegeom_2eproto::AddDescriptors();
  ::protobuf_heightmapgeom_2eproto::AddDescriptors();
  ::protobuf_meshgeom_2eproto::AddDescriptors();
  ::protobuf_vector3d_2eproto::AddDescriptors();
  ::protobuf_polylinegeom_2eproto::AddDescriptors();
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
}  // namespace protobuf_geometry_2eproto
namespace gazebo {
namespace msgs {
const ::google::protobuf::EnumDescriptor* Geometry_Type_descriptor() {
  protobuf_geometry_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_geometry_2eproto::file_level_enum_descriptors[0];
}
bool Geometry_Type_IsValid(int value) {
  switch (value) {
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
      return true;
    default:
      return false;
  }
}

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const Geometry_Type Geometry::BOX;
const Geometry_Type Geometry::CYLINDER;
const Geometry_Type Geometry::SPHERE;
const Geometry_Type Geometry::PLANE;
const Geometry_Type Geometry::IMAGE;
const Geometry_Type Geometry::HEIGHTMAP;
const Geometry_Type Geometry::MESH;
const Geometry_Type Geometry::TRIANGLE_FAN;
const Geometry_Type Geometry::LINE_STRIP;
const Geometry_Type Geometry::POLYLINE;
const Geometry_Type Geometry::EMPTY;
const Geometry_Type Geometry::Type_MIN;
const Geometry_Type Geometry::Type_MAX;
const int Geometry::Type_ARRAYSIZE;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

// ===================================================================

void Geometry::InitAsDefaultInstance() {
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->box_ = const_cast< ::gazebo::msgs::BoxGeom*>(
      ::gazebo::msgs::BoxGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->cylinder_ = const_cast< ::gazebo::msgs::CylinderGeom*>(
      ::gazebo::msgs::CylinderGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->plane_ = const_cast< ::gazebo::msgs::PlaneGeom*>(
      ::gazebo::msgs::PlaneGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->sphere_ = const_cast< ::gazebo::msgs::SphereGeom*>(
      ::gazebo::msgs::SphereGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->image_ = const_cast< ::gazebo::msgs::ImageGeom*>(
      ::gazebo::msgs::ImageGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->heightmap_ = const_cast< ::gazebo::msgs::HeightmapGeom*>(
      ::gazebo::msgs::HeightmapGeom::internal_default_instance());
  ::gazebo::msgs::_Geometry_default_instance_._instance.get_mutable()->mesh_ = const_cast< ::gazebo::msgs::MeshGeom*>(
      ::gazebo::msgs::MeshGeom::internal_default_instance());
}
void Geometry::clear_box() {
  if (box_ != NULL) box_->Clear();
  clear_has_box();
}
void Geometry::clear_cylinder() {
  if (cylinder_ != NULL) cylinder_->Clear();
  clear_has_cylinder();
}
void Geometry::clear_plane() {
  if (plane_ != NULL) plane_->Clear();
  clear_has_plane();
}
void Geometry::clear_sphere() {
  if (sphere_ != NULL) sphere_->Clear();
  clear_has_sphere();
}
void Geometry::clear_image() {
  if (image_ != NULL) image_->Clear();
  clear_has_image();
}
void Geometry::clear_heightmap() {
  if (heightmap_ != NULL) heightmap_->Clear();
  clear_has_heightmap();
}
void Geometry::clear_mesh() {
  if (mesh_ != NULL) mesh_->Clear();
  clear_has_mesh();
}
void Geometry::clear_points() {
  points_.Clear();
}
void Geometry::clear_polyline() {
  polyline_.Clear();
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Geometry::kTypeFieldNumber;
const int Geometry::kBoxFieldNumber;
const int Geometry::kCylinderFieldNumber;
const int Geometry::kPlaneFieldNumber;
const int Geometry::kSphereFieldNumber;
const int Geometry::kImageFieldNumber;
const int Geometry::kHeightmapFieldNumber;
const int Geometry::kMeshFieldNumber;
const int Geometry::kPointsFieldNumber;
const int Geometry::kPolylineFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Geometry::Geometry()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_geometry_2eproto::scc_info_Geometry.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:gazebo.msgs.Geometry)
}
Geometry::Geometry(const Geometry& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_),
      points_(from.points_),
      polyline_(from.polyline_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_box()) {
    box_ = new ::gazebo::msgs::BoxGeom(*from.box_);
  } else {
    box_ = NULL;
  }
  if (from.has_cylinder()) {
    cylinder_ = new ::gazebo::msgs::CylinderGeom(*from.cylinder_);
  } else {
    cylinder_ = NULL;
  }
  if (from.has_plane()) {
    plane_ = new ::gazebo::msgs::PlaneGeom(*from.plane_);
  } else {
    plane_ = NULL;
  }
  if (from.has_sphere()) {
    sphere_ = new ::gazebo::msgs::SphereGeom(*from.sphere_);
  } else {
    sphere_ = NULL;
  }
  if (from.has_image()) {
    image_ = new ::gazebo::msgs::ImageGeom(*from.image_);
  } else {
    image_ = NULL;
  }
  if (from.has_heightmap()) {
    heightmap_ = new ::gazebo::msgs::HeightmapGeom(*from.heightmap_);
  } else {
    heightmap_ = NULL;
  }
  if (from.has_mesh()) {
    mesh_ = new ::gazebo::msgs::MeshGeom(*from.mesh_);
  } else {
    mesh_ = NULL;
  }
  type_ = from.type_;
  // @@protoc_insertion_point(copy_constructor:gazebo.msgs.Geometry)
}

void Geometry::SharedCtor() {
  ::memset(&box_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&mesh_) -
      reinterpret_cast<char*>(&box_)) + sizeof(mesh_));
  type_ = 1;
}

Geometry::~Geometry() {
  // @@protoc_insertion_point(destructor:gazebo.msgs.Geometry)
  SharedDtor();
}

void Geometry::SharedDtor() {
  if (this != internal_default_instance()) delete box_;
  if (this != internal_default_instance()) delete cylinder_;
  if (this != internal_default_instance()) delete plane_;
  if (this != internal_default_instance()) delete sphere_;
  if (this != internal_default_instance()) delete image_;
  if (this != internal_default_instance()) delete heightmap_;
  if (this != internal_default_instance()) delete mesh_;
}

void Geometry::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* Geometry::descriptor() {
  ::protobuf_geometry_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_geometry_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const Geometry& Geometry::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_geometry_2eproto::scc_info_Geometry.base);
  return *internal_default_instance();
}


void Geometry::Clear() {
// @@protoc_insertion_point(message_clear_start:gazebo.msgs.Geometry)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  points_.Clear();
  polyline_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(box_ != NULL);
      box_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(cylinder_ != NULL);
      cylinder_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(plane_ != NULL);
      plane_->Clear();
    }
    if (cached_has_bits & 0x00000008u) {
      GOOGLE_DCHECK(sphere_ != NULL);
      sphere_->Clear();
    }
    if (cached_has_bits & 0x00000010u) {
      GOOGLE_DCHECK(image_ != NULL);
      image_->Clear();
    }
    if (cached_has_bits & 0x00000020u) {
      GOOGLE_DCHECK(heightmap_ != NULL);
      heightmap_->Clear();
    }
    if (cached_has_bits & 0x00000040u) {
      GOOGLE_DCHECK(mesh_ != NULL);
      mesh_->Clear();
    }
    type_ = 1;
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool Geometry::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:gazebo.msgs.Geometry)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .gazebo.msgs.Geometry.Type type = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          int value;
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   int, ::google::protobuf::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::gazebo::msgs::Geometry_Type_IsValid(value)) {
            set_type(static_cast< ::gazebo::msgs::Geometry_Type >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                1, static_cast< ::google::protobuf::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.BoxGeom box = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_box()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.CylinderGeom cylinder = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_cylinder()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.PlaneGeom plane = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_plane()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.SphereGeom sphere = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_sphere()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.ImageGeom image = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_image()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.HeightmapGeom heightmap = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(58u /* 58 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_heightmap()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .gazebo.msgs.MeshGeom mesh = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(66u /* 66 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_mesh()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.Vector3d points = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(74u /* 74 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_points()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .gazebo.msgs.Polyline polyline = 10;
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(82u /* 82 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
                input, add_polyline()));
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
  // @@protoc_insertion_point(parse_success:gazebo.msgs.Geometry)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:gazebo.msgs.Geometry)
  return false;
#undef DO_
}

void Geometry::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:gazebo.msgs.Geometry)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .gazebo.msgs.Geometry.Type type = 1;
  if (cached_has_bits & 0x00000080u) {
    ::google::protobuf::internal::WireFormatLite::WriteEnum(
      1, this->type(), output);
  }

  // optional .gazebo.msgs.BoxGeom box = 2;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, this->_internal_box(), output);
  }

  // optional .gazebo.msgs.CylinderGeom cylinder = 3;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->_internal_cylinder(), output);
  }

  // optional .gazebo.msgs.PlaneGeom plane = 4;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, this->_internal_plane(), output);
  }

  // optional .gazebo.msgs.SphereGeom sphere = 5;
  if (cached_has_bits & 0x00000008u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, this->_internal_sphere(), output);
  }

  // optional .gazebo.msgs.ImageGeom image = 6;
  if (cached_has_bits & 0x00000010u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      6, this->_internal_image(), output);
  }

  // optional .gazebo.msgs.HeightmapGeom heightmap = 7;
  if (cached_has_bits & 0x00000020u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      7, this->_internal_heightmap(), output);
  }

  // optional .gazebo.msgs.MeshGeom mesh = 8;
  if (cached_has_bits & 0x00000040u) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      8, this->_internal_mesh(), output);
  }

  // repeated .gazebo.msgs.Vector3d points = 9;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->points_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      9,
      this->points(static_cast<int>(i)),
      output);
  }

  // repeated .gazebo.msgs.Polyline polyline = 10;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->polyline_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      10,
      this->polyline(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:gazebo.msgs.Geometry)
}

::google::protobuf::uint8* Geometry::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:gazebo.msgs.Geometry)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .gazebo.msgs.Geometry.Type type = 1;
  if (cached_has_bits & 0x00000080u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteEnumToArray(
      1, this->type(), target);
  }

  // optional .gazebo.msgs.BoxGeom box = 2;
  if (cached_has_bits & 0x00000001u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->_internal_box(), deterministic, target);
  }

  // optional .gazebo.msgs.CylinderGeom cylinder = 3;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->_internal_cylinder(), deterministic, target);
  }

  // optional .gazebo.msgs.PlaneGeom plane = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, this->_internal_plane(), deterministic, target);
  }

  // optional .gazebo.msgs.SphereGeom sphere = 5;
  if (cached_has_bits & 0x00000008u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        5, this->_internal_sphere(), deterministic, target);
  }

  // optional .gazebo.msgs.ImageGeom image = 6;
  if (cached_has_bits & 0x00000010u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        6, this->_internal_image(), deterministic, target);
  }

  // optional .gazebo.msgs.HeightmapGeom heightmap = 7;
  if (cached_has_bits & 0x00000020u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        7, this->_internal_heightmap(), deterministic, target);
  }

  // optional .gazebo.msgs.MeshGeom mesh = 8;
  if (cached_has_bits & 0x00000040u) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        8, this->_internal_mesh(), deterministic, target);
  }

  // repeated .gazebo.msgs.Vector3d points = 9;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->points_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        9, this->points(static_cast<int>(i)), deterministic, target);
  }

  // repeated .gazebo.msgs.Polyline polyline = 10;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->polyline_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        10, this->polyline(static_cast<int>(i)), deterministic, target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:gazebo.msgs.Geometry)
  return target;
}

size_t Geometry::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:gazebo.msgs.Geometry)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  // repeated .gazebo.msgs.Vector3d points = 9;
  {
    unsigned int count = static_cast<unsigned int>(this->points_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->points(static_cast<int>(i)));
    }
  }

  // repeated .gazebo.msgs.Polyline polyline = 10;
  {
    unsigned int count = static_cast<unsigned int>(this->polyline_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          this->polyline(static_cast<int>(i)));
    }
  }

  if (_has_bits_[0 / 32] & 255u) {
    // optional .gazebo.msgs.BoxGeom box = 2;
    if (has_box()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *box_);
    }

    // optional .gazebo.msgs.CylinderGeom cylinder = 3;
    if (has_cylinder()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *cylinder_);
    }

    // optional .gazebo.msgs.PlaneGeom plane = 4;
    if (has_plane()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *plane_);
    }

    // optional .gazebo.msgs.SphereGeom sphere = 5;
    if (has_sphere()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *sphere_);
    }

    // optional .gazebo.msgs.ImageGeom image = 6;
    if (has_image()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *image_);
    }

    // optional .gazebo.msgs.HeightmapGeom heightmap = 7;
    if (has_heightmap()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *heightmap_);
    }

    // optional .gazebo.msgs.MeshGeom mesh = 8;
    if (has_mesh()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSize(
          *mesh_);
    }

    // optional .gazebo.msgs.Geometry.Type type = 1;
    if (has_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::EnumSize(this->type());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Geometry::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:gazebo.msgs.Geometry)
  GOOGLE_DCHECK_NE(&from, this);
  const Geometry* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Geometry>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:gazebo.msgs.Geometry)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:gazebo.msgs.Geometry)
    MergeFrom(*source);
  }
}

void Geometry::MergeFrom(const Geometry& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:gazebo.msgs.Geometry)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  points_.MergeFrom(from.points_);
  polyline_.MergeFrom(from.polyline_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 255u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_box()->::gazebo::msgs::BoxGeom::MergeFrom(from.box());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_cylinder()->::gazebo::msgs::CylinderGeom::MergeFrom(from.cylinder());
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_plane()->::gazebo::msgs::PlaneGeom::MergeFrom(from.plane());
    }
    if (cached_has_bits & 0x00000008u) {
      mutable_sphere()->::gazebo::msgs::SphereGeom::MergeFrom(from.sphere());
    }
    if (cached_has_bits & 0x00000010u) {
      mutable_image()->::gazebo::msgs::ImageGeom::MergeFrom(from.image());
    }
    if (cached_has_bits & 0x00000020u) {
      mutable_heightmap()->::gazebo::msgs::HeightmapGeom::MergeFrom(from.heightmap());
    }
    if (cached_has_bits & 0x00000040u) {
      mutable_mesh()->::gazebo::msgs::MeshGeom::MergeFrom(from.mesh());
    }
    if (cached_has_bits & 0x00000080u) {
      type_ = from.type_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Geometry::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:gazebo.msgs.Geometry)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Geometry::CopyFrom(const Geometry& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:gazebo.msgs.Geometry)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Geometry::IsInitialized() const {
  if (!::google::protobuf::internal::AllAreInitialized(this->points())) return false;
  if (!::google::protobuf::internal::AllAreInitialized(this->polyline())) return false;
  if (has_box()) {
    if (!this->box_->IsInitialized()) return false;
  }
  if (has_cylinder()) {
    if (!this->cylinder_->IsInitialized()) return false;
  }
  if (has_plane()) {
    if (!this->plane_->IsInitialized()) return false;
  }
  if (has_sphere()) {
    if (!this->sphere_->IsInitialized()) return false;
  }
  if (has_image()) {
    if (!this->image_->IsInitialized()) return false;
  }
  if (has_heightmap()) {
    if (!this->heightmap_->IsInitialized()) return false;
  }
  if (has_mesh()) {
    if (!this->mesh_->IsInitialized()) return false;
  }
  return true;
}

void Geometry::Swap(Geometry* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Geometry::InternalSwap(Geometry* other) {
  using std::swap;
  CastToBase(&points_)->InternalSwap(CastToBase(&other->points_));
  CastToBase(&polyline_)->InternalSwap(CastToBase(&other->polyline_));
  swap(box_, other->box_);
  swap(cylinder_, other->cylinder_);
  swap(plane_, other->plane_);
  swap(sphere_, other->sphere_);
  swap(image_, other->image_);
  swap(heightmap_, other->heightmap_);
  swap(mesh_, other->mesh_);
  swap(type_, other->type_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata Geometry::GetMetadata() const {
  protobuf_geometry_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_geometry_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace gazebo
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::gazebo::msgs::Geometry* Arena::CreateMaybeMessage< ::gazebo::msgs::Geometry >(Arena* arena) {
  return Arena::CreateInternal< ::gazebo::msgs::Geometry >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
