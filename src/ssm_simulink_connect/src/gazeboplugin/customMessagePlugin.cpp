/* Copyright 2019 The MathWorks, Inc. */

//This function is for internal use only. It may be removed in the future.
// EDIT CAREFULLY!
/**Note: 
 * code line from onWorldUpdateStart() and subscribe functions are auto 
 * generated based on user-defined custom message. The commented part 
 * can be removed and edit as per need. Further, the message initialization 
 * is needed in the onWorldUpdateStart(). Please take help of documentation.
 */

#include <iostream>
#include <math.h>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <MultiplevectorVisual.pb.h>
#include <SphericalVector.pb.h>
#include <VectorVisual.pb.h>
#include <boxgeom.pb.h>
#include <collision.pb.h>
#include <color.pb.h>
#include <contact.pb.h>
#include <contacts.pb.h>
#include <cylindergeom.pb.h>
#include <double.pb.h>
#include <friction.pb.h>
#include <geometry.pb.h>
#include <heightmapgeom.pb.h>
#include <image.pb.h>
#include <imagegeom.pb.h>
#include <images_stamped.pb.h>
#include <imu.pb.h>
#include <imu_sensor.pb.h>
#include <inertial.pb.h>
#include <int.pb.h>
#include <joint_wrench.pb.h>
#include <material.pb.h>
#include <meshgeom.pb.h>
#include <planegeom.pb.h>
#include <plugin.pb.h>
#include <polylinegeom.pb.h>
#include <pose.pb.h>
#include <quaternion.pb.h>
#include <sensor_noise.pb.h>
#include <spheregeom.pb.h>
#include <surface.pb.h>
#include <time.pb.h>
#include <vector2d.pb.h>
#include <vector3d.pb.h>
#include <visual.pb.h>
#include <wrench.pb.h>


namespace gazebo
{

typedef const boost::shared_ptr<const ssm_msgs::msgs::MultiplevectorVisual> ssm_msgs_msgs_MultiplevectorVisualPtr;
typedef const boost::shared_ptr<const ssm_msgs::msgs::SphericalVector> ssm_msgs_msgs_SphericalVectorPtr;
typedef const boost::shared_ptr<const ssm_msgs::msgs::VectorVisual> ssm_msgs_msgs_VectorVisualPtr;
typedef const boost::shared_ptr<const gazebo::msgs::BoxGeom> gazebo_msgs_BoxGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Collision> gazebo_msgs_CollisionPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Color> gazebo_msgs_ColorPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Contact> gazebo_msgs_ContactPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Contacts> gazebo_msgs_ContactsPtr;
typedef const boost::shared_ptr<const gazebo::msgs::CylinderGeom> gazebo_msgs_CylinderGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Double> gazebo_msgs_DoublePtr;
typedef const boost::shared_ptr<const gazebo::msgs::Friction> gazebo_msgs_FrictionPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Geometry> gazebo_msgs_GeometryPtr;
typedef const boost::shared_ptr<const gazebo::msgs::HeightmapGeom> gazebo_msgs_HeightmapGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Image> gazebo_msgs_ImagePtr;
typedef const boost::shared_ptr<const gazebo::msgs::ImageGeom> gazebo_msgs_ImageGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::ImagesStamped> gazebo_msgs_ImagesStampedPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMU> gazebo_msgs_IMUPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMUSensor> gazebo_msgs_IMUSensorPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Inertial> gazebo_msgs_InertialPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Int> gazebo_msgs_IntPtr;
typedef const boost::shared_ptr<const gazebo::msgs::JointWrench> gazebo_msgs_JointWrenchPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Material> gazebo_msgs_MaterialPtr;
typedef const boost::shared_ptr<const gazebo::msgs::MeshGeom> gazebo_msgs_MeshGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::PlaneGeom> gazebo_msgs_PlaneGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Plugin> gazebo_msgs_PluginPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Polyline> gazebo_msgs_PolylinePtr;
typedef const boost::shared_ptr<const gazebo::msgs::Pose> gazebo_msgs_PosePtr;
typedef const boost::shared_ptr<const gazebo::msgs::Quaternion> gazebo_msgs_QuaternionPtr;
typedef const boost::shared_ptr<const gazebo::msgs::SensorNoise> gazebo_msgs_SensorNoisePtr;
typedef const boost::shared_ptr<const gazebo::msgs::SphereGeom> gazebo_msgs_SphereGeomPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Surface> gazebo_msgs_SurfacePtr;
typedef const boost::shared_ptr<const gazebo::msgs::Time> gazebo_msgs_TimePtr;
typedef const boost::shared_ptr<const gazebo::msgs::Vector2d> gazebo_msgs_Vector2dPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Vector3d> gazebo_msgs_Vector3dPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Visual> gazebo_msgs_VisualPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Wrench> gazebo_msgs_WrenchPtr;


class customMessagePlugin : public WorldPlugin
{
  transport::NodePtr node;

  transport::SubscriberPtr commandSubscriberssm_msgs_msgs_multiplevectorvisual0;
transport::SubscriberPtr commandSubscriberssm_msgs_msgs_sphericalvector0;
transport::SubscriberPtr commandSubscriberssm_msgs_msgs_vectorvisual0;
transport::SubscriberPtr commandSubscribergazebo_msgs_boxgeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_collision0;
transport::SubscriberPtr commandSubscribergazebo_msgs_color0;
transport::SubscriberPtr commandSubscribergazebo_msgs_contact0;
transport::SubscriberPtr commandSubscribergazebo_msgs_contacts0;
transport::SubscriberPtr commandSubscribergazebo_msgs_cylindergeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_double0;
transport::SubscriberPtr commandSubscribergazebo_msgs_friction0;
transport::SubscriberPtr commandSubscribergazebo_msgs_geometry0;
transport::SubscriberPtr commandSubscribergazebo_msgs_heightmapgeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_image0;
transport::SubscriberPtr commandSubscribergazebo_msgs_imagegeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_imagesstamped0;
transport::SubscriberPtr commandSubscribergazebo_msgs_imu0;
transport::SubscriberPtr commandSubscribergazebo_msgs_imusensor0;
transport::SubscriberPtr commandSubscribergazebo_msgs_inertial0;
transport::SubscriberPtr commandSubscribergazebo_msgs_int0;
transport::SubscriberPtr commandSubscribergazebo_msgs_jointwrench0;
transport::SubscriberPtr commandSubscribergazebo_msgs_material0;
transport::SubscriberPtr commandSubscribergazebo_msgs_meshgeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_planegeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_plugin0;
transport::SubscriberPtr commandSubscribergazebo_msgs_polyline0;
transport::SubscriberPtr commandSubscribergazebo_msgs_pose0;
transport::SubscriberPtr commandSubscribergazebo_msgs_quaternion0;
transport::SubscriberPtr commandSubscribergazebo_msgs_sensornoise0;
transport::SubscriberPtr commandSubscribergazebo_msgs_spheregeom0;
transport::SubscriberPtr commandSubscribergazebo_msgs_surface0;
transport::SubscriberPtr commandSubscribergazebo_msgs_time0;
transport::SubscriberPtr commandSubscribergazebo_msgs_vector2d0;
transport::SubscriberPtr commandSubscribergazebo_msgs_vector3d0;
transport::SubscriberPtr commandSubscribergazebo_msgs_visual0;
transport::SubscriberPtr commandSubscribergazebo_msgs_wrench0;


  gazebo::transport::PublisherPtr commandPublisherssm_msgs_msgs_multiplevectorvisual0;
gazebo::transport::PublisherPtr commandPublisherssm_msgs_msgs_sphericalvector0;
gazebo::transport::PublisherPtr commandPublisherssm_msgs_msgs_vectorvisual0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_boxgeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_collision0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_color0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_contact0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_contacts0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_cylindergeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_double0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_friction0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_geometry0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_heightmapgeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_image0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_imagegeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_imagesstamped0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_imu0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_imusensor0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_inertial0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_int0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_jointwrench0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_material0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_meshgeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_planegeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_plugin0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_polyline0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_pose0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_quaternion0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_sensornoise0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_spheregeom0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_surface0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_time0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_vector2d0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_vector3d0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_visual0;
gazebo::transport::PublisherPtr commandPublishergazebo_msgs_wrench0;


  physics::WorldPtr world;
  gazebo::event::ConnectionPtr worldUpdateStartEventConnection;
  gazebo::event::ConnectionPtr m_worldUpdateEndEventConnection;
  std::mutex pubMutex;

  public:

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;

    // Initialize the node with the world name
    node->Init(world->Name());

    commandSubscriberssm_msgs_msgs_multiplevectorvisual0 = node->Subscribe("gazebo/default/ssm_msgs_msgs_MultiplevectorVisual/test_subscriber0", &customMessagePlugin::subscribeCallbackssm_msgs_msgs_multiplevectorvisual0, this);
commandSubscriberssm_msgs_msgs_sphericalvector0 = node->Subscribe("gazebo/default/ssm_msgs_msgs_SphericalVector/test_subscriber0", &customMessagePlugin::subscribeCallbackssm_msgs_msgs_sphericalvector0, this);
commandSubscriberssm_msgs_msgs_vectorvisual0 = node->Subscribe("gazebo/default/ssm_msgs_msgs_VectorVisual/test_subscriber0", &customMessagePlugin::subscribeCallbackssm_msgs_msgs_vectorvisual0, this);
commandSubscribergazebo_msgs_boxgeom0 = node->Subscribe("gazebo/default/gazebo_msgs_BoxGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_boxgeom0, this);
commandSubscribergazebo_msgs_collision0 = node->Subscribe("gazebo/default/gazebo_msgs_Collision/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_collision0, this);
commandSubscribergazebo_msgs_color0 = node->Subscribe("gazebo/default/gazebo_msgs_Color/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_color0, this);
commandSubscribergazebo_msgs_contact0 = node->Subscribe("gazebo/default/gazebo_msgs_Contact/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_contact0, this);
commandSubscribergazebo_msgs_contacts0 = node->Subscribe("gazebo/default/gazebo_msgs_Contacts/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_contacts0, this);
commandSubscribergazebo_msgs_cylindergeom0 = node->Subscribe("gazebo/default/gazebo_msgs_CylinderGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_cylindergeom0, this);
commandSubscribergazebo_msgs_double0 = node->Subscribe("gazebo/default/gazebo_msgs_Double/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_double0, this);
commandSubscribergazebo_msgs_friction0 = node->Subscribe("gazebo/default/gazebo_msgs_Friction/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_friction0, this);
commandSubscribergazebo_msgs_geometry0 = node->Subscribe("gazebo/default/gazebo_msgs_Geometry/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_geometry0, this);
commandSubscribergazebo_msgs_heightmapgeom0 = node->Subscribe("gazebo/default/gazebo_msgs_HeightmapGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_heightmapgeom0, this);
commandSubscribergazebo_msgs_image0 = node->Subscribe("gazebo/default/gazebo_msgs_Image/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_image0, this);
commandSubscribergazebo_msgs_imagegeom0 = node->Subscribe("gazebo/default/gazebo_msgs_ImageGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_imagegeom0, this);
commandSubscribergazebo_msgs_imagesstamped0 = node->Subscribe("gazebo/default/gazebo_msgs_ImagesStamped/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_imagesstamped0, this);
commandSubscribergazebo_msgs_imu0 = node->Subscribe("gazebo/default/gazebo_msgs_IMU/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_imu0, this);
commandSubscribergazebo_msgs_imusensor0 = node->Subscribe("gazebo/default/gazebo_msgs_IMUSensor/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_imusensor0, this);
commandSubscribergazebo_msgs_inertial0 = node->Subscribe("gazebo/default/gazebo_msgs_Inertial/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_inertial0, this);
commandSubscribergazebo_msgs_int0 = node->Subscribe("gazebo/default/gazebo_msgs_Int/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_int0, this);
commandSubscribergazebo_msgs_jointwrench0 = node->Subscribe("gazebo/default/gazebo_msgs_JointWrench/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_jointwrench0, this);
commandSubscribergazebo_msgs_material0 = node->Subscribe("gazebo/default/gazebo_msgs_Material/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_material0, this);
commandSubscribergazebo_msgs_meshgeom0 = node->Subscribe("gazebo/default/gazebo_msgs_MeshGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_meshgeom0, this);
commandSubscribergazebo_msgs_planegeom0 = node->Subscribe("gazebo/default/gazebo_msgs_PlaneGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_planegeom0, this);
commandSubscribergazebo_msgs_plugin0 = node->Subscribe("gazebo/default/gazebo_msgs_Plugin/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_plugin0, this);
commandSubscribergazebo_msgs_polyline0 = node->Subscribe("gazebo/default/gazebo_msgs_Polyline/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_polyline0, this);
commandSubscribergazebo_msgs_pose0 = node->Subscribe("gazebo/default/gazebo_msgs_Pose/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_pose0, this);
commandSubscribergazebo_msgs_quaternion0 = node->Subscribe("gazebo/default/gazebo_msgs_Quaternion/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_quaternion0, this);
commandSubscribergazebo_msgs_sensornoise0 = node->Subscribe("gazebo/default/gazebo_msgs_SensorNoise/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_sensornoise0, this);
commandSubscribergazebo_msgs_spheregeom0 = node->Subscribe("gazebo/default/gazebo_msgs_SphereGeom/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_spheregeom0, this);
commandSubscribergazebo_msgs_surface0 = node->Subscribe("gazebo/default/gazebo_msgs_Surface/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_surface0, this);
commandSubscribergazebo_msgs_time0 = node->Subscribe("gazebo/default/gazebo_msgs_Time/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_time0, this);
commandSubscribergazebo_msgs_vector2d0 = node->Subscribe("gazebo/default/gazebo_msgs_Vector2d/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_vector2d0, this);
commandSubscribergazebo_msgs_vector3d0 = node->Subscribe("gazebo/default/gazebo_msgs_Vector3d/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_vector3d0, this);
commandSubscribergazebo_msgs_visual0 = node->Subscribe("gazebo/default/gazebo_msgs_Visual/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_visual0, this);
commandSubscribergazebo_msgs_wrench0 = node->Subscribe("gazebo/default/gazebo_msgs_Wrench/test_subscriber0", &customMessagePlugin::subscribeCallbackgazebo_msgs_wrench0, this);


    commandPublisherssm_msgs_msgs_multiplevectorvisual0 = node->Advertise<ssm_msgs::msgs::MultiplevectorVisual>("gazebo/default/ssm_msgs_msgs_MultiplevectorVisual/test_publisher");
commandPublisherssm_msgs_msgs_sphericalvector0 = node->Advertise<ssm_msgs::msgs::SphericalVector>("gazebo/default/ssm_msgs_msgs_SphericalVector/test_publisher");
commandPublisherssm_msgs_msgs_vectorvisual0 = node->Advertise<ssm_msgs::msgs::VectorVisual>("gazebo/default/ssm_msgs_msgs_VectorVisual/test_publisher");
commandPublishergazebo_msgs_boxgeom0 = node->Advertise<gazebo::msgs::BoxGeom>("gazebo/default/gazebo_msgs_BoxGeom/test_publisher");
commandPublishergazebo_msgs_collision0 = node->Advertise<gazebo::msgs::Collision>("gazebo/default/gazebo_msgs_Collision/test_publisher");
commandPublishergazebo_msgs_color0 = node->Advertise<gazebo::msgs::Color>("gazebo/default/gazebo_msgs_Color/test_publisher");
commandPublishergazebo_msgs_contact0 = node->Advertise<gazebo::msgs::Contact>("gazebo/default/gazebo_msgs_Contact/test_publisher");
commandPublishergazebo_msgs_contacts0 = node->Advertise<gazebo::msgs::Contacts>("gazebo/default/gazebo_msgs_Contacts/test_publisher");
commandPublishergazebo_msgs_cylindergeom0 = node->Advertise<gazebo::msgs::CylinderGeom>("gazebo/default/gazebo_msgs_CylinderGeom/test_publisher");
commandPublishergazebo_msgs_double0 = node->Advertise<gazebo::msgs::Double>("gazebo/default/gazebo_msgs_Double/test_publisher");
commandPublishergazebo_msgs_friction0 = node->Advertise<gazebo::msgs::Friction>("gazebo/default/gazebo_msgs_Friction/test_publisher");
commandPublishergazebo_msgs_geometry0 = node->Advertise<gazebo::msgs::Geometry>("gazebo/default/gazebo_msgs_Geometry/test_publisher");
commandPublishergazebo_msgs_heightmapgeom0 = node->Advertise<gazebo::msgs::HeightmapGeom>("gazebo/default/gazebo_msgs_HeightmapGeom/test_publisher");
commandPublishergazebo_msgs_image0 = node->Advertise<gazebo::msgs::Image>("gazebo/default/gazebo_msgs_Image/test_publisher");
commandPublishergazebo_msgs_imagegeom0 = node->Advertise<gazebo::msgs::ImageGeom>("gazebo/default/gazebo_msgs_ImageGeom/test_publisher");
commandPublishergazebo_msgs_imagesstamped0 = node->Advertise<gazebo::msgs::ImagesStamped>("gazebo/default/gazebo_msgs_ImagesStamped/test_publisher");
commandPublishergazebo_msgs_imu0 = node->Advertise<gazebo::msgs::IMU>("gazebo/default/gazebo_msgs_IMU/test_publisher");
commandPublishergazebo_msgs_imusensor0 = node->Advertise<gazebo::msgs::IMUSensor>("gazebo/default/gazebo_msgs_IMUSensor/test_publisher");
commandPublishergazebo_msgs_inertial0 = node->Advertise<gazebo::msgs::Inertial>("gazebo/default/gazebo_msgs_Inertial/test_publisher");
commandPublishergazebo_msgs_int0 = node->Advertise<gazebo::msgs::Int>("gazebo/default/gazebo_msgs_Int/test_publisher");
commandPublishergazebo_msgs_jointwrench0 = node->Advertise<gazebo::msgs::JointWrench>("gazebo/default/gazebo_msgs_JointWrench/test_publisher");
commandPublishergazebo_msgs_material0 = node->Advertise<gazebo::msgs::Material>("gazebo/default/gazebo_msgs_Material/test_publisher");
commandPublishergazebo_msgs_meshgeom0 = node->Advertise<gazebo::msgs::MeshGeom>("gazebo/default/gazebo_msgs_MeshGeom/test_publisher");
commandPublishergazebo_msgs_planegeom0 = node->Advertise<gazebo::msgs::PlaneGeom>("gazebo/default/gazebo_msgs_PlaneGeom/test_publisher");
commandPublishergazebo_msgs_plugin0 = node->Advertise<gazebo::msgs::Plugin>("gazebo/default/gazebo_msgs_Plugin/test_publisher");
commandPublishergazebo_msgs_polyline0 = node->Advertise<gazebo::msgs::Polyline>("gazebo/default/gazebo_msgs_Polyline/test_publisher");
commandPublishergazebo_msgs_pose0 = node->Advertise<gazebo::msgs::Pose>("gazebo/default/gazebo_msgs_Pose/test_publisher");
commandPublishergazebo_msgs_quaternion0 = node->Advertise<gazebo::msgs::Quaternion>("gazebo/default/gazebo_msgs_Quaternion/test_publisher");
commandPublishergazebo_msgs_sensornoise0 = node->Advertise<gazebo::msgs::SensorNoise>("gazebo/default/gazebo_msgs_SensorNoise/test_publisher");
commandPublishergazebo_msgs_spheregeom0 = node->Advertise<gazebo::msgs::SphereGeom>("gazebo/default/gazebo_msgs_SphereGeom/test_publisher");
commandPublishergazebo_msgs_surface0 = node->Advertise<gazebo::msgs::Surface>("gazebo/default/gazebo_msgs_Surface/test_publisher");
commandPublishergazebo_msgs_time0 = node->Advertise<gazebo::msgs::Time>("gazebo/default/gazebo_msgs_Time/test_publisher");
commandPublishergazebo_msgs_vector2d0 = node->Advertise<gazebo::msgs::Vector2d>("gazebo/default/gazebo_msgs_Vector2d/test_publisher");
commandPublishergazebo_msgs_vector3d0 = node->Advertise<gazebo::msgs::Vector3d>("gazebo/default/gazebo_msgs_Vector3d/test_publisher");
commandPublishergazebo_msgs_visual0 = node->Advertise<gazebo::msgs::Visual>("gazebo/default/gazebo_msgs_Visual/test_publisher");
commandPublishergazebo_msgs_wrench0 = node->Advertise<gazebo::msgs::Wrench>("gazebo/default/gazebo_msgs_Wrench/test_publisher");


    this->worldUpdateStartEventConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&customMessagePlugin::onWorldUpdateStart, this));
    this->m_worldUpdateEndEventConnection = gazebo::event::Events::ConnectWorldUpdateEnd(
        std::bind(&customMessagePlugin::onWorldUpdateEnd, this));
  }

  void onWorldUpdateStart()
  {
    std::lock_guard<std::mutex> lock(pubMutex);
    //ssm_msgs::msgs::MultiplevectorVisual  ssm_msgs_msgs_multiplevectorvisual_msg;
//commandPublisherssm_msgs_msgs_multiplevectorvisual0->Publish(ssm_msgs_msgs_multiplevectorvisual_msg);

//ssm_msgs::msgs::SphericalVector  ssm_msgs_msgs_sphericalvector_msg;
//commandPublisherssm_msgs_msgs_sphericalvector0->Publish(ssm_msgs_msgs_sphericalvector_msg);

//ssm_msgs::msgs::VectorVisual  ssm_msgs_msgs_vectorvisual_msg;
//commandPublisherssm_msgs_msgs_vectorvisual0->Publish(ssm_msgs_msgs_vectorvisual_msg);

//gazebo::msgs::BoxGeom  gazebo_msgs_boxgeom_msg;
//commandPublishergazebo_msgs_boxgeom0->Publish(gazebo_msgs_boxgeom_msg);

//gazebo::msgs::Collision  gazebo_msgs_collision_msg;
//commandPublishergazebo_msgs_collision0->Publish(gazebo_msgs_collision_msg);

//gazebo::msgs::Color  gazebo_msgs_color_msg;
//commandPublishergazebo_msgs_color0->Publish(gazebo_msgs_color_msg);

//gazebo::msgs::Contact  gazebo_msgs_contact_msg;
//commandPublishergazebo_msgs_contact0->Publish(gazebo_msgs_contact_msg);

//gazebo::msgs::Contacts  gazebo_msgs_contacts_msg;
//commandPublishergazebo_msgs_contacts0->Publish(gazebo_msgs_contacts_msg);

//gazebo::msgs::CylinderGeom  gazebo_msgs_cylindergeom_msg;
//commandPublishergazebo_msgs_cylindergeom0->Publish(gazebo_msgs_cylindergeom_msg);

//gazebo::msgs::Double  gazebo_msgs_double_msg;
//commandPublishergazebo_msgs_double0->Publish(gazebo_msgs_double_msg);

//gazebo::msgs::Friction  gazebo_msgs_friction_msg;
//commandPublishergazebo_msgs_friction0->Publish(gazebo_msgs_friction_msg);

//gazebo::msgs::Geometry  gazebo_msgs_geometry_msg;
//commandPublishergazebo_msgs_geometry0->Publish(gazebo_msgs_geometry_msg);

//gazebo::msgs::HeightmapGeom  gazebo_msgs_heightmapgeom_msg;
//commandPublishergazebo_msgs_heightmapgeom0->Publish(gazebo_msgs_heightmapgeom_msg);

//gazebo::msgs::Image  gazebo_msgs_image_msg;
//commandPublishergazebo_msgs_image0->Publish(gazebo_msgs_image_msg);

//gazebo::msgs::ImageGeom  gazebo_msgs_imagegeom_msg;
//commandPublishergazebo_msgs_imagegeom0->Publish(gazebo_msgs_imagegeom_msg);

//gazebo::msgs::ImagesStamped  gazebo_msgs_imagesstamped_msg;
//commandPublishergazebo_msgs_imagesstamped0->Publish(gazebo_msgs_imagesstamped_msg);

//gazebo::msgs::IMU  gazebo_msgs_imu_msg;
//commandPublishergazebo_msgs_imu0->Publish(gazebo_msgs_imu_msg);

//gazebo::msgs::IMUSensor  gazebo_msgs_imusensor_msg;
//commandPublishergazebo_msgs_imusensor0->Publish(gazebo_msgs_imusensor_msg);

//gazebo::msgs::Inertial  gazebo_msgs_inertial_msg;
//commandPublishergazebo_msgs_inertial0->Publish(gazebo_msgs_inertial_msg);

//gazebo::msgs::Int  gazebo_msgs_int_msg;
//commandPublishergazebo_msgs_int0->Publish(gazebo_msgs_int_msg);

//gazebo::msgs::JointWrench  gazebo_msgs_jointwrench_msg;
//commandPublishergazebo_msgs_jointwrench0->Publish(gazebo_msgs_jointwrench_msg);

//gazebo::msgs::Material  gazebo_msgs_material_msg;
//commandPublishergazebo_msgs_material0->Publish(gazebo_msgs_material_msg);

//gazebo::msgs::MeshGeom  gazebo_msgs_meshgeom_msg;
//commandPublishergazebo_msgs_meshgeom0->Publish(gazebo_msgs_meshgeom_msg);

//gazebo::msgs::PlaneGeom  gazebo_msgs_planegeom_msg;
//commandPublishergazebo_msgs_planegeom0->Publish(gazebo_msgs_planegeom_msg);

//gazebo::msgs::Plugin  gazebo_msgs_plugin_msg;
//commandPublishergazebo_msgs_plugin0->Publish(gazebo_msgs_plugin_msg);

//gazebo::msgs::Polyline  gazebo_msgs_polyline_msg;
//commandPublishergazebo_msgs_polyline0->Publish(gazebo_msgs_polyline_msg);

//gazebo::msgs::Pose  gazebo_msgs_pose_msg;
//commandPublishergazebo_msgs_pose0->Publish(gazebo_msgs_pose_msg);

//gazebo::msgs::Quaternion  gazebo_msgs_quaternion_msg;
//commandPublishergazebo_msgs_quaternion0->Publish(gazebo_msgs_quaternion_msg);

//gazebo::msgs::SensorNoise  gazebo_msgs_sensornoise_msg;
//commandPublishergazebo_msgs_sensornoise0->Publish(gazebo_msgs_sensornoise_msg);

//gazebo::msgs::SphereGeom  gazebo_msgs_spheregeom_msg;
//commandPublishergazebo_msgs_spheregeom0->Publish(gazebo_msgs_spheregeom_msg);

//gazebo::msgs::Surface  gazebo_msgs_surface_msg;
//commandPublishergazebo_msgs_surface0->Publish(gazebo_msgs_surface_msg);

//gazebo::msgs::Time  gazebo_msgs_time_msg;
//commandPublishergazebo_msgs_time0->Publish(gazebo_msgs_time_msg);

//gazebo::msgs::Vector2d  gazebo_msgs_vector2d_msg;
//commandPublishergazebo_msgs_vector2d0->Publish(gazebo_msgs_vector2d_msg);

//gazebo::msgs::Vector3d  gazebo_msgs_vector3d_msg;
//commandPublishergazebo_msgs_vector3d0->Publish(gazebo_msgs_vector3d_msg);

//gazebo::msgs::Visual  gazebo_msgs_visual_msg;
//commandPublishergazebo_msgs_visual0->Publish(gazebo_msgs_visual_msg);

//gazebo::msgs::Wrench  gazebo_msgs_wrench_msg;
//commandPublishergazebo_msgs_wrench0->Publish(gazebo_msgs_wrench_msg);


  }
  
  void onWorldUpdateEnd()
  {
  }

  void subscribeCallbackssm_msgs_msgs_multiplevectorvisual0(ssm_msgs_msgs_MultiplevectorVisualPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackssm_msgs_msgs_sphericalvector0(ssm_msgs_msgs_SphericalVectorPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackssm_msgs_msgs_vectorvisual0(ssm_msgs_msgs_VectorVisualPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_boxgeom0(gazebo_msgs_BoxGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_collision0(gazebo_msgs_CollisionPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_color0(gazebo_msgs_ColorPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_contact0(gazebo_msgs_ContactPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_contacts0(gazebo_msgs_ContactsPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_cylindergeom0(gazebo_msgs_CylinderGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_double0(gazebo_msgs_DoublePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_friction0(gazebo_msgs_FrictionPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_geometry0(gazebo_msgs_GeometryPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_heightmapgeom0(gazebo_msgs_HeightmapGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_image0(gazebo_msgs_ImagePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_imagegeom0(gazebo_msgs_ImageGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_imagesstamped0(gazebo_msgs_ImagesStampedPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_imu0(gazebo_msgs_IMUPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_imusensor0(gazebo_msgs_IMUSensorPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_inertial0(gazebo_msgs_InertialPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_int0(gazebo_msgs_IntPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_jointwrench0(gazebo_msgs_JointWrenchPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_material0(gazebo_msgs_MaterialPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_meshgeom0(gazebo_msgs_MeshGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_planegeom0(gazebo_msgs_PlaneGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_plugin0(gazebo_msgs_PluginPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_polyline0(gazebo_msgs_PolylinePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_pose0(gazebo_msgs_PosePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_quaternion0(gazebo_msgs_QuaternionPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_sensornoise0(gazebo_msgs_SensorNoisePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_spheregeom0(gazebo_msgs_SphereGeomPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_surface0(gazebo_msgs_SurfacePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_time0(gazebo_msgs_TimePtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_vector2d0(gazebo_msgs_Vector2dPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_vector3d0(gazebo_msgs_Vector3dPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_visual0(gazebo_msgs_VisualPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}

void subscribeCallbackgazebo_msgs_wrench0(gazebo_msgs_WrenchPtr &msg)
{
  //std::cout << msg->DebugString() << std::endl;
}



};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(customMessagePlugin)
}