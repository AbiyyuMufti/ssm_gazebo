
#include <algorithm>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "thrust_vectoring.hh"
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ThrustVectoring)

//////////////////////////////////
ThrustVectoring::ThrustVectoring()
{
    this->forward_direction = ignition::math::Vector3d(1, 0, 0);
    this->thurst_origin = ignition::math::Vector3d(0, 0, 0);
    this->thrust_magnitude = 0.0;
    this->thrust_vector_value.Set(0,0,0);

}

//////////////////////////////////
ThrustVectoring::~ThrustVectoring()
{

}

void ThrustVectoring::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "ThrustVectoring _model pointer is NULL");
    GZ_ASSERT(_sdf, "ThrustVectoring _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "ThrustVectoring world pointer is NULL");

    this->physics = this->world->Physics();
    this->last_pub_time = this->world->SimTime();

    GZ_ASSERT(this->physics, "ThrustVectoring physics pointer is NULL");

    GZ_ASSERT(_sdf, "ThrustVectoring _sdf pointer is NULL");

    if (_sdf->HasElement("forward_dir"))
        this->forward_direction = _sdf->Get<ignition::math::Vector3d>("forward");
    this->forward_direction.Normalize();

    if (_sdf->HasElement("thrust_origin"))
        this->thurst_origin = _sdf->Get<ignition::math::Vector3d>("forward");
    this->thurst_origin.Normalize();

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        // GZ_ASSERT(elem, "Element link_name doesn't exist!");
        std::string linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(linkName);
        // GZ_ASSERT(this->link, "Link was NULL");

        if (!this->link)
        {
        gzerr << "Link with name[" << linkName << "] not found. "
            << "The ThrustVectoring will not generate forces\n";
        }
        else
        {
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ThrustVectoring::OnUpdate, this));
        }
    }

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_thrust_vectoring] Please specify a robotNamespace.\n";
    }

    if (_sdf->HasElement("topic_name")) {
        const auto thrust_visual_topic_ = this->sdf->Get<std::string>("topic_name");
        thrust_visual_pub_ = this->node_handle_->Advertise<physics_msgs::msgs::Force>("~/" + thrust_visual_topic_);
        gzdbg << "Publishing to ~/" << thrust_visual_topic_ << std::endl;
    }

    if (_sdf->HasElement("thrust_topic")) {
        const auto thrust_topic_ = this->sdf->Get<std::string>("thrust_topic");
      this->thrust_val_sub_ = this->node_handle_->Subscribe("~/" + thrust_topic_, &ThrustVectoring::OnThrustVectoringMsgs, this);
      gzdbg << "Subscribing on ~/" << thrust_topic_ << std::endl;
    }

}

void ThrustVectoring::OnUpdate()
{
    GZ_ASSERT(this->link, "Link was NULL");
    
    const common::Time current_time = this->world->SimTime();
    const double dt = (current_time - this->last_pub_time).Double();

    // get link actual pose
    ignition::math::Pose3d pose = this->link->WorldPose();

    // rotate forward direction vectors into inertial frame
    ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward_direction);
    forwardI.Normalize();

    // normalize the thrust vector
    this->thrust_vector_value.Normalize();

    // rotating the z Axis of the thrust vector so it point the flight forward direction
    // to do that:
    // --> create z vector direction
    ignition::math::Vector3d z_vector(0,0,1);

    // --> calculate the angle to rotate from the z axis and the forward direction
    double cos_angle_to_rotate = z_vector.Dot(this->thrust_vector_value) / (z_vector.Length() * thrust_vector_value.Length());
    double angle_to_rotate = acos(cos_angle_to_rotate);

    // --> calculate the rotation axis to rotate so that the z vector located in the forward direction
    ignition::math::Vector3d axis_rotation = this->thrust_vector_value.Cross(z_vector);

    // --> create quaternion for the rotation using the axis rotation and the angle
    ignition::math::Quaternion quat_rot(axis_rotation, angle_to_rotate);

    // --> rotate the thrust vector to the forward direction in initial frame
    this->thrust_vector_value = quat_rot.RotateVector(this->thrust_vector_value);
    this->thrust_vector_value.Normalize();

    // calculate the thrust force with the thrust magnintude
    ignition::math::Vector3d thrust_force = this->thrust_vector_value * this->thrust_magnitude;

    // apply forces at thrust origin
    this->link->AddForceAtRelativePosition(thrust_force, this->thurst_origin);
    
    // publishe the force vector for visualization
    if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
        msgs::Vector3d* force_center_msg = new msgs::Vector3d;
        force_center_msg->set_x(this->thurst_origin.X());
        force_center_msg->set_y(this->thurst_origin.Y());
        force_center_msg->set_z(this->thurst_origin.Z());

        msgs::Vector3d* force_vector_msg = new msgs::Vector3d;
        force_vector_msg->set_x(thrust_force.X());
        force_vector_msg->set_y(thrust_force.Y());
        force_vector_msg->set_z(thrust_force.Z());

        physics_msgs::msgs::Force force_msg;
        force_msg.set_allocated_center(force_center_msg);
        force_msg.set_allocated_force(force_vector_msg);

        thrust_visual_pub_->Publish(force_msg);
        this->last_pub_time = current_time;
    }

}

void ThrustVectoring::OnThrustVectoringMsgs(const boost::shared_ptr<const physics_msgs::msgs::SphericalVector> &msg)
{
    
    this->thrust_magnitude = msg->r();
    this->thrust_vector_value.X(cos(msg->phi())*sin(msg->theta()));
    this->thrust_vector_value.Y(sin(msg->phi())*sin(msg->theta()));
    this->thrust_vector_value.Z(cos(msg->theta()));
    
}