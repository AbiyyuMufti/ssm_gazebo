
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
    // Start Loading the plugin
    ROS_WARN("Loading thrust vectoring");

    // Load Model
    GZ_ASSERT(_model, "ThrustVectoring _model pointer is NULL");
    this->model = _model;

    // Load SDF
    GZ_ASSERT(_sdf, "ThrustVectoring _sdf pointer is NULL");
    this->sdf = _sdf;

    // Load World
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "ThrustVectoring world pointer is NULL");

    // Load Physics
    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "ThrustVectoring physics pointer is NULL");

    // Store simulation time
    this->last_pub_time = this->world->SimTime();

    // Get link element which the thrust originate from
    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        GZ_ASSERT(elem, "Element link_name doesn't exist!");
        std::string linkName = elem->Get<std::string>();
        
        this->link = this->model->GetLink(linkName);
        GZ_ASSERT(this->link, "Link was NULL");

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

    // Get the forward direction of thrust relative to the link object
    if (_sdf->HasElement("forward_dir"))
        this->forward_direction = _sdf->Get<ignition::math::Vector3d>("forward_dir");
    this->forward_direction.Normalize();
    ROS_WARN_STREAM("forward_dir " << this->forward_direction);

    // Get the origin of thrust relative to the link object
    if (_sdf->HasElement("thrust_origin"))
        this->thurst_origin = _sdf->Get<ignition::math::Vector3d>("thrust_origin");
    ROS_WARN_STREAM("origin " << this->thurst_origin);

    // Use Additional Namespace if neede
    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_thrust_vectoring] Please specify a robotNamespace.\n";
    }
    
    // Gazebo node transport handle
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Create subscriber of thrust vectoring message
    if (_sdf->HasElement("vector_topic")) {
        const auto thrust_topic_ = this->sdf->Get<std::string>("vector_topic");
      this->thrust_val_sub_ = this->node_handle_->Subscribe("~/" + thrust_topic_, &ThrustVectoring::OnThrustVectoringMsgs, this);
      gzdbg << "Subscribing on ~/" << thrust_topic_ << std::endl;
    }

    // Create publisher for thrust force visual
    if (_sdf->HasElement("topic_name")) {
        const auto thrust_visual_topic_ = this->sdf->Get<std::string>("topic_name");
        thrust_visual_pub_ = this->node_handle_->Advertise<ssm_msgs::msgs::Force>("~/" + thrust_visual_topic_);
        gzdbg << "Publishing to ~/" << thrust_visual_topic_ << std::endl;
    }
}

void ThrustVectoring::OnUpdate()
{
    GZ_ASSERT(this->link, "Link was NULL");
    
    // calculate current time
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
    double cos_angle_to_rotate = z_vector.Dot(forwardI) / (z_vector.Length() * forwardI.Length());
    double angle_to_rotate = acos(cos_angle_to_rotate);

    // --> calculate the rotation axis to rotate so that the z vector located in the forward direction
    ignition::math::Vector3d axis_rotation = z_vector.Cross(forwardI);

    // --> create quaternion for the rotation using the axis rotation and the angle
    ignition::math::Quaternion quat_rot(axis_rotation, angle_to_rotate);

    // --> rotate the thrust vector to the forward direction in initial frame
    auto thrustVectorI = quat_rot.RotateVector(this->thrust_vector_value);
    thrustVectorI.Normalize();

    // calculate the thrust force with the thrust magnintude
    ignition::math::Vector3d thrust_force = this->thrust_magnitude * thrustVectorI;

    // Correct from inf and NaN
    thrust_force.Correct();
    this->thurst_origin.Correct();

    

    // apply forces at thrust origin
    this->link->AddForceAtRelativePosition(thrust_force, this->thurst_origin);
    
    // publishe the force vector for visualization
    if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
        // calculate the force origin relative to the link pose
        ignition::math::Vector3d relative_center = this->link->RelativePose().Pos() + this->thurst_origin;
        
        // the force vector for visualization is relative to the link frame, 
        // so we need to reverse the world pose rotation
        ignition::math::Vector3d force_vis = pose.Rot().RotateVectorReverse(thrust_force);

        relative_center.Correct();
        
        msgs::Vector3d* force_center_msg = new msgs::Vector3d;
        force_center_msg->set_x(relative_center.X());
        force_center_msg->set_y(relative_center.Y());
        force_center_msg->set_z(relative_center.Z());

        msgs::Vector3d* force_vector_msg = new msgs::Vector3d;
        force_vector_msg->set_x(force_vis.X());
        force_vector_msg->set_y(force_vis.Y());
        force_vector_msg->set_z(force_vis.Z());

        ssm_msgs::msgs::Force force_msg;
        force_msg.set_allocated_center(force_center_msg);
        force_msg.set_allocated_force(force_vector_msg);

        thrust_visual_pub_->Publish(force_msg);
        this->last_pub_time = current_time;
        ROS_WARN_STREAM("vectoring force " << thrust_force);
    }

}

void ThrustVectoring::OnThrustVectoringMsgs(const boost::shared_ptr<const ssm_msgs::msgs::SphericalVector> &msg)
{
    this->thrust_magnitude = msg->r();
    this->thrust_vector_value.X(cos(msg->phi())*sin(msg->theta()));
    this->thrust_vector_value.Y(sin(msg->phi())*sin(msg->theta()));
    this->thrust_vector_value.Z(cos(msg->theta()));    
}