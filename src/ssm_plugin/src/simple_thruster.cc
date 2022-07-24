
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
#include "simple_thruster.hh"
#include <boost/bind.hpp>

#include <ros/ros.h>


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SimpleThruster);

//////////////////////////////////
SimpleThruster::SimpleThruster()
{
    this->forward_direction = ignition::math::Vector3d(1, 0, 0);
    this->thurst_origin = ignition::math::Vector3d(0, 0, 0);
    this->thrust_magnitude_ = 0.0;
}

//////////////////////////////////
SimpleThruster::~SimpleThruster()
{

}

void SimpleThruster::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Start Loading the plugin
    ROS_WARN("Loading simple thruster");

    // Load Model
    GZ_ASSERT(_model, "SimpleThruster _model pointer is NULL");
    this->model = _model;

    // Load SDF
    GZ_ASSERT(_sdf, "SimpleThruster _sdf pointer is NULL");
    this->sdf = _sdf;

    // Load World
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "SimpleThruster world pointer is NULL");

    // Load Physics
    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "SimpleThruster physics pointer is NULL");
    
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
            << "The SimpleThruster will not generate forces\n";
        }
        else
        {
            ROS_WARN("Link connected");
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&SimpleThruster::OnUpdate, this));
        }
    }

    // Get the forward direction of thrust relative to the link object
    if (_sdf->HasElement("forward_dir"))
        this->forward_direction = _sdf->Get<ignition::math::Vector3d>("forward_dir");
    this->forward_direction.Normalize();

    // Get the origin of thrust relative to the link object
    if (_sdf->HasElement("thrust_origin"))
        this->thurst_origin = _sdf->Get<ignition::math::Vector3d>("thrust_origin");

    // Use Additional Namespace if neede
    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_simple_thruster] Please specify a robotNamespace.\n";
    }

    // Gazebo node transport handle
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    // Create subscriber for thrust magnitude
    if (_sdf->HasElement("thrust_topic")) {
        const auto thrust_topic_ = this->sdf->Get<std::string>("thrust_topic");
        this->thrust_val_sub_ = this->node_handle_->Subscribe("~/" + thrust_topic_, &SimpleThruster::OnThrustMsgs, this);
        gzdbg << "Subscribing on ~/" << thrust_topic_ << std::endl;
    }

    // Create publisher for thrust force visual
    if (_sdf->HasElement("topic_name")) {
        const auto thrust_visual_topic_ = this->sdf->Get<std::string>("topic_name");
        thrust_visual_pub_ = this->node_handle_->Advertise<ssm_msgs::msgs::VectorVisual>("~/" + thrust_visual_topic_);
        gzdbg << "Publishing to ~/" << thrust_visual_topic_ << std::endl;
    }

}

void SimpleThruster::OnUpdate()
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

    // calculate the thrust force with the thrust magnintude
    ignition::math::Vector3d thrust_force = this->thrust_magnitude_ * forwardI;

    // Correct from inf and NaN
    thrust_force.Correct();
    this->thurst_origin.Correct();
    
    // apply forces at thrust origin
    this->link->AddForceAtRelativePosition(thrust_force, this->thurst_origin);
    
    if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
        
        // calculate the force origin relative to the link pose
        ignition::math::Vector3d relative_center = this->link->RelativePose().Pos() + this->thurst_origin;
        
        // the force vector for visualization is relative to the link frame, so need to reverse rotate
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
        
        ssm_msgs::msgs::VectorVisual force_msg;

        force_msg.set_allocated_center(force_center_msg);
        force_msg.set_allocated_vector(force_vector_msg);

        thrust_visual_pub_->Publish(force_msg);

        this->last_pub_time = current_time;
    }

}

void SimpleThruster::OnThrustMsgs(const boost::shared_ptr<const gazebo::msgs::Double> &msg)
{
    this->thrust_magnitude_ = msg->data();
    // ROS_WARN_STREAM("thrust magnitude " << this->thrust_magnitude_);
}