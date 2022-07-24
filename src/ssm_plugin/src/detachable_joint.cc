

#include <algorithm>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <boost/bind.hpp>
#include <ros/ros.h>
#include "detachable_joint.hh"
#include "VectorVisual.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Detachables);
        
Detachables::Detachables()
{
    this->detach_acc = 10;
    this->detach_direction = ignition::math::Vector3d(0,0,1);
}

Detachables::~Detachables()
{

}

bool Detachables::FindJoint(const std::string &_sdfParam, physics::JointPtr &_joint, std::string &jointName)
{
    // Check if the required element is found
    if (!this->sdf->HasElement(_sdfParam))
    {
        gzlog << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    jointName = this->sdf->Get<std::string>(_sdfParam);
    _joint = this->model_->GetJoint(jointName);
    if (!_joint)
    {
        gzlog << "Failed to find joint [" << jointName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

bool Detachables::FindLink(const std::string &_sdfParam, physics::LinkPtr &_link, std::string &linkName)
{
    // Check if the required element is found
    if (!this->sdf->HasElement(_sdfParam))
    {
        gzlog << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    linkName = this->sdf->Get<std::string>(_sdfParam);
    _link = this->model_->GetLink(linkName);
    if (!_link)
    {
        gzlog << "Failed to find joint [" << linkName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}


void Detachables::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Load SDF
    GZ_ASSERT(_sdf, "_sdf pointer is NULL");
    this->sdf = _sdf;

    // Load Model
    GZ_ASSERT(_model, "_model pointer is NULL");
    this->model_ = _model;

    // Load World
    this->world = this->model_->GetWorld();
    GZ_ASSERT(this->world, "Detachable Joint world pointer is NULL");

    if (!this->FindLink("parent_link", this->parent_link_, this->parent_link_name_))
    {
        gzerr << "Detachable Joint Plugin cannot be used" << std::endl;
    }
    
    if (!this->FindLink("link_to_detach", this->detachable_link_, this->detachable_link_name_))
    {
        gzerr << "Detachable Joint Plugin cannot be used" << std::endl;
    }

    if (!this->FindJoint("joint_to_detach", this->detachable_joint_, this->detachable_joint_name_))
    {
        gzerr << "Detachable Joint Plugin cannot be used" << std::endl;
    }

    this->link_is_detached = false;

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Detachables::OnUpdate, this));

    // Get the detach force direction if provided
    if (_sdf->HasElement("detach_direction"))
        this->detach_direction = _sdf->Get<ignition::math::Vector3d>("detach_direction");
    this->detach_direction.Normalize();
    ROS_WARN_STREAM(" DIR " << this->detachable_link_name_ << " : " << this->detach_direction);

        // Get the detach force direction if provided
    if (_sdf->HasElement("detach_acc"))
        this->detach_acc = _sdf->Get<double>("detach_acc");

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        namespace_ = "";
        // gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
    }

    // Gazebo node transport handle
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("detach_topic"))
    {
        const auto detach_topic_ = _sdf->Get<std::string>("detach_topic");
        this->detach_subs_ = this->node_handle_->Subscribe("~/" + detach_topic_, &Detachables::OnDetachMsgs, this);
        gzdbg << "Subscribing on ~/" << detach_topic_ << std::endl;
    }
    counter = 0;

    // Create publisher for thrust force visual
    if (_sdf->HasElement("topic_name")) {
        const auto visual_topic = this->sdf->Get<std::string>("topic_name");
        visual_pub_ = this->node_handle_->Advertise<ssm_msgs::msgs::VectorVisual>("~/" + visual_topic);
        gzdbg << "Publishing to ~/" << visual_topic << std::endl;
    }
}

void Detachables::OnUpdate()
{
    if (this->link_is_detached)
    {
        
        if ((this->world->SimTime() - time_after_detach) < 1.0)
        {
            double mass = this->detachable_link_->GetInertial()->Mass();
            ignition::math::Vector3d force = this->detach_direction * mass * this->detach_acc;
            this->detachable_link_->AddForce(force);
            
            ROS_INFO_STREAM(" " << this->detachable_link_name_ << " detaching " <<  counter);
            counter++;
        
            if (this->sdf->HasElement("topic_name"))
            {
                        // calculate the force origin relative to the link pose
                ignition::math::Vector3d relative_center = this->detachable_link_->RelativePose().Pos(); //ignition::math::Vector3d(0,0,0);
                
                // the force vector for visualization is relative to the link frame, so need to reverse rotate
                ignition::math::Vector3d force_vis(force);
                force_vis.Normalize();
                force_vis = 1000*force_vis;
                force_vis.Correct();
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

                visual_pub_->Publish(force_msg);
            }
        

        } 
    }
}

void Detachables::Reset()
{
    this->parent_link_->Reset();
    this->parent_link_->ResetPhysicsStates();
    this->detachable_link_->Reset();
    this->detachable_link_->ResetPhysicsStates();
    this->detachable_joint_->Reset();
    this->detachable_joint_->Attach(this->parent_link_, this->detachable_link_);
    this->link_is_detached = false;
    counter = 0;
}

void Detachables::OnDetachMsgs(const boost::shared_ptr<const gazebo::msgs::Int> &msg)
{
    if (this->link_is_detached)
    {
        return;
    }
    if (msg->data() >= 1)
    {
        this->detachable_joint_->Detach();
        this->link_is_detached = true;
        this->time_after_detach = this->world->SimTime();
    }
    
    // this->detachable_joint_->Fini();
    // this->parent_link_->Fini();
    // this->detachable_link_->Fini();

    // this->model_->RemoveJoint(this->detachable_joint_name_);
    // this->model_->RemoveLink(this->detachable_link_name_);
    // this->model_->Fini();
}

