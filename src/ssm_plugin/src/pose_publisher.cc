
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
#include "pose_publisher.hh"
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PosePublisher)

//////////////////////////////////
PosePublisher::PosePublisher()
{

}

//////////////////////////////////
PosePublisher::~PosePublisher()
{

}

void PosePublisher::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "PosePublisher _model pointer is NULL");
    GZ_ASSERT(_sdf, "PosePublisher _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "PosePublisher world pointer is NULL");

    this->physics = this->world->Physics();
    this->last_pub_time = this->world->SimTime();

    GZ_ASSERT(this->physics, "PosePublisher physics pointer is NULL");

    GZ_ASSERT(_sdf, "PosePublisher _sdf pointer is NULL");

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        // GZ_ASSERT(elem, "Element link_name doesn't exist!");
        this->linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(linkName);
        // GZ_ASSERT(this->link, "Link was NULL");

        if (!this->link)
        {
            gzerr << "Link with name[" << linkName << "] not found. "
            << "The PosePublisher will not publish dynamic states\n";
        }
        else
        {
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PosePublisher::OnUpdate, this));
        }
    }

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        // namespace_ = "";
        gzerr << "[" << linkName << " - Dynamic Publisher] Please specify a robotNamespace.\n";
    }
    
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("topic_name")) {
        const auto pose_pub_topic = this->sdf->Get<std::string>("topic_name");
        
        this->dynamic_state_pub_ = this->node_handle_->Advertise<ssm_msgs::msgs::DynamicState>("~/" + pose_pub_topic);
        gzdbg << "Publishing to ~/" << pose_pub_topic << std::endl;
    } 
    else
    {
        gzerr << "Cannot Publish Dynamic State of Link "  << linkName << " because topic_name wasn't spesified";
    }
    

    // TODO: Also ROS PUBLISHER


}

void PosePublisher::OnUpdate()
{
    GZ_ASSERT(this->link, "Link was NULL");
    
    const common::Time current_time = this->world->SimTime();
    // const double dt = (current_time - this->last_pub_time).Double();
    
    // Retrieve Pose in world frame
    ignition::math::Pose3d model_pose = this->link->WorldPose();
    // Retrieve Linear Velocity in World Frame
    ignition::math::Vector3d model_lin_vel = this->link->WorldLinearVel();
    // Retrieve Angular Velocity in World Frame
    ignition::math::Vector3d model_ang_vel = this->link->WorldAngularVel();
    // Retrieve Linear Acceleration in World Frame
    ignition::math::Vector3d model_lin_acc = this->link->WorldAngularAccel();
    // Retrieve Angular Acceleration in World Frame
    ignition::math::Vector3d model_ang_acc = this->link->WorldAngularAccel();

    ssm_msgs::msgs::DynamicState State;
    
    
    msgs::Vector3d *linVel = new msgs::Vector3d;
    linVel->set_x(model_lin_vel.X());
    linVel->set_y(model_lin_vel.Y());
    linVel->set_z(model_lin_vel.Z());
    State.set_allocated_linearvelocity(linVel);

    msgs::Vector3d *angVel = new msgs::Vector3d;
    angVel->set_x(model_ang_vel.X());
    angVel->set_y(model_ang_vel.Y());
    angVel->set_z(model_ang_vel.Z());      
    State.set_allocated_angluarvelocity(angVel);

    msgs::Vector3d *linAcc = new msgs::Vector3d;
    linAcc->set_x(model_lin_vel.X());
    linAcc->set_y(model_lin_vel.Y());
    linAcc->set_z(model_lin_vel.Z());
    State.set_allocated_linearacceleration(linAcc);

    msgs::Vector3d *angAcc = new msgs::Vector3d;
    angAcc->set_x(model_ang_vel.X());
    angAcc->set_y(model_ang_vel.Y());
    angAcc->set_z(model_ang_vel.Z());
    State.set_allocated_angluaracceleration(angAcc);
    
    model_pose.Rot();
    msgs::Vector3d *ps;
    ps->set_x(model_pose.Pos().X());
    ps->set_y(model_pose.Pos().Y());
    ps->set_z(model_pose.Pos().Z());

    msgs::Quaternion *rot;
    rot->set_w(model_pose.Rot().W());
    rot->set_x(model_pose.Rot().X());
    rot->set_y(model_pose.Rot().Y());
    rot->set_z(model_pose.Rot().Z());
    
    msgs::Pose* pose = new msgs::Pose;
    pose->set_name(this->linkName);
    pose->set_allocated_position(ps);
    pose->set_allocated_orientation(rot);
    State.set_allocated_pose(pose);

    msgs::Time* my_time = new msgs::Time;
    my_time->set_nsec(current_time.nsec);
    my_time->set_sec(current_time.sec);
    State.set_allocated_time(my_time);

    this->dynamic_state_pub_->Publish(State);
    // this->last_pub_time = current_time;

}