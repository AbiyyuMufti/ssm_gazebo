
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
            << "The PosePublisher will not generate forces\n";
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
        namespace_ = "";
        // gzerr << "[gazebo_simple_thruster] Please specify a robotNamespace.\n";
    }

    if (_sdf->HasElement("topic_name")) {
        const auto pose_pub_topic = this->sdf->Get<std::string>("topic_name");
        
        this->pose_publisher_ = this->node_handle_->Advertise<gazebo::msgs::PoseStamped>("~/" + pose_pub_topic);
        gzdbg << "Publishing to ~/" << pose_pub_topic << std::endl;
    }


}

void PosePublisher::OnUpdate()
{
    GZ_ASSERT(this->link, "Link was NULL");
    
    const common::Time current_time = this->world->SimTime();
    const double dt = (current_time - this->last_pub_time).Double();
    // get link actual pose
    
    
    if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name")) {
        ignition::math::Pose3d model_pose = this->link->WorldPose();
        msgs::Pose* pose = new msgs::Pose;
        msgs::Time* my_time = new msgs::Time;

        pose->set_name(this->linkName);
        
        model_pose.Rot();
        gazebo::msgs::Vector3d *ps;
        ps->set_x(model_pose.Pos().X());
        ps->set_y(model_pose.Pos().Y());
        ps->set_z(model_pose.Pos().Z());

        gazebo::msgs::Quaternion *rot;
        rot->set_w(model_pose.Rot().W());
        rot->set_x(model_pose.Rot().X());
        rot->set_y(model_pose.Rot().Y());
        rot->set_z(model_pose.Rot().Z());

        pose->set_allocated_position(ps);
        pose->set_allocated_orientation(rot);
        
        my_time->set_nsec(current_time.nsec);
        my_time->set_sec(current_time.sec);

        msgs::PoseStamped msg_pose;
        msg_pose.set_allocated_time(my_time);
        msg_pose.set_allocated_pose(pose);

        this->pose_publisher_->Publish(msg_pose);
        this->last_pub_time = current_time;
    }

}