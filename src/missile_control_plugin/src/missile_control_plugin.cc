#include "missile_control_plugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(MissileControl)

MissileControl::MissileControl()
{
    this->booster_is_attached_ = true;
    this->detaching_cmd = false;
    this->thrust_force_.Set(0,0,0);
    this->thrust_torque_.Set(0,0,0);
    this->with_booster = true;
}
MissileControl::~MissileControl()
{
    
}

void MissileControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "MissileControl _model pointer is NULL");
    GZ_ASSERT(_sdf, "MissileControl _sdf pointer is NULL");

    this->model_ = _model;

    if (!this->FindLink("missile_body", _sdf, this->missile_body_link_, this->missile_body_link_name_))
    {
        ROS_ERROR("Missile Body Link Missing");
        gzerr<<"Missile Body Link Missing"<<std::endl; 
        return;
    }
    if (!this->FindJoint("booster_joint", _sdf, this->booster_joint_, this->booster_joint_name_))
    {
        this->with_booster = false;
        ROS_WARN("Missile is without detachable booster joint");
        this->booster_joint_ = nullptr;
        gzwarn<<"Missile is without detachable booster joint"<<std::endl;
    }
    if (!this->FindLink("booster", _sdf, this->booster_link_, this->booster_link_name_))
    {
        this->booster_link_ = nullptr;
        ROS_WARN("Missile is without detachable booster link");
        gzwarn<<"Missile is without detachable booster link";
    }
    if (_sdf->HasElement("forward_dir"))
        {this->forward = _sdf->Get<ignition::math::Vector3d>("forward_dir");}
    else{
        this->forward.Set(0,0,1);
    }
    // Controller time control.
    this->lastControllerUpdateTime = this->model_->GetWorld()->SimTime();

    // Listen to the update event. This event is broadcast every simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MissileControl::OnUpdate, this, std::placeholders::_1));

    gzlog << "Missile ready to fly. The force will be with you" << std::endl;
    ROS_WARN("Loaded MissileControl Plugin with parent...%s", this->model_->GetName().c_str());

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "exocet_missile_ctrl",
            ros::init_options::NoSigintHandler);
    }

    // // Create our ROS node. This acts in a similar manner to the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("exocet_missile_ctrl"));


    // // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
    ros::SubscribeOptions::create<exocet_msgs::ExocetCMD>(
        "ExocetCMD", 
        1,
        boost::bind(&MissileControl::OnExocetCommand, this, _1),
        ros::VoidPtr(), &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);
    
    // // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&MissileControl::QueueThread, this));

}

bool MissileControl::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string &jointName)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzlog << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    jointName = _sdf->Get<std::string>(_sdfParam);
    _joint = this->model_->GetJoint(jointName);
    if (!_joint)
    {
        gzlog << "Failed to find joint [" << jointName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

bool MissileControl::FindLink(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::LinkPtr &_link, std::string &linkName)
{
    // Check if the required element is found
    if (!_sdf->HasElement(_sdfParam))
    {
        gzlog << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
        return false;
    }

    // Load the joint name
    linkName = _sdf->Get<std::string>(_sdfParam);
    _link = this->model_->GetLink(linkName);
    if (!_link)
    {
        gzlog << "Failed to find joint [" << linkName << "] aborting plugin load." << std::endl;
        return false;
    }
    return true;    
}

void MissileControl::OnUpdate(const common::UpdateInfo &_info)
{   
    this->applyThrust();
    if(this->with_booster){
        if (this->booster_is_attached_ && this->detaching_cmd)
        {
            this->detachingBooster();
            // if (seconds_since_last_update > 5)
            // {

            // }
        }
    }
}

void MissileControl::OnExocetCommand(const exocet_msgs::ExocetCMDConstPtr &_msg)
{
    double val = _msg->propulsion.force.z;

    this->thrust_force_ = this->forward * val;
    // ROS_WARN_STREAM("forward_dir " << this->forward);
    // ROS_WARN_STREAM("Force " << this->thrust_force_);
    // this->thrust_force_.Set(0,0,val);
    this->detaching_cmd = _msg->detach_booster;
}

void MissileControl::QueueThread()
{
    static const double timeout = 0.01;
    while (this->rosNode->ok())
    {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

void MissileControl::Reset()
{
    if(this->with_booster && !this->booster_is_attached_)
    {
        this->booster_link_->Reset();
        this->booster_link_->ResetPhysicsStates();
        this->booster_joint_->Reset();
        this->booster_joint_->Attach(this->missile_body_link_, this->booster_link_);
        this->booster_is_attached_ = true;
    }
    this->missile_body_link_->Reset();
    this->missile_body_link_->ResetPhysicsStates();
    
}

void MissileControl::applyThrust()
{
    if(this->with_booster && this->booster_is_attached_){
        this->booster_link_->AddRelativeForce(this->thrust_force_);
    } else
    {
        this->missile_body_link_->AddRelativeForce(this->thrust_force_);
    }
}

void MissileControl::detachingBooster()
{
    if(this->with_booster){
        
        this->booster_joint_->Detach();
        this->missile_body_link_->RemoveChildJoint(this->booster_joint_name_);
        this->missile_body_link_->RemoveChild(this->booster_link_name_);
        this->booster_is_attached_ = false;
        // this->model_->RemoveLink(this->booster_link_name_);
        // this->model_->RemoveChild(this->booster_link_);
        this->model_->RemoveJoint(this->booster_joint_name_);
        this->model_->Update();
        ROS_WARN("Detaching Booster");
    }
    
}