
#include <algorithm>
#include <string>
#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "initial_position.hh"
#include <boost/bind.hpp>

#include <ros/ros.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(InitialPosition)

//////////////////////////////////
InitialPosition::InitialPosition()
{

}

//////////////////////////////////
InitialPosition::~InitialPosition()
{

}


/*

*/
void InitialPosition::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    gzwarn << "SETTING INITIAL POSITION" << std::endl;

    std::map<std::string, double> positions;
    if(_sdf->HasElement("joint"))
    {
        sdf::ElementPtr joint_elem = _sdf->GetElement("joint");
        while (joint_elem)
        {
            auto joint = joint_elem->GetAttribute("name");
            std::string joint_name;
            joint->Get<std::string>(joint_name);
            
            auto pos = joint_elem->GetValue();
            double initial_position;
            pos->Get<double>(initial_position);

            positions.insert(std::pair<std::string,double>(joint_name, initial_position));

            joint_elem = joint_elem->GetNextElement("joint");
        }
        
        
    }

    for (physics::JointPtr joint : _model->GetJoints()){
        // continue if given joint does not exist
        if (positions.count(joint->GetName()) == 0){
            continue;
        } 
        // set the initial position
        gzdbg << "Set Initial Position for Joint: " << joint->GetName() << std::endl;
        joint->SetPosition(0, positions[joint->GetName()]);
    }    
}

void InitialPosition::Reset()
{

}