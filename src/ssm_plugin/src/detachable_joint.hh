#ifndef _DETACHABLE_JOINT_HH_
#define _DETACHABLE_JOINT_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
    class GAZEBO_VISIBLE Detachables : public ModelPlugin
    {
        /// \brief Constructor.
        public: Detachables();

        /// \brief Destructor.
        public: ~Detachables();

        // Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback for World Update events.
        public: virtual void OnUpdate();

        /// \brief Callback for Simulation Reset.
        public: void Reset();

        /// \brief Callback for detaching command.
        public: void OnDetachMsgs(const boost::shared_ptr<const gazebo::msgs::Int> &msg);

        private: bool FindJoint(const std::string &_sdfParam, physics::JointPtr &_joint, std::string &jointName);

        private: bool FindLink(const std::string &_sdfParam, physics::LinkPtr &_link, std::string &linkName);

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief The node handle to manage the message transport
        private: transport::NodePtr node_handle_;

        /// \brief The subscriber for the detaching command messages
        private: transport::SubscriberPtr detach_subs_;

        private: transport::PublisherPtr visual_pub_;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief Pointer to model containing plugin.
        physics::ModelPtr model_;
        
        /// \brief Pointer to world.
        protected: physics::WorldPtr world;

        /// \brief Pointer to detachable link.
        physics::LinkPtr detachable_link_;

        /// \brief Name of detachable link.
        std::string detachable_link_name_;
        
        /// \brief Pointer to parent link which the detachable link are attached into
        physics::LinkPtr parent_link_;

        /// \brief Name of parent link which the detachable link are attach into.
        std::string parent_link_name_;

        /// \brief Pointer to detachable joint that connect parent and detachable link.
        physics::JointPtr detachable_joint_;

        /// \brief Name of detachable joint that connect parent and detachable link.
        std::string detachable_joint_name_;

        /// \brief Detaching force magnitude.
        double detach_acc;

        /// \brief Flag if link is detached or still atached.
        bool link_is_detached;

        /// \brief Detaching force magnitude.
        ignition::math::Vector3d detach_direction;

        /// \brief Used if using additional namespace
        private: std::string namespace_;

        private: common::Time time_after_detach;
        private: int counter;
    };

};

#endif // _DETACHABLE_JOINT_HH_