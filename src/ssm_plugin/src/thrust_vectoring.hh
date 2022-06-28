#ifndef _THRUST_VECTORING_HH_
#define _THRUST_VECTORING_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include <ignition/math.hh>

#include "Force.pb.h"
#include "SphericalVector.pb.h"

namespace gazebo
{
    /// \brief A plugin that simulates simple thrust of a jet booster in one direction.
    class GAZEBO_VISIBLE ThrustVectoring : public ModelPlugin
    {
        /// \brief Constructor.
        public: ThrustVectoring();

        /// \brief Destructor.
        public: ~ThrustVectoring();

        // Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Callback for incomming Thrust Message.
        protected: void OnThrustVectoringMsgs(const boost::shared_ptr<const ssm_msgs::msgs::SphericalVector> &msg);

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to world.
        protected: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        /// \brief Pointer to link currently targeted by mud joint.
        protected: physics::LinkPtr link;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief The node handle to manage the message transport
        private: transport::NodePtr node_handle_;

        /// \brief The subscriber for the thrust value message
        private: transport::SubscriberPtr thrust_val_sub_;

        /// \brief The publisher of the thrust force vector for visualization
        private: transport::PublisherPtr thrust_visual_pub_;

        /// \brief The timer to count the published time
        private: common::Time last_pub_time;

        /// \brief Used if using additional namespace
        private: std::string namespace_;

        /// \brief The vector direction of the thrust
        protected: ignition::math::Vector3d thrust_vector_value;

        /// \brief The magnitude of the thrust
        protected: double thrust_magnitude;

        /// \brief Forward direction of the thrust
        protected: ignition::math::Vector3d forward_direction;
        
        /// \brief The origin of the thrust force
        protected: ignition::math::Vector3d thurst_origin;
    };
}

#endif