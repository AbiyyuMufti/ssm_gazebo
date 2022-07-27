#ifndef _POSE_PUBLISHER_HH_
#define _POSE_PUBLISHER_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include <ignition/math.hh>

#include "VectorVisual.pb.h"

namespace gazebo
{
    /// \brief A plugin that simulates simple thrust of a jet booster in one direction.
    class GAZEBO_VISIBLE PosePublisher : public ModelPlugin
    {
        /// \brief Constructor.
        public: PosePublisher();

        /// \brief Destructor.
        public: ~PosePublisher();

        // Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        /// \brief Callback for World Update events.
        protected: virtual void OnUpdate();

        /// \brief Connection to World Update events.
        protected: event::ConnectionPtr updateConnection;

        /// \brief Pointer to world.
        protected: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        protected: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        protected: physics::ModelPtr model;

        /// \brief Pointer to link which it's poses will be published.
        protected: physics::LinkPtr link;
        protected: std::string linkName;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        /// \brief The node handle to manage the message transport
        private: transport::NodePtr node_handle_;

        /// \brief The publisher of the thrust force vector for visualization
        private: transport::PublisherPtr pose_publisher_;

        /// \brief The timer to count the published time
        private: common::Time last_pub_time;

        /// \brief Used if using additional namespace
        private: std::string namespace_;
    };
}

#endif