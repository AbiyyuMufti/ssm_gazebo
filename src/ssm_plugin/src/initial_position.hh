#ifndef _INITIAL_POSITION_HH_
#define _INITIAL_POSITION_HH_

#include <string>
#include <vector>
#include <map>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include <ignition/math.hh>

namespace gazebo
{
    /// \brief A plugin that simulates simple thrust of a jet booster in one direction.
    class GAZEBO_VISIBLE InitialPosition : public ModelPlugin
    {
        /// \brief Constructor.
        public: InitialPosition();

        /// \brief Destructor.
        public: ~InitialPosition();
        
        /// \brief Callback for Simulation Reset.
        public: void Reset();

        // Documentation Inherited.
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

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
    };
}

#endif