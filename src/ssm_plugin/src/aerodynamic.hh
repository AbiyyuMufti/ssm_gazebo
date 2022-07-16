#ifndef _AERODYNAMIC_HH_
#define _AERODYNAMIC_HH_

#include <string>
#include <vector>
#include <boost/bind.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include <ignition/math.hh>


namespace gazebo
{
      /// \brief A plugin that simulates lift and drag.
    class GAZEBO_VISIBLE Aerodynamic : public ModelPlugin
    {
        /// \brief Constructor.
        public: Aerodynamic();

        /// \brief Destructor.
        public: ~Aerodynamic();

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

        /// \brief air density
        /// at 25 deg C it's about 1.1839 kg/m^3
        /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
        protected: double rho;

        /// \brief if the shape is aerodynamically radially symmetric about
        /// the forward direction. Defaults to false for wing shapes.
        /// If set to true, the upward direction is determined by the
        /// angle of attack.
        protected: bool radialSymmetry;

        /// \brief effective planeform surface area
        // protected: double area;

        /// \brief angle of sweep
        protected: double sweep;

        /// \brief angle of attack
        protected: double alpha;

        /// \brief center of pressure in link local coordinates
        protected: ignition::math::Vector3d cp;

        /// \brief Normally, this is taken as a direction parallel to the chord
        /// of the airfoil in zero angle of attack forward flight.
        protected: ignition::math::Vector3d forward;

        /// \brief A vector in the lift/drag plane, perpendicular to the forward
        /// vector. Inflow velocity orthogonal to forward and upward vectors
        /// is considered flow in the wing sweep direction.
        protected: ignition::math::Vector3d upward;

        /// \brief Pointer to link currently targeted by mud joint.
        protected: physics::LinkPtr link;

        /// \brief SDF for this plugin;
        protected: sdf::ElementPtr sdf;

        private: transport::NodePtr node_handle_;
        // private: transport::SubscriberPtr wind_sub_;
        private: transport::PublisherPtr lift_force_pub_;
        private: transport::PublisherPtr drag_force_pub_;
        private: transport::PublisherPtr velocity_pub_;
        private: transport::PublisherPtr vectors_pub_;
        private: transport::PublisherPtr velocity_in_ld_pub_;
        // private: transport::PublisherPtr lift_force_pub_;

        private: common::Time last_pub_time;
        private: msgs::Factory msg_factory_;
        private: std::string namespace_;

        std::vector<std::vector<double>> cn_table;
        std::vector<std::vector<double>> cm_table;
        std::vector<std::vector<double>> ca_table;
        std::vector<std::vector<double>> cl_table;
        std::vector<std::vector<double>> cd_table;
        std::vector<std::vector<double>> cp_table;
        std::vector<double> alpha_table;
        std::vector<double> mach_table;

        /// \brief effective planeform surface area
        protected: double sref;

        protected: double lref;

    };
};

namespace tools{
    // Helper function to publish force message from the publisher side
    void publish_force(const ignition::math::Vector3d& force, const ignition::math::Vector3d& center, const gazebo::transport::PublisherPtr& publisher);
};


#endif