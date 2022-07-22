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
    // Forward declare private data class
    class AerodynamicPrivate;
    class AerodynamicLUT;
    class AerodynamicVector;
    struct AerodynamicCoefficients;

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
        public: virtual void OnUpdate();
        
        private: void ParseCoefficientTable();

        private: bool CalculateAerodynamicVectors();

        private: void CalculateAerodynamicAngles();

        private: void CalculateAerodynamicCoefficients();

        private: void CalculateAerodynamicForces();

        private: void PublishAeroForces();

        /// \brief Private data pointer
        private: std::unique_ptr<AerodynamicPrivate> dataPtr;

        /// \brief Table of aerodynamic coefficients
        private: std::vector<AerodynamicLUT> cTable;

        /// \brief Aerodynamics vectors (velocity, forces and moment)
        private: std::unique_ptr<AerodynamicVector> V;

        /// \brief Direction of Aerodynamics vectors in inertial frame
        private: std::unique_ptr<AerodynamicVector> I;

        private: std::unique_ptr<AerodynamicCoefficients> C;

        /// \brief Pointer to a node for communication
        private: transport::NodePtr node_handle_;
        
        /// \brief Connection to World Update events.
        private: event::ConnectionPtr updateConnection;
        
        /// \brief Pointer to publish aerodynamic vectors (forces and moment)
        private: transport::PublisherPtr vectors_pub_;

        /// \brief Keep track of publish time
        private: common::Time last_pub_time;

        /// \brief Topics namespaces if necesarry
        private: std::string namespace_;

        ///
        private: int ucase;

    };
};

#endif