#include <algorithm>
#include <string>
#include <vector>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "aerodynamic.hh"

namespace gazebo
{
    enum axis_type{Body, Wind, Both};

    class AerodynamicLUT
    {
        public:
            std::vector<std::vector<double>> cn_table;
            std::vector<std::vector<double>> cm_table;
            std::vector<std::vector<double>> ca_table;
            std::vector<std::vector<double>> cl_table;
            std::vector<std::vector<double>> cd_table;
            std::vector<double> alpha_table;
            std::vector<double> mach_table;
            std::vector<double> control_joint_rate;
            axis_type a_type;
            int a_case;
    };


    class AerodynamicVector
    {
        public: ignition::math::Vector3d velocity;
        public: ignition::math::Vector3d vel_in_lift_drag_plane;
        public: ignition::math::Vector3d normal;
        public: ignition::math::Vector3d axial;
        public: ignition::math::Vector3d lift;
        public: ignition::math::Vector3d drag;
        public: ignition::math::Vector3d moment;
    };


    struct AerodynamicCoefficients
    {
        double cn = 0;
        double cm = 0;
        double ca = 0;
        double cl = 0;
        double cd = 0;
        double rate_cl = 0;
    };


    class AerodynamicPrivate
    {
        /// \brief Pointer to world.
        public: physics::WorldPtr world;

        /// \brief Pointer to physics engine.
        public: physics::PhysicsEnginePtr physics;

        /// \brief Pointer to model containing plugin.
        public: physics::ModelPtr model;

        /// \brief Pointer to link currently targeted by mud joint.
        public: physics::LinkPtr link;

        /// \brief Pointer to a joint that actuates a control surface for this lifting body
        public: physics::JointPtr controlJoint;

        /// \brief Link pose in world frame
        public: ignition::math::Pose3d pose;

        /// \brief SDF for this plugin;
        public: sdf::ElementPtr sdf;

        /// \brief air density
        /// at 25 deg C it's about 1.1839 kg/m^3
        /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
        public: double rho;

        /// \brief if the shape is aerodynamically radially symmetric about
        /// the forward direction. Defaults to false for wing shapes.
        /// If set to true, the upward direction is determined by the
        /// angle of attack.
        public: bool radialSymmetry;

        /// \brief effective planeform surface area
        public: double sref;

        /// \brief effective planeform lenght
        public: double lref;

        /// \brief angle of sweep
        public: double sweep;

        /// \brief angle of attack
        public: double alpha;

        public: double controlAngle;

        /// \brief angle of attack
        public: double mach;

        /// \brief dynamic pressure
        public: double q;

        /// \brief center of pressure in link local coordinates
        public: ignition::math::Vector3d cp;

        /// \brief Normally, this is taken as a direction parallel to the chord
        /// of the airfoil in zero angle of attack forward flight.
        public: ignition::math::Vector3d forward;

        /// \brief A vector in the lift/drag plane, perpendicular to the forward
        /// vector. Inflow velocity orthogonal to forward and upward vectors
        /// is considered flow in the wing sweep direction.
        public: ignition::math::Vector3d upward;

        /// \brief Number of aerodynamic cases
        public: int number_of_cases;
    };

};