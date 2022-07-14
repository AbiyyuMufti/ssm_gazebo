#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "aerodynamic.hh"

#include <ros/ros.h>

#include "Force.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Aerodynamic);

/////////////////////////////////////////////////
Aerodynamic::Aerodynamic()
{
    this->rho = 1.2041;
    this->cp = ignition::math::Vector3d(0, 0, 0);
    this->forward = ignition::math::Vector3d(1, 0, 0);
    this->upward = ignition::math::Vector3d(0, 0, 1);
    this->area = 1.0;
    this->alpha = 0.0;
    this->sweep = 0.0;
    this->radialSymmetry = false;    
}

/////////////////////////////////////////////////
Aerodynamic::~Aerodynamic()
{
}


/////////////////////////////////////////////////
void Aerodynamic::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "Aerodynamic _model pointer is NULL");
    GZ_ASSERT(_sdf, "Aerodynamic _sdf pointer is NULL");
    this->model = _model;
    this->sdf = _sdf;
    
    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "Aerodynamic world pointer is NULL");

    this->physics = this->world->Physics();
    this->last_pub_time = this->world->SimTime();

    GZ_ASSERT(this->physics, "Aerodynamic physics pointer is NULL");
    GZ_ASSERT(_sdf, "Aerodynamic _sdf pointer is NULL");

    if (_sdf->HasElement("radial_symmetry"))
        this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");
    
    if (_sdf->HasElement("cp"))
        this->cp = _sdf->Get<ignition::math::Vector3d>("cp");

    // blade forward (-drag) direction in link frame
    if (_sdf->HasElement("forward"))
        this->forward = _sdf->Get<ignition::math::Vector3d>("forward");
    this->forward.Normalize();

    // blade upward (+lift) direction in link frame
    if (_sdf->HasElement("upward"))
        this->upward = _sdf->Get<ignition::math::Vector3d>("upward");
    this->upward.Normalize();

    if (_sdf->HasElement("area"))
        this->area = _sdf->Get<double>("area");

    if (_sdf->HasElement("air_density"))
        this->rho = _sdf->Get<double>("air_density");

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        // GZ_ASSERT(elem, "Element link_name doesn't exist!");
        std::string linkName = elem->Get<std::string>();
        this->link = this->model->GetLink(linkName);
        // GZ_ASSERT(this->link, "Link was NULL");
        if (!this->link)
        {
            gzerr << "Link with name[" << linkName << "] not found. "
            << "The Aerodynamic will not generate forces\n";
        }
        else
        {
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Aerodynamic::OnUpdate, this));
        }
    }

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
    }

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("topic_name")) {
        const auto lift_force_topic = this->sdf->Get<std::string>("topic_name");
        lift_force_pub_ = node_handle_->Advertise<ssm_msgs::msgs::Force>("~/" + lift_force_topic);
        gzdbg << "Publishing to ~/" << lift_force_topic << std::endl;
    }
    mach_table.assign({0.2, 0.4, 0.5, 0.6, 0.7, 0.75, 0.8, 0.93});
    alpha_table.assign({.00, 4.00, 8.00, 12.00, 16.00, 20.00, 24.00, 28.00});

    // mach 0.2
    cn_table.push_back({0.000, 1.037, 2.115, 3.322, 4.383, 5.449, 6.445, 7.475});
    cm_table.push_back({0.000, -0.222, -0.289, -0.558, -0.080, 0.688, 1.595, 2.038});
    ca_table.push_back({0.266, 0.249, 0.194, 0.148, 0.134, 0.160, 0.128, 0.050});
    cp_table.push_back({-0.295, -0.214, -0.136, -0.168, -0.018, 0.126, 0.247, 0.273});
    cl_table.push_back({.000, 1.017, 2.067, 3.218, 4.176, 5.066, 5.836, 6.576});
    cd_table.push_back({.266, .321, .487, .835, 1.337, 2.014, 2.738, 3.553});

    // mach 0.4
    cn_table.push_back({0.000, 1.051, 2.134, 3.337, 4.373, 5.448, 6.500, 7.626});
    cm_table.push_back({0.000, -0.250, -0.333, -0.604, -0.055, 0.655, 1.426, 1.835});
    ca_table.push_back({0.263, 0.246, 0.193, 0.147, 0.133, 0.157, 0.121, 0.037});
    cp_table.push_back({-0.323, -0.238, -0.156, -0.181, -0.013, 0.120, 0.219, 0.241});
    cl_table.push_back({.000, 1.031, 2.087, 3.233, 4.167, 5.066, 5.889, 6.716});
    cd_table.push_back({.263, .318, .488, .838, 1.333, 2.011, 2.754, 3.613});

    // mach 0.5
    cn_table.push_back({0.000, 1.070, 2.169, 3.390, 4.433, 5.517, 6.603, 7.866});
    cm_table.push_back({0.000, -0.276, -0.389, -0.727, -0.210, 0.446, 1.204, 1.690});
    ca_table.push_back({0.261, 0.243, 0.187, 0.141, 0.127, 0.152, 0.116, 0.025});
    cp_table.push_back({-0.339, -0.258, -0.180, -0.214, -0.047, 0.081, 0.182, 0.215});
    cl_table.push_back({.000, 1.051, 2.121, 3.286, 4.227, 5.132, 5.986, 6.934});
    cd_table.push_back({.261, .317, .487, .842, 1.344, 2.030, 2.791, 3.715});

    // mach 0.6
    cn_table.push_back({0.000, 1.093, 2.208, 3.444, 4.494, 5.607, 6.812, 8.206});
    cm_table.push_back({0.000, -0.305, -0.444, -0.825, -0.337, 0.307, 1.051, 1.581});
    ca_table.push_back({0.259, 0.240, 0.183, 0.135, 0.121, 0.148, 0.108, 0.011});
    cp_table.push_back({-0.359, -0.279, -0.201, -0.239, -0.075, 0.055, 0.154, .193});
    cl_table.push_back({.000, 1.074, 2.161, 3.341, 4.287, 5.219, 6.179, 7.240});
    cd_table.push_back({.259, .316, .488, .848, 1.355, 2.056, 2.869, 3.862});

    // mach 0.7
    cn_table.push_back({0.000, 1.101, 2.218, 3.456, 4.531, 5.736, 7.029, 8.570});
    cm_table.push_back({0.000, -0.320, -0.476, -0.877, -0.474, 0.203, 0.845, 1.353});
    ca_table.push_back({0.257, 0.238, 0.180, 0.132, 0.119, 0.144, 0.101, -0.002});
    cp_table.push_back({-0.369, -0.291, -0.214, -0.254, -0.105, 0.035, 0.120, 0.158});
    cl_table.push_back({.000, 1.081, 2.172, 3.353, 4.323, 5.341, 6.381, 7.568});
    cd_table.push_back({.257, .314, .487, .848, 1.363, 2.097, 2.951, 4.021});

    // mach 0.75
    cn_table.push_back({0.000, 1.118, 2.250, 3.503, 4.607, 5.853, 7.206, 8.818});
    cm_table.push_back({0.000, -0.345, -0.528, -0.974, -0.628, 0.040, 0.636, 1.106});
    ca_table.push_back({0.256, 0.236, 0.178, 0.129, 0.116, 0.141, 0.097, -0.009});
    cp_table.push_back({-0.382, -0.308, -0.235, -0.278, -0.136, 0.007, 0.088, 0.125});
    cl_table.push_back({.000, 1.099, 2.204, 3.399, 4.397, 5.452, 6.543, 7.790});
    cd_table.push_back({.256, .314, .489, .855, 1.382, 2.135, 3.020, 4.132});

    // mach 0.80
    cn_table.push_back({0.000, 1.148, 2.324, 3.656, 4.850, 6.177, 7.617, 9.288});
    cm_table.push_back({0.000, -0.366, -0.575, -1.115, -0.810, -0.218, 0.398, 0.701});
    ca_table.push_back({0.271, 0.251, 0.189, 0.137, 0.123, 0.151, 0.107, -0.001});
    cp_table.push_back({-0.392, -0.319, -0.247, -0.305, -0.167, -0.035, 0.052, 0.076});
    cl_table.push_back({.000, 1.128, 2.275, 3.547, 4.628, 5.753, 6.915, 8.201});
    cd_table.push_back({.271, .330, .510, .894, 1.455, 2.255, 3.196, 4.359});

    // mach 0.93
    cn_table.push_back({0.000, 1.254, 2.540, 4.024, 5.393, 6.903, 8.553, 10.586});
    cm_table.push_back({0.000, -0.458, -0.795, -1.620,-1.509, -0.989, -0.510, -0.389});
    ca_table.push_back({0.458, 0.455, 0.440, 0.421, 0.399, 0.370, 0.294, 0.147});
    cp_table.push_back({-0.418, -0.365, -0.313, -0.403, -0.280, -0.143, -0.060, -0.037});
    cl_table.push_back({.000, 1.219, 2.454, 3.848, 5.074, 6.360, 7.694, 9.278});
    cd_table.push_back({.458, .541, .790, 1.248, 1.870, 2.709, 3.748, 5.100});
}


/////////////////////////////////////////////////
void Aerodynamic::OnUpdate()
{
    GZ_ASSERT(this->link, "Link was NULL");
  
    // get linear velocity at cp in inertial frame
    // ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp) - wind_vel_;
    ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp);
    const common::Time current_time = this->world->SimTime();
    
    ROS_WARN_STREAM(" velocity: " << vel);

    // get the direction of the linear velocity in inertia frame
    ignition::math::Vector3d velI = vel;
    velI.Normalize();

    // get update time different
    const double dt = (current_time - this->last_pub_time).Double();

    // do nothing if the object almost still
    if (vel.Length() <= 0.01)
        return;

    // get actual pose of body relative to the world link
    ignition::math::Pose3d pose = this->link->WorldPose();
    ROS_WARN_STREAM(" body bose: " << pose);    

    // rotate forward vectors into inertial frame
    ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);
    ROS_WARN_STREAM(" forward in inertia: " << forwardI);

    if (forwardI.Dot(vel) <= 0.0){
        // Only calculate lift or drag if the body relative velocity is in the same direction
        return;
    }

    // calculate upward direction
    ignition::math::Vector3d upwardI;
    // if the object is radial symetrz
    if (this->radialSymmetry)
    {
        // use inflow velocity to determine upward direction
        // which is the component of inflow perpendicular to forward direction.
        ignition::math::Vector3d tmp = forwardI.Cross(velI);
        upwardI = forwardI.Cross(tmp).Normalize();
    }
    else
    {
        // else rotate upward vectors into inertial frame
        upwardI = pose.Rot().RotateVector(this->upward);
    }
    ROS_WARN_STREAM(" upward in inertia: " << upwardI);

    // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
    ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();
  ROS_WARN_STREAM(" spanwise in inertia: " << spanwiseI);

    // TODO: What is sweep angle defined???? is it necessary
    const double minRatio = -1.0;
    const double maxRatio = 1.0;
    // check sweep (angle between velI and lift-drag-plane)
    double sinSweepAngle = ignition::math::clamp(
        spanwiseI.Dot(velI), minRatio, maxRatio);

    this->sweep = asin(sinSweepAngle);

    // truncate sweep to within +/-90 deg
    while (fabs(this->sweep) > 0.5 * M_PI) 
        this->sweep = this->sweep > 0 ? this->sweep - M_PI : this->sweep + M_PI;
    
    // get cos from trig identity
    double cosSweepAngle = sqrt(1.0 - sin(this->sweep) * sin(this->sweep));

    // angle of attack is the angle between
    // velI projected into lift-drag plane
    //  and
    // forward vector
    //
    // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
    //
    // so,
    // removing spanwise velocity from vel
    ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

    // get direction of drag
    ignition::math::Vector3d dragDirection = -velInLDPlane;
    dragDirection.Normalize();

    // get direction of lift
    ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
    liftI.Normalize();

    // get direction of moment
    ignition::math::Vector3d momentDirection = spanwiseI;

    // compute angle between upwardI and liftI
    // in general, given vectors a and b:
    //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
    // given upwardI and liftI are both unit vectors, we can drop the denominator
    //   cos(theta) = a.Dot(b)
    double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);
    double mycosAlpha = ignition::math::clamp(velInLDPlane.Dot(forwardI), minRatio, maxRatio);

    ROS_WARN_STREAM(" alpha_orig_code: " << cosAlpha);
    ROS_WARN_STREAM(" alpha_my_own: " << mycosAlpha);

    // Is alpha positive or negative? Test:
    // forwardI points toward zero alpha
    // if forwardI is in the same direction as lift, alpha is positive.
    // liftI is in the same direction as forwardI?
    if (liftI.Dot(forwardI) >= 0.0)
        this->alpha = acos(cosAlpha);
    else
        this->alpha = -1 * acos(cosAlpha);

    // normalize to within +/-90 deg
    while (fabs(this->alpha) > 0.5 * M_PI)
        this->alpha = this->alpha > 0 ? this->alpha - M_PI : this->alpha + M_PI;

    
    double speed = vel.Length();
    const double speed_of_sound = 343;
    // convert speed to mach number
    double mach_val = speed / speed_of_sound;
    std::vector<double>::iterator alpha_iter, mach_iter;
    int m_u, m_l, a_u, a_l;
    alpha_iter = std::lower_bound(alpha_table.begin(), alpha_table.end(), this->alpha);
    mach_iter = std::lower_bound(mach_table.begin(), mach_table.end(), mach_val);
    
    a_u = alpha_iter - alpha_table.begin();
    m_u = mach_iter - mach_table.begin();

    a_l = a_u - 1;
    m_l = m_u - 1;


    // x = ((x2 - x1)*(y - y1)/(y2 - y1)) + x1);
    // Get Aerodynamic Parameter from the table
    double slope_cn = (cn_table[m_u][a_u] - cn_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);
    double slope_cm = (cm_table[m_u][a_u] - cm_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);
    double slope_ca = (ca_table[m_u][a_u] - ca_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);
    double slope_cl = (cl_table[m_u][a_u] - cl_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);
    double slope_cd = (cd_table[m_u][a_u] - cd_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);
    double slope_cp = (cp_table[m_u][a_u] - cp_table[m_l][a_l])/(alpha_table[a_u] - alpha_table[a_l]);

    double cn_val = cn_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_cn);
    double cm_val = cm_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_cm);
    double ca_val = ca_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_ca);
    double cl_val = cl_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_cl);
    double cd_val = cd_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_cd);
    double cp_val = cp_table[m_l][a_l] + ((this->alpha - alpha_table[a_l])*slope_cp);  

    // compute dynamic pressure
    double speedInLDPlane = velInLDPlane.Length();
    double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

    ROS_WARN_STREAM(" cn_val: " << cn_val);
    ROS_WARN_STREAM(" cm_val: " << cm_val);
    ROS_WARN_STREAM(" ca_val: " << ca_val);
    ROS_WARN_STREAM(" cl_val: " << cl_val);
    ROS_WARN_STREAM(" cd_val: " << cd_val);
    ROS_WARN_STREAM(" cp_val: " << cp_val << " tambah: " << cp_val + this->cp);


    ignition::math::Vector3d lift = cn_val * q * this->area * liftI;
    ignition::math::Vector3d drag = ca_val * q * this->area * dragDirection;
    ignition::math::Vector3d moment = cm_val * q * this->area * momentDirection;
    ignition::math::Vector3d normalF = cm_val * q * this->area * upwardI;
    ignition::math::Vector3d axialF = cm_val * q * this->area * forwardI;

    ROS_WARN_STREAM(" normal: " << normalF);
    ROS_WARN_STREAM(" axial: " << axialF);
    ROS_WARN_STREAM(" moment: " << moment);
    ROS_WARN_STREAM(" lift: " << normalF);
    ROS_WARN_STREAM(" drag: " << axialF);


    ignition::math::Vector3d force = lift + drag;

    this->cp += cp_val*this->forward;

    force.Correct();
    this->cp.Correct();
    moment.Correct();

    // apply forces at cg (with torques for position shift)
    this->link->AddForceAtRelativePosition(force, this->cp);
    this->link->AddTorque(moment);

    auto relative_center = this->link->RelativePose().Pos() + this->cp;

    // Publish force and center of pressure for potential visual plugin.
    // - dt is used to control the rate at which the force is published
    // - it only gets published if 'topic_name' is defined in the sdf
    if (dt > 1.0 / 10 && this->sdf->HasElement("topic_name"))
    {
        msgs::Vector3d* force_center_msg = new msgs::Vector3d;
        force_center_msg->set_x(relative_center.X());
        force_center_msg->set_y(relative_center.Y());
        force_center_msg->set_z(relative_center.Z());

        msgs::Vector3d* force_vector_msg = new msgs::Vector3d;
        force_vector_msg->set_x(force.X());
        force_vector_msg->set_y(force.Y());
        force_vector_msg->set_z(force.Z());

        ssm_msgs::msgs::Force force_msg;
        force_msg.set_allocated_center(force_center_msg);
        force_msg.set_allocated_force(force_vector_msg);

        lift_force_pub_->Publish(force_msg);
        this->last_pub_time = current_time;

    }    
}