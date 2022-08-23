
#include <algorithm>
#include <string>
#include <vector>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "aerodynamic.hh"
#include "aerodynamic_private.hh"

#include <ros/ros.h>

#include "MultiplevectorVisual.pb.h"


#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(Aerodynamic);


/////////////////////////////////////////////////
Aerodynamic::Aerodynamic() : dataPtr(new AerodynamicPrivate), V(new AerodynamicVector), I(new AerodynamicVector), C(new AerodynamicCoefficients)
{
    dataPtr->rho = 1.2041;
    dataPtr->cp = ignition::math::Vector3d(0, 0, 0);
    dataPtr->forward = ignition::math::Vector3d(1, 0, 0);
    dataPtr->upward = ignition::math::Vector3d(0, 0, 1);
    dataPtr->alpha = 0.0;
    dataPtr->sweep = 0.0;
    dataPtr->radialSymmetry = false;
    dataPtr->controlAngle = 0;

    this->ucase = 0;
}

/////////////////////////////////////////////////
Aerodynamic::~Aerodynamic()
{
}

/////////////////////////////////////////////////
void Aerodynamic::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    dataPtr->model = _model;
    GZ_ASSERT(dataPtr->model, "Aerodynamic _model pointer is NULL");

    dataPtr->sdf = _sdf;
    GZ_ASSERT(dataPtr->sdf, "Aerodynamic _sdf pointer is NULL");

    dataPtr->world = dataPtr->model->GetWorld();
    GZ_ASSERT(dataPtr->world, "Aerodynamic world pointer is NULL");

    dataPtr->physics = dataPtr->world->Physics();
    GZ_ASSERT(dataPtr->physics, "Aerodynamic physics pointer is NULL");

    this->last_pub_time = dataPtr->world->SimTime();

    if (_sdf->HasElement("air_density"))
    {
        dataPtr->rho = _sdf->Get<double>("air_density");
    }

    if (_sdf->HasElement("radial_symmetry"))
    {
        dataPtr->radialSymmetry = _sdf->Get<bool>("radial_symmetry");
        ROS_WARN_STREAM("Radial Symetry: " << dataPtr->radialSymmetry);
    }

    if (_sdf->HasElement("cp"))
    {
        dataPtr->cp = _sdf->Get<ignition::math::Vector3d>("cp");
    }
    ROS_WARN_STREAM("Forces will be applied here: "<< dataPtr->cp);

    if (_sdf->HasElement("forward"))
    {
        dataPtr->forward = _sdf->Get<ignition::math::Vector3d>("forward");
    }
    dataPtr->forward.Normalize();

    if (_sdf->HasElement("upward"))
    {
        dataPtr->upward = _sdf->Get<ignition::math::Vector3d>("upward");
    }
    dataPtr->upward.Normalize();

    if (_sdf->HasElement("sref"))
    {
        dataPtr->sref = _sdf->Get<double>("sref");
    }

    if (_sdf->HasElement("lref"))
    {
        dataPtr->lref = _sdf->Get<double>("lref");
    }

    if (_sdf->HasElement("link_name"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("link_name");
        GZ_ASSERT(elem, "Element link_name doesn't exist!");

        std::string linkName = elem->Get<std::string>();
        dataPtr->link = dataPtr->model->GetLink(linkName);

        if (!dataPtr->link)
        {
            gzerr << "Link with name[" << linkName << "] not found. " << "The Aerodynamic will not generate forces\n";
        }
        else
        {
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Aerodynamic::OnUpdate, this));
        }
    }

    if (_sdf->HasElement("ref_link"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("ref_link");
        GZ_ASSERT(elem, "Element ref_link doesn't exist!");

        std::string linkName = elem->Get<std::string>();
        dataPtr->link_ref = dataPtr->model->GetLink(linkName);

        if (!dataPtr->link_ref)
        {
            gzerr << "Link with name[" << linkName << "] not found. " << "The Aerodynamic will not generate forces\n";
        }
    }

    if (_sdf->HasElement("control_joint_name"))
    {
        auto ctrJointElem = _sdf->GetElement("control_joint_name");
        std::string controlJointName;
        ctrJointElem->GetValue()->Get<std::string>(controlJointName);

        this->dataPtr->controlJoint = this->dataPtr->model->GetJoint(controlJointName);
        if (!this->dataPtr->controlJoint)
        {
            gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
        }

        if (ctrJointElem->HasAttribute("inverse")){
            auto inversVal = ctrJointElem->GetAttribute("inverse");
            bool val;
            inversVal->Get<bool>(val);
            this->dataPtr->inverse_joint_value = val;
        } 
        else
        {
            this->dataPtr->inverse_joint_value = 0;
        }

        ROS_WARN_STREAM("Joint: " << controlJointName << " inverse " << dataPtr->inverse_joint_value);

    }

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        namespace_ = "";
        // gzerr << "[gazebo_liftdrag_plugin] Please specify a robotNamespace.\n";
    }

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("topic_name"))
    {
        const auto vel_vis_topic = dataPtr->sdf->Get<std::string>("topic_name");
        vectors_pub_ = node_handle_->Advertise<ssm_msgs::msgs::MultiplevectorVisual>("~/" + vel_vis_topic);
        ROS_WARN_STREAM("Publishing to ~/" << vel_vis_topic << std::endl);
    }

    this->ParseCoefficientTable();

    for (size_t i = 0; i < cTable.size(); i++)
    {
        gzdbg << "Case: " << cTable[i].a_case << std::endl;
        gzdbg << "Type: " << cTable[i].a_type << std::endl;
        std::string mach_table = "mach_table\t: ";
        for (size_t j = 0; j < 8; j++)
        {
            
            mach_table += std::to_string(cTable[i].mach_table[j]) + " ";
        }
        gzdbg << mach_table << std::endl;

        std::string alpha_table = "alpha_table\t: ";
        for (size_t j = 0; j < 8; j++)
        {
            alpha_table += std::to_string(cTable[i].alpha_table[j]) + " ";
        }

        gzdbg << alpha_table << std::endl;
        
        for (size_t k = 0; k < 8; k++)
        {
            gzdbg << " MACH: " << cTable[i].mach_table[k] << std::endl;
            std::string cm_table = "cm_table\t: ";
            for (size_t j = 0; j < 8; j++)
            {
                cm_table += std::to_string(cTable[i].cm_table[k][j]) + " ";
            }
            gzdbg << cm_table << std::endl;

            if (cTable[i].a_type == Wind)
            {
                std::string cl_table = "cl_table\t: ";
                for (size_t j = 0; j < 8; j++)
                {
                    cl_table += std::to_string(cTable[i].cl_table[k][j]) + " ";
                }
                gzdbg << cl_table << std::endl;

                std::string cd_table = "cd_table\t: ";
                for (size_t j = 0; j < 8; j++)
                {
                    cd_table += std::to_string(cTable[i].cd_table[k][j]) + " ";
                }
                gzdbg << cd_table << std::endl;
            }
            if (cTable[i].a_type == Body)
            {
                std::string cn_table = "cn_table\t: ";
                for (size_t j = 0; j < 8; j++)
                {
                    cn_table += std::to_string(cTable[i].cn_table[k][j]) + " ";
                }
                gzdbg << cn_table << std::endl;
                
                std::string ca_table = "ca_table\t: ";
                for (size_t j = 0; j < 8; j++)
                {
                    ca_table += std::to_string(cTable[i].ca_table[k][j]) + " ";
                }
                gzdbg << ca_table << std::endl;
            }
        }
    }
    ROS_INFO_STREAM("Loading Aero plugin for link " << this->dataPtr->link->GetName());
    
    gzlog << "Aerodynamic plugin loaded for link " << this->dataPtr->link->GetName() << std::endl;
}

/////////////////////////////////////////////////
static void store_table(const sdf::ElementPtr& elem, const std::string& elem_name, std::vector<double>& v)
{
    std::string str = elem->Get<std::string>(elem_name);
    // gzdbg << "elem: " << elem_name << " : " <<str << "\n";
    // split str to string of vector
    std::vector<std::string> stringVector;
    boost::split(stringVector, str, boost::is_any_of(", "), boost::token_compress_on);

    // convert vector of string to vector of double
    std::vector<double> doubleVector(stringVector.size());
    std::transform(stringVector.begin(), stringVector.end(), doubleVector.begin(), [](const std::string& val)
    {
        return stod(val);
    });

    // assign to the vector
    for (std::size_t i = 0; i < doubleVector.size(); i++){
        v.push_back(doubleVector[i]);
        // gzdbg << doubleVector[i];
    }
}

/////////////////////////////////////////////////
void Aerodynamic::ParseCoefficientTable()
{
    ROS_WARN_STREAM(" Parsing Coefficient Table");

    if(dataPtr->sdf->HasElement("aerodynamics"))
    {
        dataPtr->number_of_cases = 0;
        // store data foreach aerodynamics element
        sdf::ElementPtr aeroElemPtr  = dataPtr->sdf->GetElement("aerodynamics");
        do
        {
            AerodynamicLUT coef;
            // count the aerodynamic use case each aerodynamics element
            coef.a_case = dataPtr->number_of_cases++;
            // get aerodynamic axis to use for calculation, use wind axis by default
            if (aeroElemPtr->HasAttribute("axis_type"))
            {
                auto aero_axis_type = aeroElemPtr->GetAttribute("axis_type");
                std::string txt;
                aero_axis_type->Get<std::string>(txt);
                if(txt.compare("body")==0)
                    coef.a_type = Body;
                else if(txt.compare("wind")==0)
                {
                    coef.a_type = Wind;
                } 
                else
                {
                    coef.a_type = Both;
                }
            }
            else
            {
                coef.a_type = Wind;
            }

            // store mach_table
            if (aeroElemPtr->HasElement("mach_table"))
            {
                store_table(aeroElemPtr, "mach_table", coef.mach_table);
            }
            else
            {
                gzerr << "Please specified list of mach number under tag <mach_table> </mach_table>!";
            }

            // store alpha_table
            if (aeroElemPtr->HasElement("alpha_table"))
            {
                store_table(aeroElemPtr, "alpha_table", coef.alpha_table);
            }
            else
            {
                gzerr << "Please specified list of alpha number under tag <alpha_table> </alpha_table>!";
            }

            if (aeroElemPtr->HasElement("mach"))
            {
                sdf::ElementPtr machElemPtr  = aeroElemPtr->GetElement("mach");
                do
                {
                    // store cm_table
                    std::vector<double> cur_cm;
                    if (machElemPtr->HasElement("cm_table"))
                    {
                        store_table(machElemPtr, "cm_table", cur_cm);
                    }
                    else
                    {
                        gzerr << "Please specified list of mach number under tag <cm_table> </cm_table>!";
                    }
                    coef.cm_table.push_back(cur_cm);

                    if (coef.a_type == Body)
                    {
                        // store cn_table
                        std::vector<double> cur_cn;
                        if (machElemPtr->HasElement("cn_table"))
                        {
                            store_table(machElemPtr, "cn_table", cur_cn);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cn_table> </cn_table>!";
                        }
                        coef.cn_table.push_back(cur_cn);

                        // store ca_table
                        std::vector<double> cur_ca;
                        if (machElemPtr->HasElement("ca_table"))
                        {
                            store_table(machElemPtr, "ca_table", cur_ca);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <ca_table> </ca_table>!";
                        }
                        coef.ca_table.push_back(cur_ca);
                    }

                    if (coef.a_type == Wind)
                    {
                        // store cl_table
                        std::vector<double> cur_cl;
                        if (machElemPtr->HasElement("cl_table"))
                        {
                            store_table(machElemPtr, "cl_table", cur_cl);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cl_table> </cl_table>!";
                        }
                        coef.cl_table.push_back(cur_cl);

                        // store cd_table
                        std::vector<double> cur_cd;
                        if (machElemPtr->HasElement("cd_table"))
                        {
                            store_table(machElemPtr, "cd_table", cur_cd);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cd_table> </cd_table>!";
                        }
                        coef.cd_table.push_back(cur_cd);
                    }

                    if (coef.a_type == Both)
                    {
                        // store cl_table
                        std::vector<double> cur_cl;
                        if (machElemPtr->HasElement("cl_table"))
                        {
                            store_table(machElemPtr, "cl_table", cur_cl);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cl_table> </cl_table>!";
                        }
                        coef.cl_table.push_back(cur_cl);

                        // store cd_table
                        std::vector<double> cur_cd;
                        if (machElemPtr->HasElement("cd_table"))
                        {
                            store_table(machElemPtr, "cd_table", cur_cd);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cd_table> </cd_table>!";
                        }
                        coef.cd_table.push_back(cur_cd);

                        // store cn_table
                        std::vector<double> cur_cn;
                        if (machElemPtr->HasElement("cn_table"))
                        {
                            store_table(machElemPtr, "cn_table", cur_cn);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <cn_table> </cn_table>!";
                        }
                        coef.cn_table.push_back(cur_cn);

                        // store ca_table
                        std::vector<double> cur_ca;
                        if (machElemPtr->HasElement("ca_table"))
                        {
                            store_table(machElemPtr, "ca_table", cur_ca);
                        }
                        else
                        {
                            gzerr << "Please specified list of mach number under tag <ca_table> </ca_table>!";
                        }
                        coef.ca_table.push_back(cur_ca);                        
                    }

                    machElemPtr = machElemPtr->GetNextElement("mach");

                } while (machElemPtr);

            }
            else
            {
                gzerr << "Please specified aerodynamic coefficient under tag <mach> </mach>!";
            }

            // store control_joint_rate if control joint is used
            if (dataPtr->sdf->HasElement("control_joint_name"))
            {
                if (aeroElemPtr->HasElement("control_joint_rate"))
                {
                    store_table(aeroElemPtr, "control_joint_rate", coef.control_joint_rate);
                }
                else
                {
                    gzerr << "Please specified list of mach number under tag <mach_table> </mach_table>!";
                }
            }

            cTable.push_back(coef);
            aeroElemPtr = aeroElemPtr->GetNextElement("aerodynamics");

        } while (aeroElemPtr);

        if (dataPtr->number_of_cases > 1)
        {
            const auto case_change_topic = this->dataPtr->model->GetName()  + "/" + this->dataPtr->link->GetName() + "/AerodynamicCaseChange";
            case_change_sub_ = this->node_handle_->Subscribe("~/" + case_change_topic, &Aerodynamic::OnCaseChange, this);
            gzdbg << "Subscribing to ~/" << case_change_topic << std::endl;
        }
        
        // TODO: Check order, Check vector dimension
        // TODO: Check if wrong table spesified, Check vector dimension

    }
}


/////////////////////////////////////////////////
void Aerodynamic::OnUpdate()
{
    GZ_ASSERT(dataPtr->link, "Link was NULL");

    // get update time different
    const common::Time current_time = dataPtr->world->SimTime();
    const double dt = (current_time - this->last_pub_time).Double();

    // if Aerodynamic calculation gives false, aerodynamic will not be calculated
    if(!this->CalculateAerodynamicVectors())
    {
        return;
    }

    this->CalculateAerodynamicAngles();

    this->CalculateAerodynamicCoefficients();

    this->CalculateAerodynamicForces();

    std::string lk;
    ignition::math::Vector3d offset;
    if (dataPtr->link_ref)
    {
        auto rel_pos = dataPtr->link_ref->RelativePose();
        offset = dataPtr->cp + rel_pos.Pos();
        lk = dataPtr->link_ref->GetName();
    }
    else
    {
        lk = dataPtr->link->GetName();
        offset = dataPtr->cp;
    }

    // Apply force
    if (cTable[ucase].a_type == Body)
    {
        dataPtr->link->AddForceAtRelativePosition(V->normal, offset);
        dataPtr->link->AddForceAtRelativePosition(V->axial, offset);
    }
    else
    {
        dataPtr->link->AddForceAtRelativePosition(V->lift, offset);
        dataPtr->link->AddForceAtRelativePosition(V->drag, offset);
    }

    dataPtr->link->AddTorque(V->moment);


    // // Publish force and center of pressure for potential visual plugin.
    // // - dt is used to control the rate at which the force is published
    // // - it only gets published if 'topic_name' is defined in the sdf
    if (dt > 1.0 / 10)
    {
        this->last_pub_time = current_time;
        this->PublishAeroForces();

        ROS_WARN_STREAM(" " << lk << "     sweep: " << dataPtr->sweep << "(Rad) -- " << dataPtr->sweep * 180/M_PI << "(Deg)");
        ROS_WARN_STREAM(" " << lk << "   ctrlAng: " << dataPtr->controlAngle*M_PI/180 << "(Rad) -- " << dataPtr->controlAngle << "(Deg)");
        ROS_WARN_STREAM(" " << lk << " ClDuectrl: " << C->rate_cl);
        ROS_WARN_STREAM(" " << lk << "  velocity: " << V->velocity.Length());
        ROS_WARN_STREAM(" " << lk << "     alpha: " << dataPtr->alpha*M_PI/180 << "(Rad) -- " << dataPtr->alpha << "(Deg)");
        ROS_WARN_STREAM(" " << lk << "      mach: " << dataPtr->mach);
        // ROS_WARN_STREAM(" " << lk << " dynamic_pressure\t" << dataPtr->q);
        // ROS_WARN_STREAM(" " << lk << " cn_val\t: " << C->cn);
        // ROS_WARN_STREAM(" " << lk << " ca_val\t: " << C->ca);
        // ROS_WARN_STREAM(" " << lk << " cl_val\t: " << C->cl);
        // ROS_WARN_STREAM(" " << lk << " cd_val\t: " << C->cd);
        // ROS_WARN_STREAM(" " << lk << " cm_val\t: " << C->cm);
        ROS_WARN_STREAM(" " << lk << "     lift: " << V->lift.Length() << " <" << V->lift << ">");
        ROS_WARN_STREAM(" " << lk << "     drag: " << V->drag.Length() << " <" << V->drag << ">");
        ROS_WARN_STREAM(" " << lk << "   normal: " << V->normal.Length() << " <" << V->normal << ">");
        ROS_WARN_STREAM(" " << lk << "    axial: " << V->axial.Length() << " <" << V->axial << ">");
        ROS_WARN_STREAM(" " << lk << "   moment: " << V->moment.Length() << " <" << V->moment << ">");
        // ROS_WARN_STREAM(" " << lk << " body bose: " << dataPtr->pose);
        // ROS_WARN_STREAM(" " << lk << "    lift: " << I->lift);
        // ROS_WARN_STREAM(" " << lk << "    drag: " << I->drag);
        // ROS_WARN_STREAM(" " << lk << "  normal: " << I->normal);
        // ROS_WARN_STREAM(" " << lk << "   axial: " << I->axial);
        ROS_WARN_STREAM(" " << lk << "  momentDir: " << I->moment);
        ROS_WARN("\n");      
    }
}


/////////////////////////////////////////////////
bool Aerodynamic::CalculateAerodynamicVectors()
{
    if (dataPtr->link_ref)
    {
        dataPtr->pose = dataPtr->link_ref->WorldPose();
        V->velocity = dataPtr->link_ref->WorldLinearVel(dataPtr->cp);
    }
    else 
    {
        // get actual pose of body relative to the world link
        dataPtr->pose = dataPtr->link->WorldPose();        
        // get linear velocity at cp in inertial frame
        V->velocity = dataPtr->link->WorldLinearVel(dataPtr->cp);
    }
    

    // get the direction of the linear velocity in inertia frame
    I->velocity = V->velocity;
    I->velocity.Normalize();

    // do nothing if the object almost still
    if (V->velocity.Length() < 1)
        return false;

    // rotate forward vectors into inertial frame
    I->axial = dataPtr->pose.Rot().RotateVector(dataPtr->forward);

    // Only calculate aerodynamic, if the body relative velocity is in the same direction
    if (I->axial.Dot(V->velocity) <= 0.0)
        return false;

    // calculate upward direction
    // if the object is radial symetry
    if (dataPtr->radialSymmetry)
    {
        // use inflow velocity to determine upward direction
        // which is the component of inflow perpendicular to forward direction.
        ignition::math::Vector3d tmp = I->axial.Cross(I->velocity);
        I->normal = I->axial.Cross(tmp).Normalize();
        // ROS_WARN("RADIAL C");
    }
    else
    {
        // else rotate upward vectors given in sdf into inertial frame
        I->normal = dataPtr->pose.Rot().RotateVector(dataPtr->upward);
    }

    // get direction of moment
    // I->moment: a vector normal to lift-drag-plane described in inertial frame
    I->moment = I->axial.Cross(I->normal).Normalize();

    // Get the velocity projected into lift-drag plane by removing spanwise velocity from V->velocity
    V->vel_in_lift_drag_plane = V->velocity - V->velocity.Dot(I->moment)*I->moment;

    // get direction of drag
    I->drag = -V->vel_in_lift_drag_plane;
    I->drag.Normalize();

    // get direction of lift
    I->lift = I->moment.Cross(V->vel_in_lift_drag_plane);
    I->lift.Normalize();

    // angle of attack is the angle between body relative velocity projected in lift-drag plane and I->axial
    I->vel_in_lift_drag_plane = V->vel_in_lift_drag_plane;
    I->vel_in_lift_drag_plane.Normalize();

    I->axial = -1 * I->axial;

    return true;
}

/////////////////////////////////////////////////
void Aerodynamic::CalculateAerodynamicAngles()
{
    const double minRatio = -1.0;
    const double maxRatio = 1.0;

    // check sweep (angle between I->velocity and lift-drag-plane)
    double sinSweepAngle = ignition::math::clamp(I->moment.Dot(I->velocity), minRatio, maxRatio);
    dataPtr->sweep = asin(sinSweepAngle);

    // truncate sweep to within +/-90 deg
    while (fabs(dataPtr->sweep) > 0.5 * M_PI)
        dataPtr->sweep = dataPtr->sweep > 0 ? dataPtr->sweep - M_PI : dataPtr->sweep + M_PI;


    // compute angle of attack
    double cosAlpha = ignition::math::clamp(I->lift.Dot(I->normal), minRatio, maxRatio);
    // double cosAlpha = ignition::math::clamp(I->vel_in_lift_drag_plane.Dot(I->axial), minRatio, maxRatio);

    dataPtr->alpha = acos(cosAlpha);

    // normalize to within +/-90 deg
    while (fabs(dataPtr->alpha) > 0.5 * M_PI)
        dataPtr->alpha = dataPtr->alpha > 0 ? dataPtr->alpha - M_PI : dataPtr->alpha + M_PI;
    
    // alpha table for look up using degree
    dataPtr->alpha *= (180 / M_PI);

}

/////////////////////////////////////////////////
static double linear_function(double x, double x1, double x2, double y1, double y2)
{
    // (y - y1)/(y2 - y1)) = (x - x1)/(x2 - x1);
    // (y - y1)) = (x - x1)*(y2 - y1)*/(x2 - x1);
    // m = (y2 - y1)*/(x2 - x1)
    // y = (x - x1)*m + y1

    double m = (y2 - y1)/(x2 - x1);
    return (x - x1) * m + y1;
}

/////////////////////////////////////////////////
void Aerodynamic::CalculateAerodynamicCoefficients()
{
    double speed = V->velocity.Length();
    // const double speed_of_sound = 343;

    // convert speed to mach number
    // dataPtr->mach = speed / speed_of_sound;
    dataPtr->mach = speed * 0.002915;

    // ROS_WARN_STREAM("SPEED: " << speed);
    // ROS_WARN_STREAM("MACH: " << dataPtr->mach);
    // ROS_WARN_STREAM("ALPHA: " << dataPtr->alpha);
    // TODO: what happened if the alpha or mach value greater than the table? do asertion
    // GZ_ASSERT(dataPtr->mach <= cTable[ucase].mach_table[cTable[ucase].mach_table.size() -1], "Speed is greater than the allowed working speed");
    GZ_ASSERT(dataPtr->alpha >= cTable[ucase].alpha_table[0],"AoA is lower than working area in alpha table");
    // GZ_ASSERT(dataPtr->alpha <= cTable[ucase].alpha_table[cTable[ucase].alpha_table.size() - 1],"AoA is greater than working area in alpha table");

    std::vector<double>::iterator alpha_iter, mach_iter;
    int m_upper, m_lower, a_upper, a_lower;

    alpha_iter = std::upper_bound(cTable[ucase].alpha_table.begin(), cTable[ucase].alpha_table.end(), dataPtr->alpha);
    mach_iter = std::upper_bound(cTable[ucase].mach_table.begin(), cTable[ucase].mach_table.end(), dataPtr->mach);

    a_upper = alpha_iter - cTable[ucase].alpha_table.begin();
    m_upper = mach_iter - cTable[ucase].mach_table.begin();

    a_lower = a_upper - 1;
    // for cases that mach is lower than 0.2
    m_lower = m_upper == 0 ? 0.0 : m_upper - 1;
    m_upper = m_upper == cTable[ucase].mach_table.size() ? m_lower : m_upper;
    a_upper = a_upper == cTable[ucase].alpha_table.size() ? a_lower : a_upper;

    double a1 = cTable[ucase].alpha_table[a_lower];
    double a2 = cTable[ucase].alpha_table[a_upper];
    double m1 = cTable[ucase].mach_table[m_lower];
    double m2 = cTable[ucase].mach_table[m_upper];

    AerodynamicCoefficients lower;
    AerodynamicCoefficients upper;

    // Interpolate the aerodynamic coefficient, first in regard angle of attack then in regard mach number

    lower.cm = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cm_table[m_lower][a_lower], cTable[ucase].cm_table[m_lower][a_upper]);
    upper.cm = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cm_table[m_upper][a_lower], cTable[ucase].cm_table[m_upper][a_upper]);
    C->cm = (lower.cm == upper.cm) ? lower.cm : linear_function(dataPtr->mach, m1, m2, lower.cm, upper.cm);

    if (cTable[ucase].a_type == Body)
    {
        lower.cn = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cn_table[m_lower][a_lower], cTable[ucase].cn_table[m_lower][a_upper]);
        lower.ca = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].ca_table[m_lower][a_lower], cTable[ucase].ca_table[m_lower][a_upper]);
        upper.cn = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cn_table[m_upper][a_lower], cTable[ucase].cn_table[m_upper][a_upper]);
        upper.ca = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].ca_table[m_upper][a_lower], cTable[ucase].ca_table[m_upper][a_upper]);

        C->cn = (lower.cn == upper.cn) ? lower.cn : linear_function(dataPtr->mach, m1, m2, lower.cn, upper.cn);
        C->ca = (lower.ca == upper.ca) ? lower.ca : linear_function(dataPtr->mach, m1, m2, lower.ca, upper.ca);
    }
    else if (cTable[ucase].a_type == Wind)
    {
        lower.cl = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cl_table[m_lower][a_lower], cTable[ucase].cl_table[m_lower][a_upper]);
        lower.cd = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cd_table[m_lower][a_lower], cTable[ucase].cd_table[m_lower][a_upper]);
        upper.cl = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cl_table[m_upper][a_lower], cTable[ucase].cl_table[m_upper][a_upper]);
        upper.cd = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cd_table[m_upper][a_lower], cTable[ucase].cd_table[m_upper][a_upper]);
        C->cl = (lower.cl == upper.cl) ? lower.cl : linear_function(dataPtr->mach, m1, m2, lower.cl, upper.cl);
        C->cd = (lower.cd == upper.cd) ? lower.cd : linear_function(dataPtr->mach, m1, m2, lower.cd, upper.cd);
    }
    else 
    {
        lower.cn = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cn_table[m_lower][a_lower], cTable[ucase].cn_table[m_lower][a_upper]);
        lower.ca = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].ca_table[m_lower][a_lower], cTable[ucase].ca_table[m_lower][a_upper]);
        upper.cn = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cn_table[m_upper][a_lower], cTable[ucase].cn_table[m_upper][a_upper]);
        upper.ca = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].ca_table[m_upper][a_lower], cTable[ucase].ca_table[m_upper][a_upper]);

        C->cn = (lower.cn == upper.cn) ? lower.cn : linear_function(dataPtr->mach, m1, m2, lower.cn, upper.cn);
        C->ca = (lower.ca == upper.ca) ? lower.ca : linear_function(dataPtr->mach, m1, m2, lower.ca, upper.ca);

        lower.cl = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cl_table[m_lower][a_lower], cTable[ucase].cl_table[m_lower][a_upper]);
        lower.cd = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cd_table[m_lower][a_lower], cTable[ucase].cd_table[m_lower][a_upper]);
        upper.cl = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cl_table[m_upper][a_lower], cTable[ucase].cl_table[m_upper][a_upper]);
        upper.cd = linear_function(dataPtr->alpha, a1, a2, cTable[ucase].cd_table[m_upper][a_lower], cTable[ucase].cd_table[m_upper][a_upper]);
        C->cl = (lower.cl == upper.cl) ? lower.cl : linear_function(dataPtr->mach, m1, m2, lower.cl, upper.cl);
        C->cd = (lower.cd == upper.cd) ? lower.cd : linear_function(dataPtr->mach, m1, m2, lower.cd, upper.cd);        
    }

    // Compensate with the sweep angle if there are any
    double cossweep = sqrt(1.0 - sin(dataPtr->sweep) * sin(dataPtr->sweep));
    C->cm *= cossweep;
    C->cn *= cossweep;
    C->ca *= cossweep;
    C->cl *= cossweep;
    C->cd *= cossweep;
    // dataPtr->sweep *= (180 / M_PI);

    if (this->dataPtr->controlJoint)
    {
        this->dataPtr->controlAngle = this->dataPtr->controlJoint->Position(0) * 180 / M_PI;

        int rate_size = cTable[ucase].control_joint_rate.size();

        C->rate_cl = (m_upper >= rate_size) ? cTable[ucase].control_joint_rate[rate_size - 1] 
            : linear_function(dataPtr->mach, m1, m2, cTable[ucase].control_joint_rate[m_lower], cTable[ucase].control_joint_rate[m_upper]);

        // inverse the direction if inverse flag is true
        this->dataPtr->controlAngle = this->dataPtr->inverse_joint_value ? -1 * this->dataPtr->controlAngle : this->dataPtr->controlAngle;

        C->cl += this->dataPtr->controlAngle * C->rate_cl;
        C->cm += this->dataPtr->controlAngle * C->rate_cl;
    }
    
}

/////////////////////////////////////////////////
void Aerodynamic::CalculateAerodynamicForces()
{
    // compute dynamic pressure
    double speedInLDPlane = V->vel_in_lift_drag_plane.Length();
    dataPtr->q = 0.5 * dataPtr->rho * speedInLDPlane * speedInLDPlane;

    // compute aerodynamic forces and moment
    if (cTable[ucase].a_type == Body)
    {
        V->normal = I->normal * (C->cn * dataPtr->q * dataPtr->sref);
        V->axial = I->axial * (C->ca * dataPtr->q * dataPtr->sref);
        V->normal.Correct();
        V->axial.Correct();
    }
    else if (cTable[ucase].a_type == Wind)
    {
        V->lift = I->lift * (C->cl * dataPtr->q * dataPtr->sref);
        V->drag = I->drag * (C->cd * dataPtr->q * dataPtr->sref);
        V->lift.Correct();
        V->drag.Correct();
    }
    else
    {
        V->normal = I->normal * (C->cn * dataPtr->q * dataPtr->sref);
        V->axial = I->axial * (C->ca * dataPtr->q * dataPtr->sref);
        V->normal.Correct();
        V->axial.Correct();

        V->lift = I->lift * (C->cl * dataPtr->q * dataPtr->sref);
        V->drag = I->drag * (C->cd * dataPtr->q * dataPtr->sref);
        V->lift.Correct();
        V->drag.Correct();        
    }
    V->moment = I->moment * (C->cm * dataPtr->q * dataPtr->sref * dataPtr->lref);
    V->moment.Correct();
}

/////////////////////////////////////////////////
void Aerodynamic::PublishAeroForces()
{
    if (dataPtr->sdf->HasElement("topic_name"))
    {
        ignition::math::Vector3d offset;
        
        if (dataPtr->link_ref)
        {
            auto rel_pos = dataPtr->link_ref->RelativePose();
            offset = dataPtr->cp + rel_pos.Pos();
        }
        else
        {
            offset = dataPtr->cp;
        }        
        ignition::math::Vector3d rel_center = dataPtr->link->RelativePose().Pos() + offset;
        rel_center.Correct();

        ignition::math::Vector3d vel_vis = dataPtr->pose.Rot().RotateVectorReverse(I->vel_in_lift_drag_plane);
        // vel_vis.Normalize();
        vel_vis = 1000 * vel_vis;
        msgs::Vector3d* vel_vector_msg = new msgs::Vector3d;
        msgs::Set(vel_vector_msg, vel_vis);

        ignition::math::Vector3d lift_vis = dataPtr->pose.Rot().RotateVectorReverse(I->lift);
        // lift_vis.Normalize();
        lift_vis = 1000 * lift_vis;
        msgs::Vector3d* lift_vector_msg = new msgs::Vector3d;
        msgs::Set(lift_vector_msg, lift_vis);

        ignition::math::Vector3d drag_vis = dataPtr->pose.Rot().RotateVectorReverse(I->drag);
        // drag_vis.Normalize();
        drag_vis = 1000 * drag_vis;
        msgs::Vector3d* drag_vector_msg = new msgs::Vector3d;
        msgs::Set(drag_vector_msg, drag_vis);

        ignition::math::Vector3d normal_vis = dataPtr->pose.Rot().RotateVectorReverse(I->normal);
        // normal_vis.Normalize();
        normal_vis = 1000 * normal_vis;
        msgs::Vector3d* normal_vector_msg = new msgs::Vector3d;
        msgs::Set(normal_vector_msg, normal_vis);

        ignition::math::Vector3d axial_vis = dataPtr->pose.Rot().RotateVectorReverse(I->axial);
        // axial_vis.Normalize();
        axial_vis = 1000 * axial_vis;
        msgs::Vector3d* axial_vector_msg = new msgs::Vector3d;
        msgs::Set(axial_vector_msg, axial_vis);

        msgs::Vector3d* relative_center_msg1 = new msgs::Vector3d;
        msgs::Set(relative_center_msg1, rel_center);
        
        msgs::Vector3d* relative_center_msg2 = new msgs::Vector3d;
        msgs::Set(relative_center_msg2, rel_center);
        
        msgs::Vector3d* relative_center_msg3 = new msgs::Vector3d;
        msgs::Set(relative_center_msg3, rel_center);
        
        msgs::Vector3d* relative_center_msg4 = new msgs::Vector3d;
        msgs::Set(relative_center_msg4, rel_center);
        
        msgs::Vector3d* relative_center_msg5 = new msgs::Vector3d;
        msgs::Set(relative_center_msg5, rel_center);

        ssm_msgs::msgs::VectorVisual vel_msg;
        vel_msg.set_allocated_center(relative_center_msg1);
        vel_msg.set_allocated_vector(vel_vector_msg);

        ssm_msgs::msgs::VectorVisual lift_msg;
        lift_msg.set_allocated_center(relative_center_msg2);
        lift_msg.set_allocated_vector(lift_vector_msg);

        ssm_msgs::msgs::VectorVisual drag_msg;
        drag_msg.set_allocated_center(relative_center_msg3);
        drag_msg.set_allocated_vector(drag_vector_msg);

        ssm_msgs::msgs::VectorVisual normal_msg;
        normal_msg.set_allocated_center(relative_center_msg4);
        normal_msg.set_allocated_vector(normal_vector_msg);

        ssm_msgs::msgs::VectorVisual axial_msg;
        axial_msg.set_allocated_center(relative_center_msg5);
        axial_msg.set_allocated_vector(axial_vector_msg);

        ssm_msgs::msgs::MultiplevectorVisual vectors_to_visualize;
        vectors_to_visualize.add_vectors()->CopyFrom(vel_msg);
        vectors_to_visualize.add_vectors()->CopyFrom(lift_msg);
        vectors_to_visualize.add_vectors()->CopyFrom(drag_msg);
        vectors_to_visualize.add_vectors()->CopyFrom(normal_msg);
        vectors_to_visualize.add_vectors()->CopyFrom(axial_msg);

        vectors_pub_->Publish(vectors_to_visualize);
    }
}

/////////////////////////////////////////////////
void Aerodynamic::OnCaseChange(const boost::shared_ptr<const gazebo::msgs::Int> &msg)
{
    int data = msg->data();
    if (data >= this->dataPtr->number_of_cases || data < 0)
    {
        gzwarn << "There is no case with number " << data << "No case changes!\nTry again with cases in range between 0 and " << (this->dataPtr->number_of_cases - 1) << std::endl;
    }
    else
    {
        this->ucase = data;
        // ROS_WARN_STREAM("Use " << this->ucase);
    }
    
}