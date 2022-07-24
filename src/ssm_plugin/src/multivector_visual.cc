#include "multivectors_visual.hh"
#include <ros/ros.h>

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(MultivectorsVisual);

MultivectorsVisual::MultivectorsVisual() {}

MultivectorsVisual::~MultivectorsVisual() {}

void MultivectorsVisual::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_visual, "MultivectorsVisual _visual pointer is NULL");
    GZ_ASSERT(_sdf, "MultivectorsVisual _sdf pointer is NULL");

    this->visual = _visual;
    this->sdf = _sdf;

    const auto typeParam = this->sdf->GetAttribute("name");
    std::string name;
    typeParam->Get<std::string>(name);
    gzdbg << "Loading MultivectorsVisual plugin for " << name << std::endl;

    this->node.reset(new gazebo::transport::Node());
    this->node->Init();
    this->subs.reset();
    
    int vector_size = 1;

    if (this->sdf->HasElement("topic_name")) {
        
        sdf::ElementPtr multivectors = this->sdf->GetElement("topic_name");
        std::string multivectors_topic;
        multivectors->GetValue()->Get<std::string>(multivectors_topic);
        // multivectors->Get<std::string>(multivectors_topic);
        

        this->subs = this->node->Subscribe("~/" + multivectors_topic, &MultivectorsVisual::OnUpdate, this);
        gzdbg << "Subscribing on ~/" << multivectors_topic << std::endl;

        const auto multivectors_attr = multivectors->GetAttribute("vector_size");
        
        multivectors_attr->Get<int>(vector_size);
    }
    gzdbg << "Will visualize " << vector_size << " vectors." << std::endl;
    std::vector<std::string> material_list = {"Gazebo/Red", "Gazebo/Green", "Gazebo/Blue", "Gazebo/Orange", "Gazebo/Purple"};

    for (int i = 0; i < vector_size; i++)
    {
        auto forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
        rendering::DynamicLinesPtr vector_ptr(forceVector);
        
        this->vectors.push_back(vector_ptr);

        // this->forceVector.reset(forceVector);
        this->vectors[i]->setVisibilityFlags(GZ_VISIBILITY_GUI);
        this->vectors[i]->setMaterial(material_list[i%5]);
        
        // if (this->sdf->HasElement("color")) {
        //     this->forceVector->setMaterial(this->sdf->Get<std::string>("color"));
        // } else {
            
        // }

        // if (this->sdf->HasElement("material")) {
        //     this->forceVector->setMaterial(this->sdf->Get<std::string>("material"));
        // }

        for(int k = 0; k < 6; ++k) { // -> needs three lines, so 6 points
            this->vectors[i]->AddPoint(ignition::math::Vector3d::Zero);
        }
        this->vectors[i]->Update();
    }
}


void MultivectorsVisual::OnUpdate(ConstMultipleVectorPtr& vectors_msgs)
{
    // gzdbg << "Receiving " << vectors_msgs->vectors_size() << " vectors" << std::endl;
    for (int i = 0; i < vectors_msgs->vectors_size(); i++)
    {
        ignition::math::Vector3d vector = msgs::ConvertIgn(vectors_msgs->vectors(i).vector());
        ignition::math::Vector3d center = msgs::ConvertIgn(vectors_msgs->vectors(i).center());

        const auto scale = this->visual->Scale();
        center = center / scale;

        this->UpdateVector(center, vector, this->vectors[i]);
    }
    

    // for(auto it = std::begin(vectors_msgs); it != std::end(vectors_msgs); ++it) {
    //     // std::cout << *it << "\n";
    //     ignition::math::Vector3d force;
    //     auto force_vector_msg = it->vector();
    //     force.Set(force_vector_msg.x(), force_vector_msg.y(), force_vector_msg.z());

    //     ignition::math::Vector3d center;
    //     auto force_center_msg = it->center();
    //     center.Set(force_center_msg.x(), force_center_msg.y(), force_center_msg.z());
        
    //     // Note that if the visual is scaled, the center needs to be scaled, too,
    //     // such that the force appears on the visual surface control and not on
    //     // the actual joint (which will look weird).

    //     const auto scale = this->visual->Scale();
    //     center = center / scale;
        
    //     this->UpdateVector(center, force, it);

    // }    
}

void MultivectorsVisual::UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force, rendering::DynamicLinesPtr& vis_vec)
{
    const float arrow_scale = 0.1;

    ignition::math::Vector3d begin = center;
    ignition::math::Vector3d end = center + force;
    
    vis_vec->SetPoint(0, begin);
    vis_vec->SetPoint(1, end);
    vis_vec->SetPoint(2, end);
    vis_vec->SetPoint(3, end - arrow_scale * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848) * (end - begin));
    vis_vec->SetPoint(4, end);
    vis_vec->SetPoint(5, end - arrow_scale * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848) * (end - begin));
}