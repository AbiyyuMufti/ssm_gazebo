#ifndef _GAZEBO_MULTIVECTORS_VISUAL_PLUGIN_HH_
#define _GAZEBO_MULTIVECTORS_VISUAL_PLUGIN_HH_

#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/TransportTypes.hh>

#include "sdf/sdf.hh"

#include "MultiplevectorVisual.pb.h"

namespace gazebo
{
    typedef const boost::shared_ptr<const ssm_msgs::msgs::MultiplevectorVisual> ConstMultipleVectorPtr;

    /// \brief A visual plugin that draws a force published on a topic
    class GAZEBO_VISIBLE MultivectorsVisual : public VisualPlugin
    {
        /// \brief Constructor.
        public: MultivectorsVisual();

        /// \brief Destructor.
        public: ~MultivectorsVisual();

        // Documentation Inherited.
        public: virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

        /// \brief Pointer to visual containing plugin.
        protected: rendering::VisualPtr visual;

        /// \brief Pointer to element containing sdf.
        protected: sdf::ElementPtr sdf;

        private: void OnUpdate(ConstMultipleVectorPtr& vectors_msgs);
        
        private: void UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force, rendering::DynamicLinesPtr& vis_vec);

        private: transport::SubscriberPtr subs;
        private: transport::NodePtr node;
        
        private: std::vector<rendering::DynamicLinesPtr> vectors;

    };
}


#endif