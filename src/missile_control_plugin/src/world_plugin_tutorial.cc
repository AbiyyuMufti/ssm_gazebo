#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace gazebo
{
    class WorldPluginTutorial : public WorldPlugin
    {

        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection;

        /// \brief Controller update mutex.
        boost::mutex mutex_;

        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;    

        /// \brief A ROS subscriber
        ros::Subscriber rosSub;

        ros::Publisher rosPub;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;

        public: WorldPluginTutorial() : WorldPlugin()
        {
            
        }

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            if(!ros::isInitialized())
            {
                ROS_FATAL_STREAM("Load ROS node for Gazebo First");
                return;
            }

            ROS_WARN("Hello World!!!!!!!!!!!!!!!!");
        }

        void OnUpdate()
        {

        }

        void QueueThread()
        {

        }

        void onSubsmsgs()
        {
            
        }
    };




    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}