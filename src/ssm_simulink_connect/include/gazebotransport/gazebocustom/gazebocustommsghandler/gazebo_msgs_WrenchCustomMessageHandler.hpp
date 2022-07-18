/* Copyright 2019-2020 The MathWorks, Inc. */
//This function is for internal use only. It may be removed in the future.
// DO NOT EDIT!

#ifndef GAZEBO_MSGS_WRENCHCUSTOMMESSAGEHANDLER_HPP
#define GAZEBO_MSGS_WRENCHCUSTOMMESSAGEHANDLER_HPP

#include "wrench.pb.h"
#include "gazebotransport/gazebocustom/CustomMsgHandler.hpp"
#include "gazebotransport/gazebocustom/CustomMsgDispatcher.hpp"
// gazebo
#include <gazebo/common/Time.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Image.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//ignition
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
//boost
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
//others
#include <map>
#include <vector>
#include <mutex>
#include <memory>
#include <cstdint>
#include <string>
#include <stdio.h>

namespace robotics {
    namespace gazebotransport {
        typedef const boost::shared_ptr<const gazebo::msgs::Wrench> gazebo_msgs_WrenchCustomPtr;
        
        /**
         * This class initiates Gazebo Subscriber for each topic name.
         * Further, it start callback, which subscribe custom message
         * and stores in the Custom Message Container.
         * */
        class gazebo_msgs_WrenchSubscriber {
        public:
            /// Constructor
            /*
			@param topicName                Topic name for custom message subscriber
			@param node                     Gazebo transport node pointer
			@param customContainer          Custom message storage instance
             */
            explicit gazebo_msgs_WrenchSubscriber(std::string const& topicName)
            : m_topicName(topicName)
            , m_subscriberPtr(nullptr)
            , m_messageString("")
            , m_isNew(false)
            , m_mutex() {
            }
            
            /// Destructor
            virtual ~gazebo_msgs_WrenchSubscriber() {
            }
            
            // subscriber callback which subscribe custom message from specific topic
            // and stores in the custom message container
            void OnUpdate(robotics::gazebotransport::gazebo_msgs_WrenchCustomPtr& _msg) {
                {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_messageString = _msg->SerializeAsString();
                m_isNew = true;
                }
            }
            
            // initialize gazebo subscriber pointer
            void InitImpl(gazebo::transport::NodePtr node) {
                if (!m_subscriberPtr) {
                    m_subscriberPtr = node->Subscribe(m_topicName, &gazebo_msgs_WrenchSubscriber::OnUpdate, this);
                }
            }
            
            // set isNew field to false
            void resetMessageStatus()
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_isNew = false;
            }
            
            // return current status of message ( isNew )
            bool getMessageStatus()
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                return m_isNew;
            }
            
            // return received message string
            std::string getMessage() {
                {
                std::lock_guard<std::mutex> lock(m_mutex);
                return m_messageString;
                }
            }
            
        private:
            // topic name of subscriber
            std::string m_topicName;
            
            // gazebo subscriber pointer
            gazebo::transport::SubscriberPtr m_subscriberPtr;
            
            // received message string
            std::string m_messageString;
            
            // status of message
            bool m_isNew;
            
            std::mutex m_mutex;
        };
        
        /**
         * This class is created to test custom message handler functionality.
         * The custom message handlers are created on gazebogenmsg() call
         * These handlers will be same as this.
         * The gazebo_msgs_WrenchCustomMessageHandler initialize gazebo publisher, subscriber.
         * Further, it publish and subscribe custom message on/from gazebo.
         * It can support multiple topics of same message type.
         * */
        class gazebo_msgs_WrenchCustomMessageHandler : public CustomMsgHandler {
        public:
            /// Constructor
            /*
			@param node                     Gazebo transport node pointer
			@param customContainer          Custom message storage instance
             */
            explicit gazebo_msgs_WrenchCustomMessageHandler(gazebo::transport::NodePtr node);
            
            /// Destructor
            ~gazebo_msgs_WrenchCustomMessageHandler();
            
            // initialize custom message publisher
            std::string initPublisher(mw::internal::robotics::gazebotransport::Packet const& msgContent);
            
            // initialize custom message subscriber
            std::string initSubscriber(mw::internal::robotics::gazebotransport::Packet const& msgContent);
            
            // publish custom message
            std::string publishCustomMsg(mw::internal::robotics::gazebotransport::Packet const& msgContent);
            
            // subscribe custom message
            std::pair<bool,std::string> subscribeCustomMsg(mw::internal::robotics::gazebotransport::Packet const& msgContent);
            
            // store subscriber instance into subscriber map based on topic name
            std::pair<std::shared_ptr<gazebo_msgs_WrenchSubscriber>, bool> insert(
                    std::string const& topic_name,
                    std::shared_ptr<gazebo_msgs_WrenchSubscriber> subsciberPointer);
                    
            // return subscriber instance stored in map based on topic name
            std::shared_ptr<gazebo_msgs_WrenchSubscriber> find(std::string const& topic);
                    
            // store gazebo publisher pointer in the map based on topic name
            void setCustomPub(gazebo::transport::PublisherPtr customPubPtr, std::string const& topicName);
                    
            // get publisher pointer from map based on topic name
            gazebo::transport::PublisherPtr getCustomPub(std::string const& topicName);
                    
        private:
            // gazebo transport node
            gazebo::transport::NodePtr m_node;
            
            // Stores custom message subscriber instance based on topic name
            std::map<std::string, std::shared_ptr<gazebo_msgs_WrenchSubscriber>> m_subscriberPtrMap;
            
            // Storage map of custom message publisher pointer
            std::map<std::string, gazebo::transport::PublisherPtr> m_customPublisherMap;
            
            std::mutex m_mutex;
        };
        
    } // namespace gazebotransport
} // namespace robotics

#endif

