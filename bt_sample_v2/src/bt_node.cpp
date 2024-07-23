#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

using StdMsgString = std_msgs::msg::String;
using StdMsgBool = std_msgs::msg::Bool;
using namespace std;
using namespace BT;

namespace BTNodes
{
    class ReceiveString : public RosTopicSubNode<StdMsgString>
    {
        public:
            ReceiveString ( const string& name, const NodeConfig& config, const RosNodeParams& params )
                : RosTopicSubNode<StdMsgString> ( name, config, params ) {}

            static PortsList providedPorts ()
            {
                return {};
            }

            NodeStatus onTick ( const shared_ptr<StdMsgString>& last_msg ) override
            {
                if ( last_msg )  // empty if no new message received, since the last tick
                {
                    RCLCPP_INFO ( logger (), "[%s] new message: %s", name ().c_str (), last_msg->data.c_str () );
                }
                return NodeStatus::SUCCESS;
            }
    };

    class ReceiveData : public RosTopicSubNode<StdMsgBool>
    {
        public:
            ReceiveData ( const string& name, const NodeConfig& config, const RosNodeParams& params )
                : RosTopicSubNode<StdMsgBool> ( name, config, params ) {}

            static PortsList providedPorts ()
            {
                return providedBasicPorts ( { OutputPort<bool> ( "out_data" ) } );
            }

            NodeStatus onTick ( const shared_ptr<StdMsgBool>& last_msg ) override
            {
                if ( last_msg )
                {
                    RCLCPP_INFO ( logger (), "[%s] new message: %s", name ().c_str (), last_msg->data ? "true" : "false" );

                    setOutput ( "out_data", last_msg->data );
                }

                return NodeStatus::SUCCESS;
            }
    };

    class CallTest : public RosTopicPubNode<StdMsgBool>
    {
        public:
            CallTest ( const string& name, const NodeConfig& config, const RosNodeParams& params )
                : RosTopicPubNode<StdMsgBool> ( name, config, params ) {}

            static PortsList providedPorts ()
            {
                return providedBasicPorts ( { InputPort<bool> ( "in_data" ) } );
            }

            bool setMessage(StdMsgBool& msg) override
            {
                string message;
                Expected<bool> data = getInput<bool> ( "in_data" );
                if ( !data )
                {
                    // RCLCPP_ERROR(node_->get_logger(), "Missing required input [message]");
                    return false;
                }
                msg.data = data.value ();
                return true;
            }
    };

    class ContinueOnFailure : public SyncActionNode
    {
        public:
            ContinueOnFailure ( const string& name, const NodeConfig& config )
                : SyncActionNode ( name, config ) {}

            static PortsList providedPorts ()
            {
                return {};
            }

            NodeStatus tick () override
            {
                return NodeStatus::SUCCESS;
            }
    };
}