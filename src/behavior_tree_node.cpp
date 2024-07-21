#pragma once
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std;
using namespace BT;

namespace BTNode
{
    class FullWordCountJudge : public ConditionNode
    {
        public:
            FullWordCountJudge ( const string& name , const NodeConfig& config ) : ConditionNode ( name , config ) {}
            
            static PortsList providedPorts ()
            {
                return
                {
                    InputPort<string> ( "in_text" )
                };
            }

            NodeStatus tick () override
            {
                cout << "call FullWordCountJudge" << endl;

                Expected<string> msg = getInput<string> ( "in_text" );
                if ( !msg )
                {
                    // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [in_text]: ", msg.error() );
                }
                string in_text = msg.value();
                int string_length = in_text.length();
                
                if (string_length % 2 == 0)
                {
                    std::cout << "文字数は偶数だった" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    std::cout << "文字数は奇数だった" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }
    };

    class HalfWordCountJudg : public ConditionNode
    {
        public:
            HalfWordCountJudg ( const string& name , const NodeConfig& config ) : ConditionNode ( name , config ) {}
            
            static PortsList providedPorts ()
            {
                return
                {
                    InputPort<string> ( "in_text" )
                };
            }

            NodeStatus tick () override
            {
                cout << "call HalfWordCountJudg" << endl;

                Expected<string> msg = getInput<string> ( "in_text" );
                if ( !msg )
                {
                    // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [in_text]: ", msg.error() );
                }
                string in_text = msg.value();
                int string_length = in_text.length() / 2;
                
                if (string_length % 2 == 0)
                {
                    std::cout << "入力された文字の半分の文字数は偶数だった" << std::endl;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    std::cout << "入力された文字の半分の文字数は奇数だった" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
            }
    };

    class Display : public SyncActionNode
    {
        public:
            Display( const string& name , const NodeConfig& config ) : SyncActionNode ( name , config ) {}

            static PortsList providedPorts ()
            {
                return
                {
                    InputPort<string> ( "in_text" )
                };
            }

            NodeStatus tick () override
            {
                cout << "call Display" << endl;
                
                Expected<string> msg = getInput<string> ( "in_text" );
                if ( !msg )
                {
                    // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [in_text]: ", msg.error() );
                }
                string in_text = msg.value();

                cout << "入力された文字列は " << in_text << " でした。" << endl;
                return NodeStatus::SUCCESS;
            }
    };

    class SendText : public SyncActionNode
    {
        public:
            SendText( const string& name , const NodeConfig& config ) : SyncActionNode ( name , config )
            {
                m_node = rclcpp::Node::make_shared ( "send_text_node" );
                m_publisher = m_node->create_publisher<std_msgs::msg::String> ( "bt_topic", 10 );
            }

            static PortsList providedPorts ()
            {
                return
                {
                    InputPort<string> ( "in_text" )
                };
            }

            NodeStatus tick () override
            {
                cout << "call SendText" << endl;
                
                Expected<string> msg = getInput<string> ( "in_text" );
                if ( !msg )
                {
                    // Inputの値が適切でないときの処理
                    throw BT::RuntimeError("missing required input [in_text]: ", msg.error() );
                }

                std_msgs::msg::String message;
                message.data = msg.value();

                m_publisher->publish ( message );
                RCLCPP_INFO ( m_node->get_logger() , "Published message: %s" , message.data.c_str () );

                return NodeStatus::SUCCESS;
            }
        
        private:
            rclcpp::Node::SharedPtr m_node;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    };
}