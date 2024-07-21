#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behavior_tree_node.cpp"
#include <string>

using namespace std;
using namespace BT;
using namespace BTNode;

class BehaviorTreeNode : public rclcpp::Node
{
    public:
        BehaviorTreeNode () : Node ( "sample_no01" )
        {
            m_subscription = this->create_subscription<std_msgs::msg::String> ( "trigger_topic", 10, bind ( &BehaviorTreeNode::topic_callback , this , placeholders::_1 ) );

            m_Factory.registerNodeType<Display> ( "Display" );

            string package_path = ament_index_cpp::get_package_share_directory ( "bt_sample" );            
            m_Factory.registerBehaviorTreeFromFile( package_path + "/config/sample_no01.xml" );
            m_tree = m_Factory.createTree ( "SampleNo01" );

            cout << "BehaviorTreeのツリー定義表示" << endl;
            printTreeRecursively(m_tree.rootNode());
        }

    private:
        void topic_callback ( const std_msgs::msg::String::SharedPtr msg )
        {
            RCLCPP_INFO ( this->get_logger () , "Received message: '%s'" , msg->data.c_str () );
            m_tree.rootBlackboard ()->set ( "text_data" , msg->data.c_str () );

            m_tree.tickOnce ();
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;
        BehaviorTreeFactory m_Factory;
        Tree m_tree;
};

int main( int argc , char** argv )
{
    rclcpp::init ( argc , argv );
    auto node = make_shared<BehaviorTreeNode> ();
    rclcpp::spin ( node );
    rclcpp::shutdown ();
    return 0;
}
