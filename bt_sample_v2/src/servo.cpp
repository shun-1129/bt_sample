#include <std_msgs/msg/string.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "bt_node.cpp"

using namespace std;
using namespace BT;
using namespace BTNodes;

int main ( int argc, char** argv )
{
    rclcpp::init ( argc, argv );

    auto pubNode = make_shared<rclcpp::Node> ( "publisher_test" );
    auto subNode = make_shared<rclcpp::Node> ( "subscriber_test" );

    auto node = make_shared<rclcpp::Node> ( "servo_test" );

    BehaviorTreeFactory factory;

    RosNodeParams subParams;
    subParams.nh = node;
    subParams.default_port_value = "keyboard_input1";
    factory.registerNodeType<ReceiveData> ( "ReceiveData", subParams );

    RosNodeParams pubParams;
    pubParams.nh = node;
    pubParams.default_port_value = "keyboard_input2";
    factory.registerNodeType<CallTest> ( "CallTest", pubParams );

    factory.registerNodeType<ContinueOnFailure> ( "ContinueOnFailure" );
    
    string package_path = ament_index_cpp::get_package_share_directory ( "bt_sample" );
    factory.registerBehaviorTreeFromFile( package_path + "/config/servo_node.xml" );
    Tree tree = factory.createTree ( "ServoBT" );

    // cout << "call CallTest" << endl;

    cout << "BehaviorTreeのツリー定義表示" << endl;
    printTreeRecursively(tree.rootNode());

    // rclcpp::spin_some ( rclcpp::Node::make_shared ( "behavior_tree_node" ) );
    // tree.tickOnce ();
    // rclcpp::shutdown ();

    /*
    while( rclcpp::ok () )
    {
        tree.tickWhileRunning ();
    }
    */

    while ( rclcpp::ok () )
    {
        tree.tickWhileRunning ();
        // tree.tickOnce ();
        std::this_thread::sleep_for ( chrono::milliseconds ( 10 ) );
    }

    rclcpp::shutdown ();

    return 0;
}