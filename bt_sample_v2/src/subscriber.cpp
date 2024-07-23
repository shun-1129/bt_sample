#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using StdMsgBool = std_msgs::msg::Bool;
class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode () : Node ( "keyboard_subscriber_node" )
    {
        subscription_ = this->create_subscription<StdMsgBool> (
            "keyboard_input2", 10,
            std::bind ( &SubscriberNode::topic_callback, this, std::placeholders::_1 ) );
    }

private:
    void topic_callback ( const StdMsgBool::SharedPtr msg ) const
    {
        RCLCPP_INFO ( this->get_logger (), "Received: '%s'", msg->data ? "true" : "false" );
    }

    rclcpp::Subscription<StdMsgBool>::SharedPtr subscription_;
};

int main ( int argc, char *argv[] )
{
    rclcpp::init ( argc, argv );

    auto node = std::make_shared<SubscriberNode> ();

    rclcpp::spin ( node );
    rclcpp::shutdown ();
    return 0;
}
