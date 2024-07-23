#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <iostream>

using StdMsgBool = std_msgs::msg::Bool;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode () : Node ( "keyboard_input_node" )
    {
        publisher_ = this->create_publisher<StdMsgBool> ( "keyboard_input1", 10 );
        this->declare_parameter<std::string> ("prompt", "Enter a message: " );
    }

    void run ()
    {
        std::string input;
        std::string prompt;
        this->get_parameter ( "prompt", prompt );

        while ( rclcpp::ok () )
        {
            std::cout << prompt;
            std::getline( std::cin, input );

            auto message = StdMsgBool ();
            
            if ( input == "true" || input == "t" )
            {
                message.data = true;
            }
            else
            {
                message.data = false;
            }

            RCLCPP_INFO ( this->get_logger (), "Publishing: '%s'", message.data ? "true" : "false" );
            publisher_->publish ( message );

            rclcpp::spin_some ( this->get_node_base_interface () );
        }
    }

private:
    rclcpp::Publisher<StdMsgBool>::SharedPtr publisher_;
};

int main( int argc, char *argv[] )
{
    rclcpp::init ( argc, argv );

    auto node = std::make_shared<PublisherNode> ();
    node->run ();

    rclcpp::shutdown ();
    return 0;
}
