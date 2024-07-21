#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

class PublisherNode : public rclcpp::Node
{
  public:
    PublisherNode () : Node ( "publisher_node" )
    {
      publisher_ = this->create_publisher<std_msgs::msg::String> ( "trigger_topic" , 10 );
      RCLCPP_INFO ( this->get_logger(), "Keyboard Node has been started." );
      this->timer_ = this->create_wall_timer (
        std::chrono::milliseconds ( 100 ),
        std::bind ( &PublisherNode::publish_input , this ) );
    }

  private:
    void publish_input ()
    {
      std::string input;
      std::getline ( std::cin , input );
      
      auto message = std_msgs::msg::String ();
      message.data = input;

      publisher_->publish ( message );
      RCLCPP_INFO ( this->get_logger () , "Published: '%s'" , message.data.c_str () );
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main ( int argc , char * argv[] )
{
  rclcpp::init ( argc , argv );
  auto node = std::make_shared<PublisherNode> ();
  rclcpp::spin ( node );
  rclcpp::shutdown ();
  return 0;
}
