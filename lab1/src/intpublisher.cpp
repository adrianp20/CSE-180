#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <std_msgs/msg/int32.hpp> // to publish integers
#include <std_msgs/msg/string.hpp> // to publish strings

int main(int argc,char **argv) {
  
  rclcpp::init(argc,argv); // initialize the ROS subsystem
  
  rclcpp::Node::SharedPtr nodeh;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubs;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pubi;
  rclcpp::Rate rate(2);

  nodeh = rclcpp::Node::make_shared("intpublisher"); // create node
    // create publisher to topic "inttopic" of integers
  pubi = nodeh->create_publisher<std_msgs::msg::Int32>("inttopic",10);
  
  int value=0;
  std_msgs::msg::Int32 intToSend; // integer message to send
  // std_msgs::msg::String stringToSend; // string message to send
  // stringToSend.data = "CSE180-Robotics"; // constant string to send
  
  while (rclcpp::ok()) {
    intToSend.data = value++; // update message to send
    pubi->publish(intToSend); // publish the integer message
    // pubs->publish(stringToSend); // publish the string message
    RCLCPP_INFO(nodeh->get_logger(),"Completed iteration  #%d",value);    	
    rate.sleep(); // wait
  }
  rclcpp::shutdown(); // unreachable in the current form
  return 0;
}
