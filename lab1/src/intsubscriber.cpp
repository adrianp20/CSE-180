
#include <rclcpp/rclcpp.hpp> // needed for basic functions
#include <std_msgs/msg/int32.hpp> // to receive integers
#include <std_msgs/msg/string.hpp> // to receive strings

int last_int = -1, second_last_int = -1;
rclcpp::Node::SharedPtr nodeh;

// callback function called every time a message is received from the
// topic "stringm"
// void stringCallback(const std_msgs::msg::String::SharedPtr msg) {
//   // print received string to the screen
//   RCLCPP_INFO(nodeh->get_logger(),"Received string: %s",msg->data.c_str());
// }

// callback function called every time a message is received from the
// topic "intm"
void intCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  // // print received integer to the screen
  // RCLCPP_INFO(nodeh->get_logger(),"Received integer: %d",msg->data);
  // print the sum of the last 2 received integers
  // RCLCPP_INFO(nodeh->get_logger(),"Received integer: %d",msg->data);
    second_last_int = last_int;
    last_int = msg->data;
    if (second_last_int >= 0) {
        RCLCPP_INFO(rclcpp::get_logger("intsubscriber"), "Sum: %d", last_int + second_last_int);
    }

}


int main(int argc,char **argv) {


  rclcpp::init(argc,argv); // initialize ROS subsystem
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subi;
  nodeh = rclcpp::Node::make_shared("intsubscriber"); // create node instance
    // subscribe to topic "intm" an register the callback function
  subi = nodeh->create_subscription<std_msgs::msg::Int32>
                                         ("inttopic",10,&intCallback);

  // take the sum of the last two received integers and publish it
  // to the topic "inttopic"



  rclcpp::spin(nodeh); // wait for messages and process them
 
  rclcpp::shutdown();
  return 0;
  
}
