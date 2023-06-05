// used Professor's github code and modified as needed

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float x;
float y;
bool valid;

void poseReceived(const turtlesim::msg::Pose::SharedPtr msg) {
   x = msg->x;
   y = msg->y;
   valid = true;
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("replicate");

  auto sub = nodeh->create_subscription<turtlesim::msg::Pose>
                                  ("turtle1/pose",10,&poseReceived);
// use 'Twist' instead of 'Pose' to publish linear and angular velocities
  auto pub = nodeh->create_publisher<geometry_msgs::msg::Twist>("p2dx/cmd_vel",1000);
    
  geometry_msgs::msg::Twist twistToPublish;
  valid = false;
  
  while (rclcpp::ok()) {
    rclcpp::spin_some(nodeh);
    if (valid) {
      twistToPublish.linear.x = x;
      twistToPublish.angular.y = y;
      pub->publish(twistToPublish);
      valid = false;
    }
  }
}
