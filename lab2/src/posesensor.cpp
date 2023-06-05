// used code from professor's github and cross referenced textbook chapters
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

float x;
float y;
float theta;
bool valid;

void poseReceived(const turtlesim::msg::Pose::SharedPtr msg) {
   x = msg->x;
   y = msg->y;
   theta = msg->theta;
   valid = true;
}

int main(int argc,char ** argv) {

  rclcpp::init(argc,argv);
  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("posesensor");

  auto sub = nodeh->create_subscription<turtlesim::msg::Pose>
                                  ("/turtle1/pose",10,&poseReceived);
                                  
valid = false;

do {
  rclcpp::spin_some(nodeh);
  if (valid) {
    printf("2D Pose: x = %f, y = %f, theta = %f\n", x, y, theta);
    valid = false;
  }
} while (rclcpp::ok());

}
