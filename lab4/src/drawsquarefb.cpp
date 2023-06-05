// used code from professors MRTP github (drawsquarefb.cpp and convert.cpp)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// quaternion to yaw angle
tf2::Quaternion q;
bool valid = false;
float x = 0.0, y = 0.0;
float start_x = 0.0, start_y = 0.0;
int direction = 0;
// desired directions
double DIRS[] = {0, M_PI/2, M_PI, -M_PI/2}; // desired directions
// use double values because when being built it complained
double roll = 0.0, pitch = 0.0, theta = 0.0;


void odomReceived(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // converts a message quaternion into an instance of quaternion
  tf2::convert(msg->pose.pose.orientation,q);
  valid = true;
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  // convert quaternion to yaw angle
  tf2::Matrix3x3 m(q);
//   doubles that represent roll, pitch, and yaw
  m.getRPY(roll, pitch, theta);
}

int main(int argc, char ** argv) {

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("drawsquarefb");

  auto cmd_pub = node->create_publisher<geometry_msgs::msg::Twist>("/p2dx/cmd_vel", 10);
  auto sub = node->create_subscription<nav_msgs::msg::Odometry>("/p2dx/odom", 10, odomReceived);

  // wait for the first odometry message to arrive
  while (!valid && rclcpp::ok()) {
    rclcpp::spin_some(node);
  }
  start_x = x;
  start_y = y;

  rclcpp::Rate rate(10); // 10Hz

while (rclcpp::ok()) {
  rclcpp::spin_some(node);

  geometry_msgs::msg::Twist cmd_vel;

  // calculate distance from starting position
  float distance = hypotf((x - start_x), (y - start_y));

  // move forward
  if (distance < 2) {
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = 0.0;
  }
  // rotate 90 degrees
  else if (theta < DIRS[direction] - 0.05) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.5;
  }
  // move forward again
  else if (distance < 4) {
    cmd_vel.linear.x = 0.2;
    cmd_vel.angular.z = 0.0;
  }
  // rotate 90 degrees again
  else if (theta < DIRS[(direction + 1) % 4] - 0.05) {
    direction = (direction + 1) % 4;
    start_x = x;
    start_y = y;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.5;
  }
  // stop moving
  else {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  cmd_pub->publish(cmd_vel);

  rate.sleep();
}

  rclcpp::shutdown();

  return 0;
}
