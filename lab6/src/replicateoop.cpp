
// used code from my lab3 replicate.cpp and modified when needed
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Define the ReplicateOOP class that inherits from the Node class
class ReplicateOOP : public rclcpp::Node {
public:
  ReplicateOOP();  // Constructor

private:
  // Callback function for handling received Pose messages
  void poseReceived(const turtlesim::msg::Pose::SharedPtr msg);

  // Declare a shared pointer for the Pose subscription object
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;

  // Declare a shared pointer for the Twist publisher object
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

// Define the constructor
ReplicateOOP::ReplicateOOP() : Node("replicateoop") {
  // Initialize the Pose subscriber with the poseReceived callback function
  pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&ReplicateOOP::poseReceived, this, std::placeholders::_1));
  
  // Initialize the Twist publisher
  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("p2dx/cmd_vel", 1000);
}

// Define the poseReceived callback function
void ReplicateOOP::poseReceived(const turtlesim::msg::Pose::SharedPtr msg) {
  // Create a Twist message to publish the linear and angular velocities
  geometry_msgs::msg::Twist twistToPublish;
  
  // Set the linear and angular velocities from the received Pose message
  twistToPublish.linear.x = msg->linear_velocity;
  twistToPublish.angular.z = msg->angular_velocity;

  // Publish the Twist message with the extracted velocities
  cmd_vel_publisher_->publish(twistToPublish);
}

// Define the main function
int main(int argc, char** argv) {
  // Initialize the ROS2 system
  rclcpp::init(argc, argv);

  // Create a shared pointer for the ReplicateOOP node
  auto replicateoop_node = std::make_shared<ReplicateOOP>();

  // Spin the node to process incoming messages and callbacks
  rclcpp::spin(replicateoop_node);

  rclcpp::shutdown();

  return 0;
}

