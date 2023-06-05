// Include necessary headers and messages
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <navigation/navigation.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Define the ExtraPostFinder class, derived from rclcpp::Node
class ExtraPostFinder : public rclcpp::Node {
public:
  // Constructor for ExtraPostFinder class
  ExtraPostFinder()
    : rclcpp::Node("extra_post_finder"), navigator_(this), extra_post_found_(false) {
    // Create subscriptions for Lidar, map, and robot pose
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&ExtraPostFinder::lidarCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&ExtraPostFinder::mapCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", 10, std::bind(&ExtraPostFinder::amclPoseCallback, this, std::placeholders::_1));
  }

  // Public member functions for navigation and post finding status
  void SetInitialPose(const geometry_msgs::msg::Pose::SharedPtr& pose) {
    navigator_.SetInitialPose(pose);
  }

  // Wait until the navigation system is active and ready to receive commands
  void WaitUntilNav2Active() {
    // Call the corresponding function from the Navigator object
    navigator_.WaitUntilNav2Active();
  }

  // Send a navigation goal to the robot to move to the specified pose
  void GoToPose(const geometry_msgs::msg::Pose::SharedPtr& goal_pose) {
    // Call the corresponding function from the Navigator object with the given goal pose
    navigator_.GoToPose(goal_pose);
  }

  // Check if the robot's current navigation task is complete
  bool IsTaskComplete() {
    // Return the task completion status from the Navigator object
    return navigator_.IsTaskComplete();
  }

  // Check if the extra post has been found
  bool IsExtraPostFound() const {
    // Return the status of the extra_post_found_ member variable
    return extra_post_found_;
  }

  // Get the position of the extra post if it has been found
  geometry_msgs::msg::Pose GetExtraPostPosition() const {
    // Return the value of the extra_post_position_ member variable
    return extra_post_position_;
  }

private:
  // Callback functions for subscriptions
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    const double min_distance_threshold = 0.5;
    const double max_distance_threshold = 2.0;
    const double angle_threshold = 0.1;

    // Iterate through the range data in the received laser scan message
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      // Get the range value at the current index (i)
      float range = scan->ranges[i];
      // Calculate the corresponding angle for the current index (i) using the minimum angle and angle increment from the laser scan message
      float angle = scan->angle_min + i * scan->angle_increment;

      // Check for a post within the specified distance range and angle
      if (range > min_distance_threshold && range < max_distance_threshold && std::abs(angle) < angle_threshold) {
        extra_post_found_ = true;

        // Calculate the extra post position in the robot's coordinate frame
        double post_x = range * std::cos(angle);
        double post_y = range * std::sin(angle);

        // Transform the post position to the map coordinate frame using the robot's estimated pose
        // Note: This requires you to store the robot's estimated pose from amclPoseCallback
        extra_post_position_.position.x = robot_pose_.position.x + post_x * std::cos(robot_pose_.orientation.w) - post_y * std::sin(robot_pose_.orientation.w);
        extra_post_position_.position.y = robot_pose_.position.y + post_x * std::sin(robot_pose_.orientation.w) + post_y * std::cos(robot_pose_.orientation.w);

        break;
      }
    }
  }

  // Callback function to process the map data from the OccupancyGrid message
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    // Store the received map data in the map_data_ member variable
    map_data_ = map;
  }

  // Callback function to process the robot's pose data from the PoseWithCovarianceStamped message
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
    // Store the received robot pose in the robot_pose_ member variable
    robot_pose_ = pose->pose.pose;
  }

  // Declare shared pointers for Lidar, map, and robot pose subscriptions
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  // Declare a Navigator object to handle navigation tasks
  Navigator navigator_;

  // Declare shared pointers for storing map data and robot pose
  nav_msgs::msg::OccupancyGrid::SharedPtr map_data_;
  geometry_msgs::msg::Pose robot_pose_;
  // Declare a Pose object to store the position of the extra post
  geometry_msgs::msg::Pose extra_post_position_;
  // Declare a boolean variable to indicate if an extra post is found
  bool extra_post_found_;
};

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  // Create an instance of the ExtraPostFinder node
  auto extra_post_finder = std::make_shared<ExtraPostFinder>();

  // Start with initial position and orientation
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  extra_post_finder->SetInitialPose(init);

  // Wait for the navigation system to be active
  extra_post_finder->WaitUntilNav2Active();
  
  // Define a list of goal positions for the robot to navigate to
  std::vector<geometry_msgs::msg::Pose::SharedPtr> goal_positions = {
    std::make_shared<geometry_msgs::msg::Pose>(),
    std::make_shared<geometry_msgs::msg::Pose>(),
    std::make_shared<geometry_msgs::msg::Pose>(),
  };

  // Set the goal positions and orientations
  goal_positions[0]->position.x = 2;
  goal_positions[0]->position.y = 1;
  goal_positions[0]->orientation.w = 1;

  goal_positions[1]->position.x = 2;
  goal_positions[1]->position.y = -1;
  goal_positions[1]->orientation.w = 1;

  goal_positions[2]->position.x = 0;
  goal_positions[2]->position.y = 0;
  goal_positions[2]->orientation.w = 1;

  // Navigate to each goal position
  for (const auto& goal_pos : goal_positions) {
    extra_post_finder->GoToPose(goal_pos);
    // Continuously process callbacks while the task is not complete
    while (!extra_post_finder->IsTaskComplete()) {
      rclcpp::spin_some(extra_post_finder);
    }
  }

  // Output the position of the extra post if found
  if (extra_post_finder->IsExtraPostFound()) {
    auto extra_post_position = extra_post_finder->GetExtraPostPosition();
    RCLCPP_INFO(extra_post_finder->get_logger(), "Extra post found at x: %.2f, y: %.2f",
                extra_post_position.position.x, extra_post_position.position.y);
  } else {
    // Display a warning if the extra post was not found
    RCLCPP_WARN(extra_post_finder->get_logger(), "Extra post not found.");
  }

  // Shut down ROS 2 and return success
  rclcpp::shutdown();
  return 0;
}
