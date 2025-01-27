#include "viro_bt_util/plugins/condition/is_goal_reached_condition.hpp"

#include <cmath>

namespace viro_bt_util
{

IsGoalReachedCondition::IsGoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  goal_reached_tol_(0.25)
{
}

void IsGoalReachedCondition::initialize()
{

  getInput("goal_reached_tol", goal_reached_tol_); // 0.25

  // Get the Node from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // Create a callback group to manage subscription callbacks
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);  // Non-blocking callback group
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  // you also need to subscribe to the odom node and use its callback to obtain the current
  // robots poses x y and theta
  // theta must have been obtained by quarternion to euler convertion.
  // Subscribe to odometry data to get the current robot pose
  // Subscribe to the odometry data to track the robot's current pose (x, y, theta)
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;  // Using callback group for thread safety if needed
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsGoalReachedCondition::odomCallback, this, std::placeholders::_1),
    sub_option);

  std::cout << "[" << this->name() << "] goal reached bt joined node..." << std::endl;


}

IsGoalReachedCondition::~IsGoalReachedCondition()
{
  // please implement a proper deconstructor here
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsGoalReachedCondition BT node");
  callback_group_executor_.cancel();
}

void IsGoalReachedCondition::quaternionToEuler(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

BT::NodeStatus IsGoalReachedCondition::tick()
{

  std::cout << "[" << this->name() << "] goal reached bt ticked..." << std::endl;

  if (!BT::isStatusActive(status())) {
    std::cout << "[" << this->name() << "] goal reached bt initialized..." << std::endl;
    initialize();
  }

  callback_group_executor_.spin_some();
  std::cout << "[" << this->name() << "] goal reached bt spinned some..." << std::endl;

  // i think you should use the blackboard to fetch goal inserted by navigation here
  // goal = config().blackboard->get<...
  // Retrieve the goal from the blackboard
  geometry_msgs::msg::PoseStamped goal;
  if (!config().blackboard->get("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve goal from blackboard");
    return BT::NodeStatus::FAILURE; // Exit the initialize function if the goal is not found
  }

  // Check if the goal is reached
  if (isGoalReached(goal)) {
    std::cout << "[" << this->name() << "] goal reached confirmed.." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsGoalReachedCondition::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] goal reached received odom msg..." << std::endl;
  current_pose_ = msg->pose.pose;  // Update current pose from odometry
}

bool IsGoalReachedCondition::isGoalReached(const geometry_msgs::msg::PoseStamped& goal)
{
  std::cout << "[" << this->name() << "] goal reached evaluating goal status..." << std::endl;

  // Ensure current pose is available
  // you need to get the odom and also get the x y theta from the goal in the blackboard
  // then you need to check if its within the tolerance like i have done below
  if (current_pose_.position.x == 0 && current_pose_.position.y == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  // Convert quaternions to Euler angles
  double goal_roll, goal_pitch, goal_yaw;
  tf2::Quaternion goal_q(
      goal.pose.orientation.x,
      goal.pose.orientation.y,
      goal.pose.orientation.z,
      goal.pose.orientation.w);
  quaternionToEuler(goal_q, goal_roll, goal_pitch, goal_yaw);   // Get goal's theta (yaw)

  // Print the received goal for debugging purposes
  RCLCPP_INFO(node_->get_logger(), "Received goal: x = %f, y = %f, z = %f, theta = %f",
              goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
              goal_yaw);  // Assuming orientation is stored as a quaternion

  // Calculate the distance between the robot and the goal
  double dx = goal.pose.position.x - current_pose_.position.x;
  double dy = goal.pose.position.y - current_pose_.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // Check if the distance is within the tolerance
  if (distance <= goal_reached_tol_) {
    RCLCPP_DEBUG(node_->get_logger(), "Goal reached with distance tolerance %f.", goal_reached_tol_);
  } else {
    return false;  // Return false if the distance condition is not satisfied
  }

  // you also have to use the theta like check if the robots current theta is within same tolerance of the blackboard's goal theta
  // Optionally: Check orientation (theta) using quaternion to euler conversion
  // Calculate the difference between current orientation and goal orientation
  double current_roll, current_pitch, current_yaw;
  tf2::Quaternion current_q(
      current_pose_.orientation.x,
      current_pose_.orientation.y,
      current_pose_.orientation.z,
      current_pose_.orientation.w);
  quaternionToEuler(current_q, current_roll, current_pitch, current_yaw);  // Get current robot's theta (yaw)

  double theta_diff = std::fabs(goal_yaw - current_yaw);
  if (theta_diff > M_PI) {
    theta_diff = 2 * M_PI - theta_diff;  // Wrap angle difference within [-PI, PI]
  }

  if (theta_diff <= goal_reached_tol_) {
    RCLCPP_DEBUG(node_->get_logger(), "Goal orientation reached with tolerance %f.", goal_reached_tol_);
    return true;  // Return true if both distance and orientation are within tolerance
  }

  return false;  // Return false if orientation condition is not satisfied
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::IsGoalReachedCondition>("IsGoalReached");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::IsGoalReachedCondition>("IsGoalReached");
}
