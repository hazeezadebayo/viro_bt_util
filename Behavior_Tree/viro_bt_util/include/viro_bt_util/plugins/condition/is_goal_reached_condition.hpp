#ifndef VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
#define VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h> // For quaternion math
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2 to geometry_msgs conversions
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>


namespace viro_bt_util
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class IsGoalReachedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for viro_bt_util::IsGoalReachedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalReachedCondition() = delete;

  /**
   * @brief A destructor for viro_bt_util::IsGoalReachedCondition
   */
  ~IsGoalReachedCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the current robot pose lies within a given distance from the goal
   * @return bool true when goal is reached, false otherwise
   */
  bool isGoalReached(const geometry_msgs::msg::PoseStamped& goal);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void quaternionToEuler(const tf2::Quaternion& quat, double& roll, double& pitch, double& yaw);


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("goal_reached_tol", "goal distance value below which we consider success."),
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:


  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose current_pose_;

  double goal_reached_tol_;

};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
