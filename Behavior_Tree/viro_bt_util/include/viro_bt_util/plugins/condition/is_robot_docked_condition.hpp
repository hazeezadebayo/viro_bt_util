#ifndef VIRO_BT_UTIL__PLUGINS__CONDITION__IS_ROBOT_DOCKED_CONDITION_HPP_
#define VIRO_BT_UTIL__PLUGINS__CONDITION__IS_ROBOT_DOCKED_CONDITION_HPP_

#include <string>
#include <unordered_map>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <yaml-cpp/yaml.h>

namespace viro_bt_util
{

class IsRobotDocked : public BT::ConditionNode
{
public:
  IsRobotDocked(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsRobotDocked() = delete;

  ~IsRobotDocked() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("localization"),
      BT::InputPort<std::string>("stations_yaml"),
      BT::InputPort<double>("pose_tolerance"),
      BT::OutputPort<int>("est_dock_id")
    };
  }

protected:
  void initialize();
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool isRobotDocked();
  bool checkPose(const geometry_msgs::msg::Pose& pose, double tolerance, int& dock_id);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  geometry_msgs::msg::Pose current_pose_;
  std::unordered_map<int, geometry_msgs::msg::Pose> station_positions_;
  std::string localization_;
  double pose_tolerance_;
};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__CONDITION__IS_ROBOT_DOCKED_CONDITION_HPP_
