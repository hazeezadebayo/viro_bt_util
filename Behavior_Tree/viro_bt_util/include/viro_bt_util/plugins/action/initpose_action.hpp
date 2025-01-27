#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__INIT_POSE_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__INIT_POSE_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_cpp/action_node.h"

namespace viro_bt_util
{

class InitPose : public BT::SyncActionNode
{
public:
  // Constructor
  InitPose(const std::string & name, const BT::NodeConfig & conf);

  // Destructor
  ~InitPose() override;

  // Static method to define the ports
  static BT::PortsList providedPorts();

  // Override the virtual function tick()
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  void publishInitialPose(double x, double y, double z, double w);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool initial_pose_received_;
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__ACTION__INIT_POSE_ACTION_HPP_
