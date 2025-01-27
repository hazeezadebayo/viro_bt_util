#ifndef VIRO_BT_UTIL__PLUGINS__CONDITION__IS_OBSTACLE_CONDITION_HPP_
#define VIRO_BT_UTIL__PLUGINS__CONDITION__IS_OBSTACLE_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace viro_bt_util
{

class IsObstacleCondition : public BT::ConditionNode
{
public:

  IsObstacleCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsObstacleCondition() = delete;

  ~IsObstacleCondition() override;

  void onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg);

  void onLaserScanReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  BT::NodeStatus tick() override;

  void logObstacle(const std::string & msg) const;

  void updateStates();

  bool isObstacle();

  bool checkShortDistance(const std::vector<float> &range_of_interest);

  bool checkStaticScanHistory();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("use_scan", false, "whether or not to use scan information."),
      BT::InputPort<double>("brake_accel_limit", "abrupt braking deceleration threshold trigger."),
      BT::InputPort<double>("min_range_percent", "Minimum robot scan range to be considered for warning trigger"),
      BT::InputPort<double>("max_range_percent", "Maximum robot scan range to be considered for warning trigger"),
      BT::InputPort<double>("warning_threshold", "distance value below which warning triggers.")
    };
  }

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::thread callback_group_executor_thread;

  std::atomic<bool> is_obstacle_;

  std::mutex mutex_;

  // Listen to odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // Listen to scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // Store history of odometry measurements
  std::deque<nav_msgs::msg::Odometry> odom_history_;
  std::deque<nav_msgs::msg::Odometry>::size_type odom_history_size_;

  // Store history of scan measurements
  std::deque<std::vector<float>> scan_history_;
  std::size_t scan_history_size_;

  // Calculated states
  double current_accel_;

  // Robot specific parameters
  double brake_accel_limit_;
  double min_range_percent_;
  double max_range_percent_;
  double warning_threshold_;
  bool use_scan_;
};

}  // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PLUGINS__CONDITION__IS_OBSTACLE_CONDITION_HPP_
