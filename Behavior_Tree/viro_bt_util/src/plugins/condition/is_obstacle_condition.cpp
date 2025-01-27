#include "viro_bt_util/plugins/condition/is_obstacle_condition.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace viro_bt_util
{

IsObstacleCondition::IsObstacleCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_obstacle_(false),
  odom_history_size_(10),
  scan_history_size_(10),
  current_accel_(0.0),
  brake_accel_limit_(-10.0),
  min_range_percent_(0.2),
  max_range_percent_(0.8),
  warning_threshold_(0.5),
  use_scan_(false)  // Ensuring initialization order matches declaration order
{
  getInput("use_scan", use_scan_);
  getInput("brake_accel_limit", brake_accel_limit_);
  getInput("min_range_percent", min_range_percent_);
  getInput("max_range_percent", max_range_percent_);
  getInput("warning_threshold", warning_threshold_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsObstacleCondition::onOdomReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_DEBUG(node_->get_logger(), "Initialized an IsObstacleCondition BT node");

  if (use_scan_) {
    laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsObstacleCondition::onLaserScanReceived, this, std::placeholders::_1),
      sub_option);
    RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting on odometry and scan...");
  }
  else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting on odometry...");
  }

  std::cout << "[" << this->name() << "] obstacle bt joined node..." << std::endl;

}



IsObstacleCondition::~IsObstacleCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsObstacleCondition BT node");
  callback_group_executor_.cancel();
  if (callback_group_executor_thread.joinable()) {
    callback_group_executor_thread.join();
  }
}

void IsObstacleCondition::onOdomReceived(const typename nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] evaluating odom msg..." << std::endl;
  RCLCPP_INFO_ONCE(node_->get_logger(), "Got odometry");

  std::lock_guard<std::mutex> lock(mutex_);  // Locking the mutex to ensure thread safety

  while (odom_history_.size() >= odom_history_size_) {
    odom_history_.pop_front();
  }

  odom_history_.push_back(*msg);

  updateStates();
}

void IsObstacleCondition::onLaserScanReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] evaluating scan msg..." << std::endl;

  std::lock_guard<std::mutex> lock(mutex_);

  int start_index = std::max(0, static_cast<int>(min_range_percent_ * msg->ranges.size()));
  int end_index = std::min(static_cast<int>(msg->ranges.size()), static_cast<int>(max_range_percent_ * msg->ranges.size()));

  std::vector<float> range_of_interest(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);

  while (scan_history_.size() >= scan_history_size_) {
    scan_history_.pop_front();
  }

  scan_history_.push_back(range_of_interest);
}

BT::NodeStatus IsObstacleCondition::tick()
{
  std::cout << "[" << this->name() << "] obstcale bt ticked..." << std::endl;

  std::lock_guard<std::mutex> lock(mutex_);

  if (is_obstacle_) {
    logObstacle("Robot got obstacle!");
    return BT::NodeStatus::SUCCESS;  // Successfully detected a obstacle condition
  }

  logObstacle("Robot is free");
  return BT::NodeStatus::FAILURE;  // Failed to detected a obstacle condition
}

void IsObstacleCondition::logObstacle(const std::string & msg) const
{
  static std::string prev_msg;

  if (msg == prev_msg) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  prev_msg = msg;
}

void IsObstacleCondition::updateStates()
{
  std::cout << "[" << this->name() << "] obstcale updatestate called..." << std::endl;
  // Approximate acceleration
  if (odom_history_.size() > 2) {
    auto curr_odom = odom_history_.end()[-1];
    double curr_time = static_cast<double>(curr_odom.header.stamp.sec);
    curr_time += (static_cast<double>(curr_odom.header.stamp.nanosec)) * 1e-9;

    auto prev_odom = odom_history_.end()[-2];
    double prev_time = static_cast<double>(prev_odom.header.stamp.sec);
    prev_time += (static_cast<double>(prev_odom.header.stamp.nanosec)) * 1e-9;

    double dt = curr_time - prev_time;
    double vel_diff = static_cast<double>(
      curr_odom.twist.twist.linear.x - prev_odom.twist.twist.linear.x);
    current_accel_ = vel_diff / dt;
  }

  is_obstacle_ = isObstacle();
}

bool IsObstacleCondition::checkShortDistance(const std::vector<float> &range_of_interest)
{
  std::cout << "[" << this->name() << "] obstcale distance check called..." << std::endl;

  for (float range : range_of_interest) {
    if (range < warning_threshold_) {
      return true;
    }
  }
  return false;
}

bool IsObstacleCondition::checkStaticScanHistory()
{
  std::cout << "[" << this->name() << "] obstcale scan history check called..." << std::endl;

  if (scan_history_.size() < 2) {
    return false;
  }

  const float threshold = 0.05f;  // Tolerance for small changes
  for (size_t i = 1; i < scan_history_.size(); ++i) {

    const auto& prev_scan = scan_history_[i - 1];
    const auto& curr_scan = scan_history_[i];

    if (prev_scan.size() != curr_scan.size()) {
      return false;
    }

    // Compare each distance value with a threshold
    for (size_t j = 0; j < curr_scan.size(); ++j) {
      if (std::abs(curr_scan[j] - prev_scan[j]) > threshold) {
        return false;  // If the difference exceeds the threshold, return false
      }
    }
  }

  return true;
}

bool IsObstacleCondition::isObstacle()
{
  std::cout << "[" << this->name() << "] isobstcale or not..." << std::endl;

  // The robot getting stuck can result on different types of motion
  // depending on the state prior to getting stuck (sudden change in accel, not moving at all,
  // random oscillations, etc). For now, we only address the case where there is a sudden
  // harsh deceleration. A better approach to capture all situations would be to do a forward
  // simulation of the robot motion and compare it with the actual one.

  if (use_scan_) {
    // all three must agree for a positive result.
    return (current_accel_ < brake_accel_limit_)  || checkShortDistance(scan_history_.back()) || checkStaticScanHistory();
  }
  else {
    // Detect if robot bumped into something by checking for abnormal deceleration
    if (current_accel_ < brake_accel_limit_) {
      RCLCPP_DEBUG(node_->get_logger(), "Current deceleration is beyond brake limit."
        " brake limit: %.2f, current accel: %.2f", brake_accel_limit_, current_accel_);
      return true;
    }
    return false;
  }
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::IsObstacleCondition>("isObstacle");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::IsObstacleCondition>("isObstacle");
}
