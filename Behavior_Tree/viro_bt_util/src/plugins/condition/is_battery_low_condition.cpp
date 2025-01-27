#include "viro_bt_util/plugins/condition/is_battery_low_condition.hpp"

namespace viro_bt_util
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
}

void IsBatteryLowCondition::initialize()
{
  getInput("min_battery", min_battery_);
  std::string battery_topic_new;
  getInput("battery_topic", battery_topic_new);
  getInput("is_voltage", is_voltage_);

  // Only create a new subscriber if the topic has changed or subscriber is empty
  if (battery_topic_new != battery_topic_ || !battery_sub_) {
    battery_topic_ = battery_topic_new;

    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
      battery_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsBatteryLowCondition::batteryCallback, this, std::placeholders::_1),
      sub_option);
  }
  
  std::cout << "[" << this->name() << "] battery bt joined node..." << std::endl;
  
}

IsBatteryLowCondition::~IsBatteryLowCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsBatteryLowCondition BT node");
  callback_group_executor_.cancel();
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  std::cout << "[" << this->name() << "] battery bt ticked..." << std::endl;
  
  if (!BT::isStatusActive(status())) {
    std::cout << "[" << this->name() << "] battery bt initialized..." << std::endl;
    initialize();
  }

  callback_group_executor_.spin_some();
  std::cout << "[" << this->name() << "] battery bt spinned some..." << std::endl;

  if (is_battery_low_) {
    std::cout << "[" << this->name() << "] battery confirmed low.." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] evaluating battery msg..." << std::endl;

  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::IsBatteryLowCondition>("IsBatteryLow");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::IsBatteryLowCondition>("IsBatteryLow");
}
