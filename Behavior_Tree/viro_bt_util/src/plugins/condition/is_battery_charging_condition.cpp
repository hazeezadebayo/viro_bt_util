#include "viro_bt_util/plugins/condition/is_battery_charging_condition.hpp"

#include <string>

namespace viro_bt_util
{

IsBatteryChargingCondition::IsBatteryChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  is_battery_charging_(false)
{

  getInput("battery_topic", battery_topic_);

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
    std::bind(&IsBatteryChargingCondition::batteryCallback, this, std::placeholders::_1),
    sub_option);

  std::cout << "[" << this->name() << "] charge bt joined node..." << std::endl;  

}

IsBatteryChargingCondition::~IsBatteryChargingCondition()
{
  // please implement a proper deconstructor here
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsBatteryChargingCondition BT node");
  callback_group_executor_.cancel();
}

BT::NodeStatus IsBatteryChargingCondition::tick()
{
  std::cout << "[" << this->name() << "] charge bt ticked..." << std::endl;

  callback_group_executor_.spin_some();
  std::cout << "[" << this->name() << "] charge bt spinned some..." << std::endl;

  if (is_battery_charging_) {
    std::cout << "[" << this->name() << "] charge status confirmed.." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryChargingCondition::batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] evaluating charge msg..." << std::endl;

  is_battery_charging_ =
    (msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::IsBatteryChargingCondition>("IsBatteryCharging");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::IsBatteryChargingCondition>("IsBatteryCharging");
}
