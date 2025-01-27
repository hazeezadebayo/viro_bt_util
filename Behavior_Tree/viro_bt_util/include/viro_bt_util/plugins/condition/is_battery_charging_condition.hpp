#ifndef VIRO_BT_UTIL__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
#define VIRO_BT_UTIL__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace viro_bt_util
{

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is charging and FAILURE otherwise
 */
class IsBatteryChargingCondition : public BT::ConditionNode
{

public:
  /**
   * @brief A constructor for viro_bt_util::IsBatteryChargingCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBatteryChargingCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryChargingCondition() = delete;

  /**
   * @brief A destructor for viro_bt_util::IsObstacleCondition
   */
  ~IsBatteryChargingCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic")
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */
  void batteryCallback(sensor_msgs::msg::BatteryState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

  std::string battery_topic_;
  bool is_battery_charging_;

};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
