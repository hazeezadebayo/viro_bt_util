#ifndef VIRO_BT_UTIL__IS_EMERGENCY_CONDITION_HPP_
#define VIRO_BT_UTIL__IS_EMERGENCY_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "diffamr2_msgs/msg/emergency_status.hpp"

namespace viro_bt_util
{

/**
 * @brief A BT::ConditionNode that listens to an emergency topic and
 * returns SUCCESS when an emergency is detected and FAILURE otherwise
 */
class IsEmergencyCondition : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor for viro_bt_util::IsEmergencyCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsEmergencyCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsEmergencyCondition() = delete;

  /**
   * @brief A destructor for viro_bt_util::IsObstacleCondition
   */
  ~IsEmergencyCondition() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Initializes parameters and subscribers
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("emergency_topic", std::string("/emergency_status"), "Emergency topic")
    };
  }

private:
  /**
   * @brief Callback function for the emergency topic
   * @param msg Shared pointer to diffamr2_msgs::msg::EmergencyStatus message
   */
  void emergencyCallback(diffamr2_msgs::msg::EmergencyStatus::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<diffamr2_msgs::msg::EmergencyStatus>::SharedPtr emergency_sub_;

  std::string emergency_topic_;
  bool is_emergency_;
};

}  // namespace viro_bt_util

#endif // VIRO_BT_UTIL__IS_EMERGENCY_CONDITION_HPP_
