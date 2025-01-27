#include "viro_bt_util/plugins/condition/is_emergency_condition.hpp"

namespace viro_bt_util
{

IsEmergencyCondition::IsEmergencyCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  emergency_topic_("/emergency_status"),
  is_emergency_(false)
{
}

void IsEmergencyCondition::initialize()
{
  std::string emergency_topic_new;
  getInput("emergency_topic", emergency_topic_new);

  if (emergency_topic_new != emergency_topic_ || !emergency_sub_) {
    emergency_topic_ = emergency_topic_new;

    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    emergency_sub_ = node_->create_subscription<diffamr2_msgs::msg::EmergencyStatus>(
      emergency_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsEmergencyCondition::emergencyCallback, this, std::placeholders::_1),
      sub_option);
  }
  std::cout << "[" << this->name() << "] emergency bt joined node..." << std::endl;
}

IsEmergencyCondition::~IsEmergencyCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsEmergencyCondition BT node");
  callback_group_executor_.cancel();
}

BT::NodeStatus IsEmergencyCondition::tick()
{
  std::cout << "[" << this->name() << "] emergency bt ticked..." << std::endl;

  if (!BT::isStatusActive(status())) {
    std::cout << "[" << this->name() << "] emergency bt initialized..." << std::endl;
    initialize();
  }

  callback_group_executor_.spin_some();
  std::cout << "[" << this->name() << "] emergency bt spinned some..." << std::endl;

  if (is_emergency_) {
    std::cout << "[" << this->name() << "] emergency confirmed.." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsEmergencyCondition::emergencyCallback(diffamr2_msgs::msg::EmergencyStatus::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] evaluating emergency msg..." << std::endl;
  is_emergency_ = msg->emergencybutton || msg->safetybumper;
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::IsEmergencyCondition>("IsEmergency");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::IsEmergencyCondition>("IsEmergency");
}
