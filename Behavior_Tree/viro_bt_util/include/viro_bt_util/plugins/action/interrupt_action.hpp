#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__INTERRUPT_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__INTERRUPT_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/action_node.h"

namespace viro_bt_util
{

class Interrupt : public BT::SyncActionNode
{

public:
    // Constructor
    Interrupt(const std::string& name, const BT::NodeConfig& conf);

    // Destructor
    ~Interrupt() override;

    // Static method to define the ports
    static BT::PortsList providedPorts();

    // Override the virtual function tick()
    BT::NodeStatus tick() override;

private:
    // Callback function for the subscription
    void interruptCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    // Shared variable for interrupt event
    std::string interrupt_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

};

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PLUGINS__ACTION__INTERRUPT_ACTION_HPP_
