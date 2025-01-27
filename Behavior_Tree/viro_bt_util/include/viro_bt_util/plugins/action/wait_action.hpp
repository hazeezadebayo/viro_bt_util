#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__WAIT_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__WAIT_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"

namespace viro_bt_util
{

// SyncActionNode (synchronous action) with an input port.
class Wait : public BT::SyncActionNode
{

public:
  // Constructor
  Wait(const std::string& name, const BT::NodeConfig& conf);

  // Static method to define the ports
  static BT::PortsList providedPorts();

  // Override the virtual function tick()
  BT::NodeStatus tick() override;

};

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PLUGINS__ACTION__WAIT_ACTION_HPP_
