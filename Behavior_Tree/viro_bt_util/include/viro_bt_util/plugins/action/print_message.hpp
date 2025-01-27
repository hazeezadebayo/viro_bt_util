#ifndef VIRO_BT_UTIL__PRINT_MESSAGE_HPP_
#define VIRO_BT_UTIL__PRINT_MESSAGE_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp/action_node.h"

namespace viro_bt_util
{

// SyncActionNode (synchronous action) with an input port.
class PrintMessage : public BT::SyncActionNode
{

public:
  // Constructor
  PrintMessage(const std::string& name, const BT::NodeConfig& conf);

  // Static method to define the ports
  static BT::PortsList providedPorts();

  // Override the virtual function tick()
  BT::NodeStatus tick() override;

};

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PRINT_MESSAGE_HPP_
