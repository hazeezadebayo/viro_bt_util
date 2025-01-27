#include "viro_bt_util/plugins/action/wait_action.hpp"

namespace viro_bt_util
{

// Constructor definition
Wait::Wait(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
}

// Define the static method to provide ports
BT::PortsList Wait::providedPorts()
{
    // This action has a single input port called "wait_seconds"
    return { BT::InputPort<int>("wait_seconds") };
}

// Override the tick() method
BT::NodeStatus Wait::tick()
{
    std::cout << "[" << this->name() << "] wait bt ticked..." << std::endl;

    BT::Expected<int> wait_seconds = getInput<int>("wait_seconds");
    // Check if expected is valid. If not, throw its error
    if (!wait_seconds)
    {
        throw BT::RuntimeError("missing required input [wait_seconds]: ", wait_seconds.error());
    }

    int duration = wait_seconds.value();
    RCLCPP_INFO(rclcpp::get_logger("Wait"), "All nodes will sleep for %d seconds", duration);

    // Sleep for the specified duration
    rclcpp::sleep_for(std::chrono::seconds(duration));

    // Use the method value() to extract the valid message.
    RCLCPP_INFO(rclcpp::get_logger("Wait"), "All nodes are waking up");
    return BT::NodeStatus::SUCCESS;
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::Wait>("Wait");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::Wait>("Wait");
}
