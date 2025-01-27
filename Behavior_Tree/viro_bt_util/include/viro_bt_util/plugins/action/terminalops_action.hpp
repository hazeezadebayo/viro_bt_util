#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__TERMINALOPS_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__TERMINALOPS_ACTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <cstdlib>
#include <behaviortree_cpp/action_node.h>


namespace viro_bt_util
{

class TerminalOps : public BT::StatefulActionNode
{
public:
    TerminalOps(const std::string& name, const BT::NodeConfig& conf);
    ~TerminalOps();

    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    pid_t startProcess(const std::string& cmd);
    bool stopProcess(pid_t pid);
    bool isControlCommand(const std::string& cmd) const;

    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    // Static shared map to track task names and their PIDs
    static std::unordered_map<std::string, pid_t> shared_process_map_; 
};

// Initialize the static shared map
std::unordered_map<std::string, pid_t> TerminalOps::shared_process_map_;

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PLUGINS__ACTION__TERMINALOPS_ACTION_HPP_
