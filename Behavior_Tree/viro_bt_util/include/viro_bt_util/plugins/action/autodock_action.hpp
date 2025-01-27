#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__AUTODOCK_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__AUTODOCK_ACTION_HPP_

#include <string>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <viro_msgs/srv/vdock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <behaviortree_cpp/action_node.h>

namespace viro_bt_util
{

class AutoDock : public BT::StatefulActionNode
{
public:
    // Constructor for AutoDock
    AutoDock(const std::string& name, const BT::NodeConfig& conf);

    // A destructor for viro_bt_util::AutoDock
    ~AutoDock() override;

    // Static method to define the ports
    static BT::PortsList providedPorts();

protected:
    // A derived class of StatefulActionNode must override the following virtual methods, instead of tick():
    // NodeStatus onStart(): called when the Node was in IDLE state. It may succeed or fail immediately or return RUNNING. In the latter case, the next time the tick is received the method onRunning will be executed.
    // NodeStatus onRunning(): called when the Node is in RUNNING state. Return the new status.
    // void onHalted(): called when this Node was aborted by another Node in the tree.
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override; // Override the halt method since its async

private:
    rclcpp::Node::SharedPtr node_; ///< Pointer to the ROS node
    rclcpp::CallbackGroup::SharedPtr callback_group_; ///< Callback group for service clients
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_; ///< Executor for the callback group

    rclcpp::Client<viro_msgs::srv::Vdock>::SharedPtr dock_client_; ///< Client for dock service
    rclcpp::Client<viro_msgs::srv::Vdock>::SharedPtr undock_client_; ///< Client for undock service
    rclcpp::Client<viro_msgs::srv::Vdock>::SharedPtr save_dock_client_; ///< Client for save dock service
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr status_client_; ///< Client for status service

    // std::shared_future<viro_msgs::srv::Vdock::Response::SharedPtr> future_;
    rclcpp::Client<viro_msgs::srv::Vdock>::SharedFuture future_; ///< Future for service requests
    bool done_flag_ = false; ///< Flag to indicate service completion
    // Service suffix to distinguish docking/undocking/status
};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__ACTION__AUTODOCK_ACTION_HPP_
