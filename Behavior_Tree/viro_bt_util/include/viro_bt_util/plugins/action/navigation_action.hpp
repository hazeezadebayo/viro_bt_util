#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__NAVIGATION_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__NAVIGATION_ACTION_HPP_

#include <string>
#include <memory>
#include <deque>
#include <map>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>

namespace viro_bt_util
{

// Struct to keep location pose data
struct Pose {
  double x, y, theta;
};

// Sets number of locations from list.
class SetLocations : public BT::SyncActionNode
{
  public:
    SetLocations(const std::string& name, const BT::NodeConfig& conf);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Gets location from a queue of locations read from a list.
class GetLocationFromQueue : public BT::SyncActionNode
{
  public:
    GetLocationFromQueue(const std::string& name, const BT::NodeConfig& conf);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

  private:
    std::deque<std::string> location_queue_;
};

// Go to a target location (wraps around `navigate_to_pose` action).
class Navigation : public BT::StatefulActionNode
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    // Method overrides
    Navigation(const std::string& name, const BT::NodeConfig& conf);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {};
    static BT::PortsList providedPorts();


  private:
    // Action client callbacks
    void result_callback(const GoalHandleNav::WrappedResult& result);

    bool done_flag_{false};
    rclcpp_action::ResultCode nav_result_;
    rclcpp::Node::SharedPtr node_ptr_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
};

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__PLUGINS__ACTION__NAVIGATION_ACTION_HPP_
