#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__TELEOP_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__TELEOP_ACTION_HPP_

#include <string>
#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <behaviortree_cpp/action_node.h>

namespace viro_bt_util
{

class TeleOp : public BT::StatefulActionNode
{
public:
  TeleOp(const std::string& name, const BT::NodeConfig& conf);

  ~TeleOp() override;

  static BT::PortsList providedPorts();

protected:
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override; // Override the halt method since its async

private:
  void cmdCallback(const std::shared_ptr<std_msgs::msg::String> msg);
  void move_robot(char move_key, uint32_t time);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::map<char, std::vector<float>> moveBindings;
  std::map<char, std::vector<float>> speedBindings;

  double speed_;
  double turn_;
  double x_, y_, z_, th_;
  bool stop_received;

  std::string keyboard_drive_key_;
  std::string manual_drive_topic_;
};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__ACTION__TELEOP_ACTION_HPP_
