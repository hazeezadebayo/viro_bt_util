#include "viro_bt_util/plugins/action/teleop_action.hpp"
#include <unistd.h>
#include <std_msgs/msg/string.hpp>
#include <iostream>

namespace viro_bt_util
{

// Initialize static member variables
const std::map<char, std::vector<float>> moveBindings = {
    {'i', {1, 0, 0, 0}}, {'o', {1, 0, 0, -1}}, {'j', {0, 0, 0, 1}}, {'l', {0, 0, 0, -1}},
    {'u', {1, 0, 0, 1}}, {',', {-1, 0, 0, 0}}, {'.', {-1, 0, 0, 1}}, {'m', {-1, 0, 0, -1}},
    {'O', {1, -1, 0, 0}}, {'I', {1, 0, 0, 0}}, {'J', {0, 1, 0, 0}}, {'L', {0, -1, 0, 0}},
    {'U', {1, 1, 0, 0}}, {'<', {-1, 0, 0, 0}}, {'>', {-1, -1, 0, 0}}, {'M', {-1, 1, 0, 0}},
    {'t', {0, 0, 1, 0}}, {'b', {0, 0, -1, 0}}, {'k', {0, 0, 0, 0}}, {'K', {0, 0, 0, 0}}};

const std::map<char, std::vector<float>> speedBindings = {
    {'q', {1.1, 1.1}}, {'z', {0.9, 0.9}}, {'w', {1.1, 1}}, {'x', {0.9, 1}},
    {'e', {1, 1.1}}, {'c', {1, 0.9}}};

// Constructor
TeleOp::TeleOp(const std::string& name, const BT::NodeConfig& conf)
    : BT::StatefulActionNode(name, conf),
    speed_(0.5),
    turn_(1.0),
    x_(0), y_(0), z_(0), th_(0),
    stop_received(true)
{
  // Fetch the node from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
        callback_group_,
        node_->get_node_base_interface());

  // Initialize publisher
  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  std::cout << "[" << this->name() << "] TeleOp bt joined node..." << std::endl;
}

TeleOp::~TeleOp()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down TeleOp BT node");
  callback_group_executor_.cancel();
}

// Define the static method to provide ports
BT::PortsList TeleOp::providedPorts()
{
  return {
    BT::InputPort<std::string>("keyboard_drive_key"),
    BT::InputPort<std::string>("manual_drive_topic")
  };
}

// Override the tick() method
BT::NodeStatus TeleOp::onStart()
{
  std::cout << "TeleOp starting..." << std::endl;

  std::string keyboard_drive_key, manual_drive_topic;

  // Get the input keyboard drive key and manual drive topic
  BT::Expected<std::string> input_key = getInput<std::string>("keyboard_drive_key");
  BT::Expected<std::string> input_topic = getInput<std::string>("manual_drive_topic");

  if (input_key && !input_key.value().empty())
  {
    std::cout << "keyboard key received..." << std::endl;
    keyboard_drive_key = input_key.value();
    std::cout << "keys..." << keyboard_drive_key << std::endl;

    auto parts = BT::splitString(keyboard_drive_key, ';');
    char move_key = parts[0][0];
    double time_input = BT::convertFromString<double>(parts[1]);
    uint32_t time = static_cast<uint32_t>(time_input * 1e6);

    move_robot(move_key, time);
    return BT::NodeStatus::SUCCESS;
  }
  else if (input_topic && !input_topic.value().empty())
  {
    std::cout << "manual sub topic received..." << std::endl;
    manual_drive_topic = input_topic.value();
    std::cout << "topic..." << manual_drive_topic << std::endl;

    // Initialize subscriber for manual drive topic if not already initialized
    if (!sub_)
    {
      rclcpp::SubscriptionOptions sub_option;
      sub_option.callback_group = callback_group_;
      sub_ = node_->create_subscription<std_msgs::msg::String>(
          manual_drive_topic,
          rclcpp::SystemDefaultsQoS(),
          std::bind(&TeleOp::cmdCallback, this, std::placeholders::_1),
          sub_option);
      std::cout << "subscribed..." << std::endl;
    }

    callback_group_executor_.spin_some();
    RCLCPP_DEBUG(node_->get_logger(), "callback spin some...");

    stop_received = false;
    return BT::NodeStatus::RUNNING;

  }
  else
  {
    throw BT::RuntimeError("Missing required input [keyboard_drive_key or manual_drive_topic]");
  }
}


BT::NodeStatus TeleOp::onRunning()
{
  std::cout << "TeleOp onRunning called..." << std::endl;

  if (stop_received)
  {
    stop_received = false;
    RCLCPP_INFO(node_->get_logger(), "[%s] Service call completed.", this->name().c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}


void TeleOp::cmdCallback(const std::shared_ptr<std_msgs::msg::String> msg)
{
  std::cout << "cmd callback message..." << std::endl;

  std::string movement = msg->data;

  auto parts = BT::splitString(movement, ';');
  char move_key = parts[0][0];
  double time_input = BT::convertFromString<double>(parts[1]);
  uint32_t time = static_cast<uint32_t>(time_input * 1e6);

  move_robot(move_key, time);

  if (move_key == 'k' || move_key == 'K')
  {
    // Stop command received
    stop_received = true;
    // throw BT::RuntimeError("Stop command received");
  }
}

void TeleOp::move_robot(char move_key, uint32_t time)
{
  std::cout << "move robot request..." << std::endl;

  // Handle movement keys
  if (moveBindings.count(move_key) == 1)
  {
    x_ = moveBindings.at(move_key)[0];
    y_ = moveBindings.at(move_key)[1];
    z_ = moveBindings.at(move_key)[2];
    th_ = moveBindings.at(move_key)[3];
  }
  else if (speedBindings.count(move_key) == 1)
  {
    speed_ *= speedBindings.at(move_key)[0];
    turn_ *= speedBindings.at(move_key)[1];
  }
  else
  {
    x_ = 0;
    y_ = 0;
    z_ = 0;
    th_ = 0;
  }

  // Update the Twist message
  geometry_msgs::msg::Twist twist;
  twist.linear.x = x_ * speed_;
  twist.linear.y = y_ * speed_;
  twist.linear.z = z_ * speed_;
  twist.angular.z = th_ * turn_;

  // Publish and spin
  pub_->publish(twist);
  callback_group_executor_.spin_some();
  usleep(time);
  std::cout << "command published..." << std::endl;
}

void TeleOp::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "TeleOp action halted.");
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::TeleOp>("TeleOp");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::TeleOp>("TeleOp");
}
