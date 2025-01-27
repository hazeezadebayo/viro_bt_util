#include "viro_bt_util/plugins/action/interrupt_action.hpp"

namespace viro_bt_util
{

// Constructor definition
Interrupt::Interrupt(const std::string& name, const BT::NodeConfig& conf)
    : BT::SyncActionNode(name, conf),
    interrupt_("")
{
    // node_ = rclcpp::Node::make_shared("interrupt_event_node");
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        "interrupt_event",
        10,
        [this](const std_msgs::msg::String::SharedPtr msg){this->interruptCallback(msg);},
        sub_option);

    std::cout << "[" << this->name() << "] interrupt bt node joined..." << std::endl;

}

// Define the static method to provide ports
BT::PortsList Interrupt::providedPorts()
{
    return { BT::InputPort<std::string>("event") };
}

Interrupt::~Interrupt()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down Interrupt BT node");
  callback_group_executor_.cancel();
}

// Callback function definition
void Interrupt::interruptCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(), "Interrupt callback detected event: %s", msg->data.c_str());
    interrupt_ = msg->data;
    std::cout << "[" << this->name() << "] got event: " << msg->data.c_str() << std::endl;
}

// Override the tick() method
BT::NodeStatus Interrupt::tick()
{
    std::cout << "[" << this->name() << "] interrupt bt ticked..." << std::endl;

    BT::Expected<std::string> expected_event = getInput<std::string>("event");

    if (!expected_event)
    {
        throw BT::RuntimeError("missing required input [event]: ", expected_event.error());
    }

    interrupt_.clear();

    callback_group_executor_.spin_some();
    std::cout << "[" << this->name() << "] interrupt bt spinned some... expecting event: " << expected_event.value() << std::endl;

    if (interrupt_ == expected_event.value())
    {
        std::cout << "[" << this->name() << "] interrupt confirmed..." << std::endl;
        
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::Interrupt>("Interrupt");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::Interrupt>("Interrupt");
}
