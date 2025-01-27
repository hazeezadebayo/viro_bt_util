#include "viro_bt_util/plugins/action/autodock_action.hpp"

namespace viro_bt_util
{

// Constructor
AutoDock::AutoDock(const std::string& name, const BT::NodeConfig& conf)
    : BT::StatefulActionNode(name, conf),
      done_flag_(false)
{
    // Fetch the node from the blackboard
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_,
        node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    // Create service clients
    dock_client_ = node_->create_client<viro_msgs::srv::Vdock>("/v_dock/dock", rmw_qos_profile_services_default, callback_group_);
    undock_client_ = node_->create_client<viro_msgs::srv::Vdock>("/v_dock/undock", rmw_qos_profile_services_default, callback_group_);
    save_dock_client_ = node_->create_client<viro_msgs::srv::Vdock>("/v_dock/save", rmw_qos_profile_services_default, callback_group_);
    status_client_ = node_->create_client<std_srvs::srv::Trigger>("/v_dock/status", rmw_qos_profile_services_default, callback_group_);

    std::cout << "[" << this->name() << "] auto dock bt joined node..." << std::endl;
}

// Destructor
AutoDock::~AutoDock()
{
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down AutoDock BT node");
    callback_group_executor_.cancel();
}

// Define the static method to provide ports
BT::PortsList AutoDock::providedPorts()
{
    return {
        BT::InputPort<std::string>("service_name", "dock service name for the robot requesting."),
        BT::InputPort<int>("dock_id", "ID of the dock where the robot should dock.")
    };
}

BT::NodeStatus AutoDock::onStart()
{
    std::cout << "[" << this->name() << "] auto dock bt starting..." << std::endl;

    auto dock_id = getInput<int>("dock_id");
    if (!dock_id)
    {
        throw BT::RuntimeError("Missing required input [dock_id]: ", dock_id.error());
    }

    auto service_name = getInput<std::string>("service_name");
    if (!service_name)
    {
        throw BT::RuntimeError("Missing required input [service_name]: ", service_name.error());
    }

    auto request = std::make_shared<viro_msgs::srv::Vdock::Request>();
    request->dock_id = dock_id.value();

    if (service_name.value() == "dock")
    {
        if (!dock_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Dock service not available.");
            return BT::NodeStatus::FAILURE;
        }
        future_ = dock_client_->async_send_request(request).share(); // Explicit .share()
    }
    else if (service_name.value() == "undock")
    {
        if (!undock_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Undock service not available.");
            return BT::NodeStatus::FAILURE;
        }
        future_ = undock_client_->async_send_request(request).share(); // Explicit .share()
    }
    else if (service_name.value() == "save_dock")
    {
        if (!save_dock_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Save dock service not available.");
            return BT::NodeStatus::FAILURE;
        }
        future_ = save_dock_client_->async_send_request(request).share(); // Explicit .share()
    }
    else
    {
        throw BT::RuntimeError("Invalid service name: ", service_name.value());
    }

    done_flag_ = false;
    RCLCPP_INFO(node_->get_logger(), "Service request sent: %s", service_name.value().c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AutoDock::onRunning()
{
    std::cout << "[" << this->name() << "] onRunning called..." << std::endl;

    if (done_flag_)
    {
        RCLCPP_INFO(node_->get_logger(), "[%s] Service call completed.", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }

    if (future_.valid())
    {

        std::cout << "[" << this->name() << "] will wait for status service..." << std::endl;

        // Check the status of the service call
        if (status_client_->wait_for_service(std::chrono::seconds(1)))
        {
            std::cout << "[" << this->name() << "] status service available..." << std::endl;

            auto status_request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto status_future = status_client_->async_send_request(status_request).share(); // Explicit .share()

            auto status = rclcpp::spin_until_future_complete(node_, status_future, std::chrono::seconds(1));
            if (status == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = status_future.get();
                if (response->success)
                {
                    RCLCPP_INFO(node_->get_logger(), "Status: %s", response->message.c_str());

                    if (response->message == "docked" ||
                        response->message == "undocked" ||
                        response->message.find("saved") != std::string::npos)
                    {
                        RCLCPP_INFO(node_->get_logger(), "Docking operation completed successfully.");
                        done_flag_ = true;
                        return BT::NodeStatus::SUCCESS;
                    }
                }
            }
            else if (status == rclcpp::FutureReturnCode::TIMEOUT)
            {
                RCLCPP_WARN(node_->get_logger(), "Waiting for service response...");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service.");
                return BT::NodeStatus::FAILURE;
            }
        }

    }

    return BT::NodeStatus::RUNNING;
}

void AutoDock::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "AutoDock action halted.");
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::AutoDock>("AutoDock");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::AutoDock>("AutoDock");
}
