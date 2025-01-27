#include "viro_bt_util/plugins/action/mapchange_action.hpp"

namespace viro_bt_util
{

MapChange::MapChange(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  change_maps_srv_ = node_->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
  clear_costmap_global_srv_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");
  clear_costmap_local_srv_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");

  std::cout << "[" << this->name() << "] map bt joined node..." << std::endl;

}

MapChange::~MapChange()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down MapChange BT node");
}

BT::PortsList MapChange::providedPorts()
{
  return { BT::InputPort<std::string>("map_filepath") };
}

BT::NodeStatus MapChange::tick()
{
  std::cout << "[" << this->name() << "] map bt ticked..." << std::endl;

  BT::Expected<std::string> map_filepath = getInput<std::string>("map_filepath");

  if (!map_filepath)
  {
    throw BT::RuntimeError("missing required input [map_filepath]: ", map_filepath.error());
  }

  if (!change_maps_srv_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Service change_maps not available.");
    // throw BT::RuntimeError("Service change_maps not available");
    return BT::NodeStatus::FAILURE;
  }
  
  changeMap(map_filepath.value());

  if (!clear_costmap_global_srv_->wait_for_service(std::chrono::seconds(1)) ||
      !clear_costmap_local_srv_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Costmap clear services not available.");
    return BT::NodeStatus::FAILURE;
  }

  clearGlobalCostmap();
  clearLocalCostmap();
  return BT::NodeStatus::SUCCESS;
}


void MapChange::changeMap(const std::string & map_filepath)
{
  std::cout << "[" << this->name() << "] map bt changeMap..." << std::endl;

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_filepath;

  change_maps_srv_->async_send_request(request, 
      [this](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture response)
      {
        if (response.get()->result != nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS)
        {
          RCLCPP_ERROR(this->node_->get_logger(), "Change map request failed!");
          // Since we cannot directly throw in this lambda, we'll set a flag or handle it accordingly
        }
        else
        {
          RCLCPP_INFO(this->node_->get_logger(), "Change map request was successful!");
        }
      });

  std::cout << "[" << this->name() << "] debug 1..." << std::endl;
}

void MapChange::clearGlobalCostmap()
{
  std::cout << "[" << this->name() << "] map bt clearGlobalCostmap..." << std::endl;

  if (!clear_costmap_global_srv_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Service clear_costmap_global not available.");
    throw BT::RuntimeError("Service clear_costmap_global not available");
  }

  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  clear_costmap_global_srv_->async_send_request(request, 
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture)
      {
        RCLCPP_INFO(this->node_->get_logger(), "Global costmap cleared successfully!");
      });

  std::cout << "[" << this->name() << "] debug 2..." << std::endl;
}

void MapChange::clearLocalCostmap()
{
  std::cout << "[" << this->name() << "] map bt clearLocalCostmap..." << std::endl;

  if (!clear_costmap_local_srv_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Service clear_costmap_local not available.");
    throw BT::RuntimeError("Service clear_costmap_local not available");
  }

  auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

  clear_costmap_local_srv_->async_send_request(request, 
      [this](rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedFuture)
      {
        RCLCPP_INFO(this->node_->get_logger(), "Local costmap cleared successfully!");
      });

  std::cout << "[" << this->name() << "] debug 3..." << std::endl;
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
  factory.registerNodeType<viro_bt_util::MapChange>("MapChange");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::MapChange>("MapChange");
}


