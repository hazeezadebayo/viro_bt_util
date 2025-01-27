#ifndef VIRO_BT_UTIL__PLUGINS__ACTION__MAP_CHANGE_ACTION_HPP_
#define VIRO_BT_UTIL__PLUGINS__ACTION__MAP_CHANGE_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "behaviortree_cpp/action_node.h"

namespace viro_bt_util
{

class MapChange : public BT::SyncActionNode
{
public:
  // Constructor
  MapChange(const std::string & name, const BT::NodeConfig & conf);

  // Destructor
  ~MapChange() override;

  // Static method to define the ports
  static BT::PortsList providedPorts();

  // Override the virtual function tick()
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr change_maps_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_global_srv_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_local_srv_;

  void changeMap(const std::string & map_filepath);
  void clearGlobalCostmap();
  void clearLocalCostmap();
};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__ACTION__MAP_CHANGE_ACTION_HPP_
