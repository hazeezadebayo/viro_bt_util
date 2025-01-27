#ifndef VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_DOCK_CONDITION_HPP_
#define VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_DOCK_CONDITION_HPP_

#include <string>
#include <memory>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/condition_node.h>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h> // For quaternion math
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For tf2 to geometry_msgs conversions
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace viro_bt_util
{

class IsGoalDockCondition : public BT::ConditionNode
{
public:
  IsGoalDockCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalDockCondition() = delete;

  ~IsGoalDockCondition() override;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_file"),
      BT::OutputPort<int>("goal_dock_id")
    };
  }

protected:
  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Checks if the goal matches any station position from the YAML file
   * @return bool true when a match is found, false otherwise
   */
  bool checkGoalDock(const geometry_msgs::msg::PoseStamped& goal);

private:
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<int, geometry_msgs::msg::Pose> station_positions_;  // Stores station positions

};

}  // namespace viro_bt_util

#endif  // VIRO_BT_UTIL__PLUGINS__CONDITION__IS_GOAL_DOCK_CONDITION_HPP_
