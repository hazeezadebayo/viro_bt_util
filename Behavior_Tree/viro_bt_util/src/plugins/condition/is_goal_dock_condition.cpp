#include "viro_bt_util/plugins/condition/is_goal_dock_condition.hpp"
#include <cmath>
#include <fstream>  // Add this line to include the fstream library


namespace viro_bt_util
{

IsGoalDockCondition::IsGoalDockCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  initialize();  // Ensure stations are loaded from the YAML file
}

IsGoalDockCondition::~IsGoalDockCondition()
{
  // Proper destructor implementation
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsGoalDockCondition BT node");
}

void IsGoalDockCondition::initialize()
{
  // Get the Node from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  std::cout << "[" << this->name() << "] is goal dock bt joined node..." << std::endl;  

  // Get the YAML file from input port
  std::string yaml_file;
  getInput("station_file", yaml_file);

  // Check if the file path is valid and the file exists
  if (yaml_file.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "YAML file path is empty");
    throw std::runtime_error("YAML file path is empty");
  }

  std::ifstream file(yaml_file);
  if (!file.good()) {
    RCLCPP_ERROR(node_->get_logger(), "YAML file does not exist: %s", yaml_file.c_str());
    throw std::runtime_error("YAML file does not exist");
  }


  try {

    // Load the YAML file and populate the station positions map
    YAML::Node config = YAML::LoadFile(yaml_file);

    for (const auto& station : config["stations"]) {
      int id = station["id"].as<int>();
      geometry_msgs::msg::Pose pose;
      pose.position.x = station["position"]["x"].as<double>();
      pose.position.y = station["position"]["y"].as<double>();
      pose.position.z = station["position"]["z"].as<double>();
      pose.orientation.w = station["orientation"]["w"].as<double>();
      pose.orientation.x = station["orientation"]["x"].as<double>();
      pose.orientation.y = station["orientation"]["y"].as<double>();
      pose.orientation.z = station["orientation"]["z"].as<double>();
      station_positions_[id] = pose;
    }

    // Process YAML file
  } catch (const YAML::BadFile& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", e.what());
    throw std::runtime_error("Failed to load YAML file");
  }

}

BT::NodeStatus IsGoalDockCondition::tick()
{
  std::cout << "[" << this->name() << "] is goal dock bt ticked..." << std::endl;

  // Retrieve the goal from the blackboard
  geometry_msgs::msg::PoseStamped goal;
  if (!config().blackboard->get("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to retrieve goal from blackboard");
    return BT::NodeStatus::FAILURE; // Exit if the goal is not found
  }

  // Check if the goal matches any station position
  if (checkGoalDock(goal)) {
    std::cout << "[" << this->name() << "] is goal dock status confirmed..." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // we didnt find its corresponding dock id. does not exist!
  setOutput("goal_dock_id", 0);
  return BT::NodeStatus::FAILURE;
}

bool IsGoalDockCondition::checkGoalDock(const geometry_msgs::msg::PoseStamped& goal)
{
  std::cout << "[" << this->name() << "] evaluating is goal dock status..." << std::endl;

  for (const auto& entry : station_positions_) {
    const auto& station_pose = entry.second;

    // Calculate the distance between the goal and the station position
    double dx = goal.pose.position.x - station_pose.position.x;
    double dy = goal.pose.position.y - station_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Consider the position within tolerance if the distance is less than a threshold
    double tolerance = 0.5;  // Example tolerance value, can be adjusted
    if (distance <= tolerance) {
      RCLCPP_INFO(node_->get_logger(), "Goal matches station ID: %d", entry.first);
      // Set the goal dock ID to the output port
      setOutput("goal_dock_id", entry.first);
      return true;
    }
  }

  return false;
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
  factory.registerNodeType<viro_bt_util::IsGoalDockCondition>("IsGoalDock");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::IsGoalDockCondition>("IsGoalDock");
}
