#include "viro_bt_util/plugins/condition/is_robot_docked_condition.hpp"
#include <cmath>

namespace viro_bt_util
{

IsRobotDocked::IsRobotDocked(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  pose_tolerance_(0.25)
{
}

IsRobotDocked::~IsRobotDocked()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsRobotDocked BT node");
  callback_group_executor_.cancel();
}

void IsRobotDocked::initialize()
{
  getInput("pose_tolerance", pose_tolerance_);
  getInput("localization", localization_);

  if (pose_tolerance_ < 0) {
    RCLCPP_WARN(node_->get_logger(), "Pose tolerance is negative, setting to default (0.25)");
    pose_tolerance_ = 0.25;
  }

  // Get the Node from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(
    callback_group_,
    node_->get_node_base_interface());

  // Create appropriate subscription based on localization method
  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  if (localization_ == "odom")
  {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsRobotDocked::odomCallback, this, std::placeholders::_1),
      sub_option);
  }
  else if (localization_ == "amcl")
  {
    amcl_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsRobotDocked::amclCallback, this, std::placeholders::_1),
      sub_option);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported localization method: '%s'", localization_.c_str());
    throw std::runtime_error("Unsupported localization method");
  }


  // Load station positions from the YAML file
  std::string yaml_file;
  getInput("stations_yaml", yaml_file);

  try {
    YAML::Node config = YAML::LoadFile(yaml_file);

    if (!config["stations"]) {
      RCLCPP_ERROR(node_->get_logger(), "Missing 'stations' key in YAML file.");
      throw std::runtime_error("Missing 'stations' key in YAML file");
    }

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
  } catch (const YAML::BadFile& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", e.what());
    throw std::runtime_error("Failed to load YAML file");
  }
}

void IsRobotDocked::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

void IsRobotDocked::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
}

BT::NodeStatus IsRobotDocked::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();

  // Check if the robot is docked
  if (isRobotDocked()) {
    return BT::NodeStatus::SUCCESS;
  }

  // Set dock_id to 0 if no dock is found
  setOutput("est_dock_id", 0);
  return BT::NodeStatus::FAILURE;
}

bool IsRobotDocked::isRobotDocked()
{
  int dock_id;
  if (checkPose(current_pose_, pose_tolerance_, dock_id)) {
    setOutput("est_dock_id", dock_id);
    return true;
  }
  return false;
}

bool IsRobotDocked::checkPose(const geometry_msgs::msg::Pose& pose, double tolerance, int& dock_id)
{
  for (const auto& entry : station_positions_) {
    const auto& station_pose = entry.second;

    // Calculate the distance between the robot's pose and the station position
    double dx = pose.position.x - station_pose.position.x;
    double dy = pose.position.y - station_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Check if the distance is within tolerance
    if (distance <= tolerance) {
      dock_id = entry.first;  // Set the est_dock_id to the ID of the closest station
      return true;
    }
  }
  return false;
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
  factory.registerNodeType<viro_bt_util::IsRobotDocked>("IsRobotDocked");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::IsRobotDocked>("IsRobotDocked");
}
