#include "viro_bt_util/plugins/action/initpose_action.hpp"

namespace viro_bt_util
{

InitPose::InitPose(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf),
  initial_pose_received_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  init_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::cout << "[" << this->name() << "] initial pose bt joined node..." << std::endl;

}

InitPose::~InitPose()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down InitPose BT node");
  callback_group_executor_.cancel();
}

BT::PortsList InitPose::providedPorts()
{
  return { BT::InputPort<std::string>("initial_pose") };
}

BT::NodeStatus InitPose::tick()
{
  std::cout << "[" << this->name() << "] initial pose bt ticked..." << std::endl;

  BT::Expected<std::string> initial_pose_str = getInput<std::string>("initial_pose");

  if (!initial_pose_str)
  {
    throw BT::RuntimeError("missing required input [initial_pose]: ", initial_pose_str.error());
  }

  std::string initial_pose = initial_pose_str.value();

  if (!initial_pose.empty())
  {
    std::cout << "[" << this->name() << "] provided initial pose will be used..." << std::endl;

    // Parse the initial pose string
    std::vector<std::string> tokens;
    std::istringstream token_stream(initial_pose);
    std::string token;
    while (std::getline(token_stream, token, ';'))
    {
      tokens.push_back(token);
    }

    if (tokens.size() != 4)
    {
      throw BT::RuntimeError("invalid initial_pose format, expected 4 values separated by ';'");
    }

    double x = std::stod(tokens[0]);
    double y = std::stod(tokens[1]);
    double z = std::stod(tokens[2]);
    double w = std::stod(tokens[3]);

    publishInitialPose(x, y, z, w);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::cout << "[" << this->name() << "] initial pose not provided odom will be used..." << std::endl;

    // Subscribe to odom to get the pose
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SystemDefaultsQoS(),
        std::bind(&InitPose::odomCallback, this, std::placeholders::_1),
        sub_option);
    std::cout << "[" << this->name() << "] initial pose bt sub'd /odom topic..." << std::endl;

    while (!initial_pose_received_)
    {
      callback_group_executor_.spin_some();
      std::cout << "[" << this->name() << "] initial pose bt spinned some..." << std::endl;

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    return BT::NodeStatus::SUCCESS;
  }
}

void InitPose::publishInitialPose(double x, double y, double z, double w)
{
  initial_pose_.header.frame_id = "map";
  initial_pose_.header.stamp = node_->now();
  initial_pose_.pose.pose.position.x = x;
  initial_pose_.pose.pose.position.y = y;
  initial_pose_.pose.pose.orientation.z = z;
  initial_pose_.pose.pose.orientation.w = w;
  initial_pose_.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};

  init_pose_pub_->publish(initial_pose_);

  callback_group_executor_.spin_some();
  std::cout << "[" << this->name() << "] initial pose bt spinned some..." << std::endl;

  RCLCPP_INFO(node_->get_logger(), "Published initial pose: x = %f, y = %f, z = %f, w = %f", x, y, z, w);
}

void InitPose::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::cout << "[" << this->name() << "] initial pose bt odom msg received..." << std::endl;
  const auto & pose = msg->pose.pose;
  publishInitialPose(pose.position.x, pose.position.y, pose.orientation.z, pose.orientation.w);
  initial_pose_received_ = true;
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::InitPose>("InitPose");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::InitPose>("InitPose");
}
