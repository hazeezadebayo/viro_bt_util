#ifndef VIRO_BT_UTIL__UTIL_HPP_
#define VIRO_BT_UTIL__UTIL_HPP_

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

namespace viro_bt_util
{

/**
 * @brief Load all behavior tree plugins specified from the plugin_packages
 * @param nh Shared pointer to a ROS2 node
 * @param factory Output behavior tree factory
 * @param plugin_packages List of packages that contains BT plugins to be loaded into the factory.
 *                        This will be overwritten by "plugin_packages" ROS node parameter.
 * @param wait_for_server_timeout Timeout (milliseconds) used when detecting the server.
 *                                This will be overwritten by "wait_for_server_timeout" ROS node parameter.
 * @param server_timeout Timeout (milliseconds) when sending a request.
 *                       This will be overwritten by "server_timeout" ROS node parameter.
 */
void load_plugins(const std::shared_ptr<rclcpp::Node> nh,
    BT::BehaviorTreeFactory& factory,
    const std::vector<std::string> plugin_packages={"viro_bt_util"},
    const int wait_for_server_timeout=1000,
    const int server_timeout=1000);

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__UTIL_HPP_