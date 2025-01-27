/*

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" -f

*/

#include <iostream>
#include <filesystem>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "viro_bt_util/demo_bt_server.hpp"
#include "viro_bt_util/utils.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace BT;
using namespace viro_bt_util;
using namespace std::placeholders;

const std::string default_bt_xml_file = ament_index_cpp::get_package_share_directory("viro_bt_util") + "/tree/demo_tree.xml";
const std::string default_location_file = ament_index_cpp::get_package_share_directory("viro_bt_util") + "/tree/sim_locations.yaml";

DemoBtServer::DemoBtServer() : Node("demo_bt_server")
{
    service_ = this->create_service<std_srvs::srv::Empty>(
        "execute_tree",
        std::bind(&DemoBtServer::handle_service, this, _1, _2));

}


// Setup behavior tree
void DemoBtServer::setup_behavior_tree(BT::BehaviorTreeFactory& factory)
{
    auto blackboard = BT::Blackboard::create();

    // Create a ROS 2 node and set it on the blackboard
    // auto node_ = rclcpp::Node::make_shared("bt_ros2_node");
    // blackboard->set<std::shared_ptr<rclcpp::Node>>("node", node_);
    // Set the existing node on the blackboard
    blackboard->set<std::shared_ptr<rclcpp::Node>>("node", shared_from_this());

    // Read the location file and shuffle it
    this->declare_parameter<std::string>("location_file", default_location_file);
    location_file_ = this->get_parameter("location_file").as_string();
    blackboard->set<std::string>("location_file", location_file_);

    // Initialize a goal and set it on the blackboard
    geometry_msgs::msg::PoseStamped initial_goal;
    initial_goal.header.frame_id = "map";
    blackboard->set<geometry_msgs::msg::PoseStamped>("goal", initial_goal);

    // Create the behavior tree using the XML description
    this->declare_parameter("demo_tree_xml", default_bt_xml_file);
    // "/app/ros2_ws/nav2_ign/Behavior_Tree/viro_bt_util/tree/demo_tree.xml");
    // "/home/hazeezadebayo/Desktop/ros2/src/Behavior_Tree/viro_bt_util/tree/demo_tree.xml");
    std::string demo_tree_xml = this->get_parameter("demo_tree_xml").as_string();

    // Load the behavior tree
    demo_tree_ = factory.createTreeFromFile(demo_tree_xml, blackboard); // (demo_tree_xml);

    RCLCPP_INFO(this->get_logger(), "Using location file %s", location_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "%s/%s is ready.", this->get_namespace(), this->get_name());
}

void DemoBtServer::handle_service(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request, executing tree...");

    BT::NodeStatus status = demo_tree_.tickWhileRunning();

    switch (status) {
        case BT::NodeStatus::SUCCESS:
            RCLCPP_INFO(this->get_logger(), "Tree executed successfully!");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Tree execution failed!");
    }
}


int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the BT server node
    auto bt_server_node = std::make_shared<DemoBtServer>();

    // Setup behavior tree into the BT server
    BehaviorTreeFactory factory;
    load_plugins(bt_server_node, factory);
    bt_server_node->setup_behavior_tree(factory);

    // Spin forever
    rclcpp::spin(bt_server_node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
