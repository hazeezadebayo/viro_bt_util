#include "viro_bt_util/plugins/action/navigation_action.hpp"

#include <random>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace viro_bt_util
{

// SETLOCATIONS
// Gets a list of locations from a YAML file and ensures they are not empty
SetLocations::SetLocations(const std::string& name, const BT::NodeConfig& conf) :
    BT::SyncActionNode(name, conf)
{
    std::cout << "[" << this->name() << "] Starting . . ." << std::endl;
}

BT::NodeStatus SetLocations::tick() {

    std::cout << "[" << this->name() << "] set locations ticked..." << std::endl;

    std::string location_file;
    const auto result = config().blackboard->get("location_file", location_file);
    if (!result) {
        std::cerr << "[" << this->name() << "] Could not read locations file from blackboard." << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        // Load the YAML file
        YAML::Node locations = YAML::LoadFile(location_file);

        // Extract number of locations
        int num_locs = locations.size();
        if (num_locs == 0) {
            std::cerr << "[" << this->name() << "] No locations found." << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::deque<std::string> location_names;
        std::map<std::string, Pose> location_poses;

        // Parse each location from the YAML file
        for (YAML::const_iterator it = locations.begin(); it != locations.end(); ++it) {
            const std::string name = it->first.as<std::string>();
            const std::vector<double> values = it->second.as<std::vector<double>>();

            // Validate the size of the vector
            if (values.size() != 3) {
                std::cerr << "[" << this->name() << "] Invalid pose for location: " << name << std::endl;
                return BT::NodeStatus::FAILURE;
            }

            location_names.push_back(name);
            location_poses[name] = Pose{values[0], values[1], values[2]};
        }

        // Shuffle the location names
        std::random_device rd;
        std::mt19937 rng(rd());
        std::shuffle(location_names.begin(), location_names.end(), rng);

        // Set outputs for use elsewhere
        setOutput("loc_names", location_names);
        setOutput("loc_poses", location_poses);
        setOutput("num_locs", num_locs);

        std::cout << "[" << this->name() << "] Found " << num_locs << " locations." << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "[" << this->name() << "] Couldn't load locations file: " << location_file
                    << ". Error: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetLocations::providedPorts()
{
    return { BT::OutputPort<int>("num_locs"),
             BT::OutputPort<std::deque<std::string>>("loc_names"),
             BT::OutputPort<std::map<std::string, Pose>>("loc_poses")
         };
}


// GETLOCATIONFROMQUEUE
// Gets a location name from a queue of locations to visit.
// If the queue is empty, this behavior fails.
GetLocationFromQueue::GetLocationFromQueue(const std::string& name,
                                           const BT::NodeConfig& conf) :
    BT::SyncActionNode(name, conf)
{
    std::cout << "[" << this->name() << "] Initialized..." << std::endl;
}

BT::NodeStatus GetLocationFromQueue::tick()
{
    // Get the locations from the port and select first one as the next target
    auto location_queue_ = getInput<std::deque<std::string>>("loc_names");
    if (!location_queue_) {
        std::cerr << "Couldn't get loc_names!" << std::endl;
    }
    if (location_queue_.value().empty()) {
        std::cout << "[" << this->name() << "] No more locations!" << std::endl;
        return BT::NodeStatus::FAILURE;
    } else {
        std::string tgt_loc = location_queue_.value().front();
        setOutput("target_location", tgt_loc);
        location_queue_.value().pop_front();
        std::cout << "[" << this->name() << "] Targeting location: " << tgt_loc << std::endl;
        setOutput("loc_names", location_queue_.value());
        return BT::NodeStatus::SUCCESS;
    }
}

BT::PortsList GetLocationFromQueue::providedPorts()
{
    return { BT::OutputPort<std::string>("target_location"),
             BT::BidirectionalPort<std::deque<std::string>>("loc_names") };
}

// Navigation (GO_TO_POSE)
// Wrapper behavior around the `navigate_to_pose` action client,
// whose status reflects the status of the ROS action.
Navigation::Navigation(const std::string& name, const BT::NodeConfig& conf) :
    BT::StatefulActionNode(name, conf)
{
    // Fetch the node from the blackboard
    node_ptr_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::cout << "[" << this->name() << "] nav bt node joined..." << std::endl;
}

BT::NodeStatus Navigation::onStart() {
    // Validate that a node exists
    if (!node_ptr_) {
        std::cout << "ROS2 node not registered via init() method" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Read the YAML file
    BT::Expected<std::string> loc = getInput<std::string>("loc");

    auto location_poses = getInput<std::map<std::string,Pose>>("loc_poses");
    if (!location_poses){
        std::cerr << "Couldn't get loc_poses!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_loc = getInput<std::string>("loc");
    if (!target_loc) {
        std::cerr << "Couldn't get target loc!" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    auto target_pose = location_poses.value().at(target_loc.value());

    // Set up the action client
    using namespace std::placeholders;
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&Navigation::result_callback, this, _1);
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_ptr_, "/navigate_to_pose");
    std::cout << "[" << this->name() << "] creating nav2 client..." << std::endl;

    // Package up the the goal
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = target_pose.x;
    goal_msg.pose.pose.position.y = target_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_pose.theta);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Send the navigation action goal.
    done_flag_ = false;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "[" << this->name() << "] Sent goal message" << std::endl;

    // goal: set it on the blackboard
    // config().blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal_msg.pose);
    config().blackboard->set("goal", goal_msg.pose);  // Store the goal pose in the blackboard

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Navigation::onRunning() {
    // If there is a result, we can check the status of the action directly.
    // Otherwise, the action is still running.
    std::cout << "[" << this->name() << "] checking done flag..." << std::endl;

    // test purpose only:
    // return BT::NodeStatus::SUCCESS;

    if (done_flag_) {
        if (nav_result_ == rclcpp_action::ResultCode::SUCCEEDED) {
            std::cout << "[" << this->name() << "] Goal reached" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    } else {
        return BT::NodeStatus::RUNNING;
    }
}

BT::PortsList Navigation::providedPorts() {
    return { BT::InputPort<std::string>("loc"),
             BT::InputPort<std::map<std::string, Pose>>("loc_poses") };
}

void Navigation::result_callback(const GoalHandleNav::WrappedResult& result) {
    // If there is a result, we consider navigation completed and save the
    // result code to be checked in the `onRunning()` method.
    std::cout << "[" << this->name() << "] evaluating navigation feedback..." << std::endl;

    if (result.result) {
        done_flag_ = true;
        nav_result_ = result.code;
    }
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::SetLocations>("SetLocations");
    factory.registerNodeType<viro_bt_util::GetLocationFromQueue>("GetLocationFromQueue");
    factory.registerNodeType<viro_bt_util::Navigation>("Navigation");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::SetLocations>("SetLocations");
    factory.registerNodeType<viro_bt_util::GetLocationFromQueue>("GetLocationFromQueue");
    factory.registerNodeType<viro_bt_util::Navigation>("Navigation");
}
