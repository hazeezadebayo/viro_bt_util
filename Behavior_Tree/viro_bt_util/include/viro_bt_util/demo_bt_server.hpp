#ifndef VIRO_BT_UTIL__DEMO_BT_SERVER_HPP_
#define VIRO_BT_UTIL__DEMO_BT_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

namespace viro_bt_util
{

class DemoBtServer : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     */
    explicit DemoBtServer();

    /**
     * @brief Call this function to setup behavior tree. Note that this
     *        function should only be called once to prevent errors.
     * @param factory Behavior tree factory
     */
    void setup_behavior_tree(BT::BehaviorTreeFactory& factory);

private:
    // -----Variables-----
    BT::Tree demo_tree_;

    // Publishers, subscribers, service servers, action servers
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> service_;

    // Functions
    void handle_service(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);

    std::string location_file_;

};

} // namespace viro_bt_util

#endif // VIRO_BT_UTIL__DEMO_BT_SERVER_HPP_