#include "viro_bt_util/plugins/action/terminalops_action.hpp"
#include <sstream>
#include <stdexcept>
#include <cstring>



namespace viro_bt_util
{

TerminalOps::TerminalOps(const std::string& name, const BT::NodeConfig& conf)
    : BT::StatefulActionNode(name, conf)
{
    // Ensure that you are not calling rclcpp::init() here or elsewhere in your node setup.

    // Fetch the node from the blackboard, which should already have been initialized
    node_ptr_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // Verify the node pointer is valid
    if (!node_ptr_) {
        std::cerr << "Failed to retrieve ROS node from the blackboard!" << std::endl;
        throw std::runtime_error("ROS node is not available.");
    }

    callback_group_ = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_,
        node_ptr_->get_node_base_interface());  
    std::cout << "[" << this->name() << "] terminal bt node joined..." << std::endl;
}

TerminalOps::~TerminalOps()
{
  // print message to screen.
  std::cout << "Destructor called: Cleaning up all processes." << std::endl;
  // Reuse the existing onHalted logic for cleanup
  onHalted();
  RCLCPP_DEBUG(node_ptr_->get_logger(), "Shutting down TerminalOps BT node");
  callback_group_executor_.cancel();
}

BT::PortsList TerminalOps::providedPorts()
{
    return { 
            BT::InputPort<std::string>("task_name"),
            BT::InputPort<std::string>("command")
        };
}

BT::NodeStatus TerminalOps::onStart()
{
    std::cout << "[" << this->name() << "] starting..." << std::endl;

    auto task_name = getInput<std::string>("task_name");
    auto command = getInput<std::string>("command");

    if (!task_name || !command)
    {
        std::cerr << "Missing required input ports (task_name, command)" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[" << this->name() << "] received cmd: " << command.value() << std::endl;
    if (isControlCommand(command.value()))
    {
        auto it = shared_process_map_.find(task_name.value());
        if (it == shared_process_map_.end())
        {
            std::cerr << "Control command received for non-existent task: " << task_name.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (stopProcess(it->second))
        {
            shared_process_map_.erase(it);
            std::cout << "Task stopped successfully: " << task_name.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cerr << "Failed to stop task: " << task_name.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
    else
    {
        if (shared_process_map_.find(task_name.value()) != shared_process_map_.end())
        {
            std::cerr << "Task already running: " << task_name.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        pid_t pid = startProcess(command.value());
        if (pid > 0)
        {
            shared_process_map_[task_name.value()] = pid;
            std::cout << "Task started successfully: " << task_name.value() << " (PID: " << pid << ")" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cerr << "Failed to start task: " << task_name.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }
}




BT::NodeStatus TerminalOps::onRunning()
{
    std::cout << "[" << this->name() << "] onRunning called..." << std::endl;
    // Check the running state of the process (optional implementation)
    return BT::NodeStatus::RUNNING;
}

void TerminalOps::onHalted()
{
    RCLCPP_WARN(node_ptr_->get_logger(), "TerminalOps action halted.");
    std::cout << "Halted: Cleaning up all processes" << std::endl;

    for (const auto& [task_name, pid] : shared_process_map_)
    {
        if (stopProcess(pid))
        {
            std::cout << "Process (PID: " << pid << ") for task '" << task_name << "' terminated successfully during destruction." << std::endl;
        }
        else
        {
            std::cerr << "Failed to terminate process (PID: " << pid << ") for task '" << task_name << "' during destruction." << std::endl;
        }
    }
    // Clear the map after killing all processes
    shared_process_map_.clear();
}


pid_t TerminalOps::startProcess(const std::string& cmd)
{
    std::cout << "[" << this->name() << "] process start triggered..." << std::endl;
    
    pid_t pid = fork(); // creates a new child process inside this current script which is parent.

    if (pid == 0) // Child process
    {
        // Create a new process group for the child process
        setpgid(0, 0);
        execl("/bin/bash", "bash", "-c", cmd.c_str(), nullptr);
        std::cerr << "Failed to execute command: " << cmd << std::endl;
        std::exit(EXIT_FAILURE); // If execl fails
    }
    else if (pid < 0) // Fork failed
    {
        std::cerr << "Fork failed: " << strerror(errno) << std::endl;
    }

  return pid; // Return PID of the child process
}


bool TerminalOps::stopProcess(pid_t pid)
{
    std::cout << "[" << this->name() << "] process stop triggered..." << std::endl;

    // Kill the entire process group
    if (killpg(pid, SIGTERM) == 0)
    {
        std::cout << "Process group (PID: " << pid << ") terminated successfully." << std::endl;
        return true;
    }
    else
    {
        std::cerr << "Failed to terminate process group (PID: " << pid << "): " << strerror(errno) << std::endl;

        // Try to forcefully kill the process group if SIGTERM fails
        if (killpg(pid, SIGKILL) == 0)
        {
            std::cout << "Process group (PID: " << pid << ") forcefully terminated." << std::endl;
            return true;
        }
        else
        {
            std::cerr << "Failed to forcefully terminate process group (PID: " << pid << "): " << strerror(errno) << std::endl;
            return false;
        }
    }
}



bool TerminalOps::isControlCommand(const std::string& cmd) const
{
    return cmd == "cntrl c";
}

} // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::TerminalOps>("TerminalOps");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<viro_bt_util::TerminalOps>("TerminalOps");
}




// // example:
// pid_t startProcess(const std::string &cmd) {
//     pid_t pid = fork();
//     if (pid == -1) {
//         // Fork failed
//         perror("fork");
//         return -1;
//     } else if (pid == 0) {
//         // Child process
//         execl("/bin/bash", "bash", "-c", cmd.c_str(), nullptr);
//         // If execl returns, there was an error
//         perror("execl");
//         exit(EXIT_FAILURE);
//     }
//     // Parent process continues execution
//     return pid;
// }

// void stopProcess(pid_t pid) {
//     if (kill(pid, SIGTERM) == -1) {
//         perror("kill");
//     }
// }
