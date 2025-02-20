// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <string>

#include "viro_bt_util/plugins/decorator/rate_controller.hpp"

namespace viro_bt_util
{

RateController::RateController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false),
  initialized_(false)
{
}

void RateController::initialize()
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
  initialized_ = true;
}

BT::NodeStatus RateController::tick()
{
  if (!initialized_) {
    initialize();
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  // if (!BT::isStatusActive(status())) {
  //   // Reset the starting point since we're starting a new iteration of
  //   // the rate controller (moving from IDLE to RUNNING)
  //   start_ = std::chrono::high_resolution_clock::now();
  //   first_time_ = true;
  // }

  setStatus(BT::NodeStatus::RUNNING);

  // Determine how long its been since we've started this iteration
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  // Now, get that in seconds
  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion
  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::SKIPPED:
      case BT::NodeStatus::RUNNING:
      case BT::NodeStatus::FAILURE:
        return child_state;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

}  // namespace viro_bt_util

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, /* params */)
{
    factory.registerNodeType<viro_bt_util::RateController>("RateController");
}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<viro_bt_util::RateController>("RateController");
}