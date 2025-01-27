# BehaviorTree.ROS2
[![Test](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml/badge.svg)](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml)

This repository contains useful wrappers to use ROS2 and BehaviorTree.CPP together.

In particular, it provides a standard way to implement:

- Behavior Tree Executor with ROS Action interface.
- Action clients.
- Service Clients.
- Topic Subscribers.
- Topic Publishers.

Our main goals are:

- to minimize the amount of boilerplate.
- to make asynchronous Actions non-blocking.

# Documentation

- [ROS Behavior Wrappers](behaviortree_ros2/ros_behavior_wrappers.md)
- [TreeExecutionServer](behaviortree_ros2/tree_execution_server.md)
- [Sample Behaviors](btcpp_ros2_samples/README.md)

Note that this library is compatible **only** with:

- **BT.CPP** 4.6 or newer.
- **ROS2** Humble or newer.

Additionally, check **plugins.hpp** to see how to learn how to
wrap your Nodes into plugins that can be loaded at run-time.


## Acknowledgements

A lot of code is either inspired or copied from [Nav2](https://docs.nav2.org/).

For this reason, we retain the same license and copyright.


## Usage

$ sudo apt install ros-humble-behaviortree-cpp
$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup




# ----------------------------------------------------------------------- #
#               AGV CLIENT DatabaseRegistration                           #
# ----------------------------------------------------------------------- #


# viro_bt_util
ROS2 utility library for Behaviour Tree C++.

Most of the source code here are taken and modified from ros-navigation [navigation2/nav2_behaviour_tree repository](https://github.com/ros-navigation/navigation2/tree/main/nav2_behavior_tree).

## Demo:

To run demo behavior tree server:
## Terminal 1:
```bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 run viro_bt_util demo_bt_server
```

On another terminal, execute the tree via ros2 service:
# Terminal 2:
```bash
docker exec -it dev3-tailscaled-1 bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source /opt/ros/humble/setup.bash; source install/setup.bash; ros2 service call /execute_tree std_srvs/srv/Empty
```

## Author
- [@Hazeezadebayo](https://github.com/Hazeezadebayo)

## NOTE:
- Please change the special messages like diffamr blah blah blah in emergency messages
- or viro_v_dock messages in autodock etc.
- or odom topic name and the likes.


# ----------------------------------------------------------------------- #
#
#        IF, START NAVIGATION:
#                ----L----
#               0-   x   -0
#                |       |
#                |       |
#               0-       -0
#                ---------
# ----------------------------------------------------------------------- #

# USAGE:

# Terminal 1:
cd docker_ws/env/dev3
export DISPLAY=:0.0
xhost +local:docker
docker compose up --build
# or
docker run -it dev3-tailscaled /bin/bash

# -------------- NO MQTT?
# Terminal 2:
docker exec -it dev3-tailscaled-1 bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch viro_simple_fleet fleet_client.launch.py

# Terminal 3:
docker exec -it dev3-tailscaled-1 bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; python3 FleetManager/viro_simple_fleet/scripts/fleet_mngr_main.py

# -------------- MQTT?
# Terminal 2:
docker exec -it dev3-tailscaled-1 bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash; export TURTLEBOT3_MODEL=waffle; ros2 launch vda5050_tb3_adapter connector_tb3.launch.py

# Terminal 3:
docker exec -it dev3-tailscaled-1 bash
cd ros2_ws/nav2_ign && . /usr/share/gazebo/setup.sh; source install/setup.bash;
ros2 topic echo /kullar/v1/OSRF/TB3_1/state






# EXAMPLE TREE 1:

    <Sequence name="main_loop">

        <SetLocations name="set_locations" num_locs="{num_locs}" loc_names="{loc_names}" loc_poses="{loc_poses}"/>
        <!-- since failure is returned when no locations to visit -->
        <KeepRunningUntilFailure>

          <!-- nav sequence handles/visits one goal at a time. -->
          <!-- visit all locations until there is none. the nav sequence returns failure if no more goals in list. -->
          <Sequence                   name="search_location">
              <GetLocationFromQueue   name="get_loc"      loc_names="{loc_names}" target_location="{target_location}"/>
              <Navigation             name="go_to_loc"    loc_poses="{loc_poses}" loc="{target_location}"/>

              <!-- Invert the choice of our fallback, here if any fallback child is a success, then we negate it. -->
              <!-- this can help kill our while-like KeepRunningUntilFailure loop. else loop keeps looping. right? -->
              <Inverter>

                <!-- fallback returns failure if all children returns failure. and success if any returns success. -->
                <!-- will check next child only if the current child returns failure. -->
                <Fallback>

                  <!-- Check if the battery is low, we need to cancel current motion and go to charge -->
                  <IsBatteryLow min_battery="20.0" battery_topic="/battery_status" is_voltage="false"/>

                  <!-- Check if there is an emergency -->
                  <!-- <IsEmergency emergency_topic="/emergency_status"/> -->

                  <!-- Check if task is interrupted from gui -->
                  <Interrupt event="cancel"/>

                  <!-- Check if there is an obstacle -->
                  <isObstacle brake_accel_limit="-10.0" use_scan="true" warning_threshold="0.5"/>

                <!-- End of Fallback -->
                </Fallback>

              <!-- End of Invertion -->
              </Inverter>

          </Sequence>
          <!-- end nav sequence -->

        <!-- End of while-like KeepRunningUntilFailure loop -->
        <!-- The node <KeepRunningUntilFailure> must have exactly 1 child -->
        </KeepRunningUntilFailure>

    </Sequence>

# TREE 1 OUTPUT:

[INFO] [1737968789.684129520] [demo_bt_server]: //demo_bt_server is ready.
[INFO] [1737968794.539944987] [demo_bt_server]: Received request, executing tree...
[set_locations] set locations ticked...
[set_locations] Found 4 locations.
[get_loc] Targeting location: location1
[go_to_loc] creating nav2 client...
[go_to_loc] Sent goal message
checking done flag...
[IsBatteryLow] battery bt ticked...
[IsBatteryLow] battery bt initialized...
[IsBatteryLow] battery bt joined node...
[IsBatteryLow] battery bt spinned some...
[Interrupt] interrupt bt ticked...
[Interrupt] interrupt bt spinned some... expecting event: cancel
[isObstacle] obstcale bt ticked...
[INFO] [1737968794.552351836] [demo_bt_server]: Robot is free
[get_loc] Targeting location: location2
[go_to_loc] creating nav2 client...
[go_to_loc] Sent goal message
checking done flag...
[IsBatteryLow] battery bt ticked...
[IsBatteryLow] battery bt initialized...
[IsBatteryLow] battery bt joined node...
[IsBatteryLow] battery bt spinned some...
[Interrupt] interrupt bt ticked...
[Interrupt] interrupt bt spinned some... expecting event: cancel
[isObstacle] obstcale bt ticked...
[get_loc] Targeting location: location4
[go_to_loc] creating nav2 client...
[go_to_loc] Sent goal message
checking done flag...
[IsBatteryLow] battery bt ticked...
[IsBatteryLow] battery bt initialized...
[IsBatteryLow] battery bt joined node...
[IsBatteryLow] battery bt spinned some...
[Interrupt] interrupt bt ticked...
[Interrupt] interrupt bt spinned some... expecting event: cancel
[isObstacle] obstcale bt ticked...
[get_loc] Targeting location: location3
[go_to_loc] creating nav2 client...
[go_to_loc] Sent goal message
checking done flag...
[IsBatteryLow] battery bt ticked...
[IsBatteryLow] battery bt initialized...
[IsBatteryLow] battery bt joined node...
[IsBatteryLow] battery bt spinned some...
[Interrupt] interrupt bt ticked...
[Interrupt] interrupt bt spinned some... expecting event: cancel
[isObstacle] obstcale bt ticked...
[get_loc] No more locations!
[INFO] [1737968794.629775500] [demo_bt_server]: Tree execution failed!



# EXAMPLE TREE 2:

      <!-- sequence will only visit next child if current child is returns success. -->
      <Sequence name="terminalop_sequence">

        <!-- Start Mapping System -->
        <TerminalOps task_name="mapping" command="while true; do echo 'Your message here'; sleep 1; done"/>

        <!-- Wait for 5 seconds -->
        <Wait wait_seconds="5"/>

        <!-- kill the Map process -->
        <TerminalOps task_name="mapping" command="cntrl c"/>

      <!-- end dock sequence -->
      </Sequence>

# TREE 2 OUTPUT:

[INFO] [1737976610.144436656] [demo_bt_server]: //demo_bt_server is ready.
[INFO] [1737976617.362485962] [demo_bt_server]: Received request, executing tree...
[TerminalOps] starting...
[TerminalOps] received cmd: while true; do echo 'Your message here'; sleep 1; done
[TerminalOps] process start triggered...
Task started successfully: mapping (PID: 171184)
[INFO] [1737976617.363234077] [Wait]: All nodes will sleep for 5 seconds
Your message here
Your message here
Your message here
Your message here
Your message here
[INFO] [1737976622.363498799] [Wait]: All nodes are waking up
[TerminalOps] starting...
[TerminalOps] received cmd: cntrl c
[TerminalOps] process stop triggered...
Process group (PID: 171184) terminated successfully.
Task stopped successfully: mapping
