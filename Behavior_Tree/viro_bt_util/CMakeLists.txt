cmake_minimum_required(VERSION 3.5)
project(viro_bt_util)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----Find Dependencies--------
set(THIS_PACKAGE_DEPS
    ament_cmake
    rclcpp
    rclcpp_action
    std_srvs
    behaviortree_cpp
    behaviortree_ros2
    # i added these:
    geometry_msgs
    sensor_msgs
    tf2
    tf2_geometry_msgs
    nav2_msgs
    rosidl_default_generators
    viro_msgs
    yaml-cpp
)

# Find each package in the list
foreach(dep ${THIS_PACKAGE_DEPS})
    find_package(${dep} REQUIRED)
endforeach()

include_directories(include)
include_directories(${YAML_CPP_INCLUDE_DIRS})

# Main library
set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED
    src/utils.cpp
)
ament_target_dependencies(${LIBRARY_NAME}
    ${THIS_PACKAGE_DEPS}
)
target_link_libraries(${LIBRARY_NAME} yaml-cpp)

# -------BT Server--------
add_executable(demo_bt_server src/demo_bt_server.cpp)
ament_target_dependencies(demo_bt_server ${THIS_PACKAGE_DEPS})
target_link_libraries(demo_bt_server ${LIBRARY_NAME} yaml-cpp)
target_include_directories(demo_bt_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# --------BT Plugins--------
# Function to create and configure a bt plugins
function(create_bt_plugin plugin_name src_file)
    add_library(${plugin_name} SHARED ${src_file})
    target_compile_definitions(${plugin_name} PRIVATE BT_PLUGIN_EXPORT)
    ament_target_dependencies(${plugin_name} ${THIS_PACKAGE_DEPS})
    target_link_libraries(${plugin_name} yaml-cpp)

    # INSTALL plugins for other packages to load
    install(TARGETS
        ${plugin_name}
        LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
        ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
        RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
    )
endfunction()

# Create libraries with the function
# Action plugins
create_bt_plugin(autodock_action src/plugins/action/autodock_action.cpp)
create_bt_plugin(initpose_action src/plugins/action/initpose_action.cpp)
create_bt_plugin(interrupt_action src/plugins/action/interrupt_action.cpp)
create_bt_plugin(mapchange_action src/plugins/action/mapchange_action.cpp)
create_bt_plugin(navigation_action src/plugins/action/navigation_action.cpp)
create_bt_plugin(print_message src/plugins/action/print_message.cpp)
create_bt_plugin(teleop_action src/plugins/action/teleop_action.cpp)
create_bt_plugin(terminalops_action src/plugins/action/terminalops_action.cpp)
create_bt_plugin(wait_action src/plugins/action/wait_action.cpp)

# Condition plugins
create_bt_plugin(is_battery_charging_condition src/plugins/condition/is_battery_charging_condition.cpp)
create_bt_plugin(is_battery_low_condition src/plugins/condition/is_battery_low_condition.cpp)
# create_bt_plugin(is_emergency_condition src/plugins/condition/is_emergency_condition.cpp)
create_bt_plugin(is_goal_dock_condition src/plugins/condition/is_goal_dock_condition.cpp)
create_bt_plugin(is_goal_reached_condition src/plugins/condition/is_goal_reached_condition.cpp)
create_bt_plugin(is_obstacle_condition src/plugins/condition/is_obstacle_condition.cpp)
create_bt_plugin(is_robot_docked_condition src/plugins/condition/is_robot_docked_condition.cpp)

# Control plugins
create_bt_plugin(recovery_node src/plugins/control/recovery_node.cpp)

# Decorator plugins
create_bt_plugin(always_success_except_failure src/plugins/decorator/always_success_except_failure.cpp)
create_bt_plugin(rate_controller src/plugins/decorator/rate_controller.cpp)

## --------Install---------
install(DIRECTORY include/
    DESTINATION include/
)

install(TARGETS
    demo_bt_server
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${LIBRARY_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY config launch tree
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()