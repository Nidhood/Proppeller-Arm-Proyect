cmake_minimum_required(VERSION 3.5)
project(prop_arm_gazebo_control)

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 and ros2_control dependencies:
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ROS2 Control dependencies
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)  

# Message dependencies
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# gz_ros2_control dependencies - this is crucial!
find_package(gz_ros2_control REQUIRED)

# Gazebo dependencies for transport
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Gazebo simulation dependencies
find_package(gz-sim8 REQUIRED)

# Include headers
include_directories(include)

# Build hardware interface library
add_library(prop_arm_hardware_interface SHARED
    src/prop_arm_hardware_interface.cpp
)

ament_target_dependencies(prop_arm_hardware_interface
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    gz_ros2_control
)

# Link Gazebo libraries
target_link_libraries(prop_arm_hardware_interface
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-sim8::gz-sim8
)

# Build Propeller Force Controller Node
add_executable(propeller_force_controller_node
    src/propeller_force_controller.cpp
)

ament_target_dependencies(propeller_force_controller_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    rosgraph_msgs
)

# Build Motor Commander Node  
add_executable(motor_commander_node
    src/motor_commander.cpp
)

# FIXED: Link Gazebo libraries properly
target_link_libraries(motor_commander_node
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)

ament_target_dependencies(motor_commander_node
    rclcpp
    std_msgs
    trajectory_msgs
)

# Export plugin description
pluginlib_export_plugin_description_file(
    gz_ros2_control
    prop_arm_hardware_interface_plugin.xml
)

# Install the plugin library
install(
    TARGETS prop_arm_hardware_interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install executables
install(
    TARGETS 
        propeller_force_controller_node
        motor_commander_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install configs, launch files and URDF
install(
    DIRECTORY config launch urdf
    DESTINATION share/${PROJECT_NAME}
)

# Install public headers
install(
    DIRECTORY include/
    DESTINATION include/
)

# Export for downstream packages
ament_export_dependencies(
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
    gz_ros2_control
    controller_interface
    realtime_tools
    control_msgs
    trajectory_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
)
ament_export_include_directories(include)
ament_export_libraries(prop_arm_hardware_interface)

ament_package()