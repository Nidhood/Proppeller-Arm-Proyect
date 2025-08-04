cmake_minimum_required(VERSION 3.5)
project(prop_arm_gazebo_control)

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 and ros2_control dependencies:
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gz_ros2_control REQUIRED) 
find_package(controller_interface REQUIRED)

# Include headers
include_directories(include)

# Build hardware interface library
add_library(prop_arm_hardware_interface SHARED
    src/prop_arm_hardware_interface.cpp
)

ament_target_dependencies(prop_arm_hardware_interface
    rclcpp
    hardware_interface
    pluginlib
    gz_ros2_control
    controller_interface
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface prop_arm_hardware_interface_plugin.xml)

# Install folders and targets
install(
    TARGETS prop_arm_hardware_interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY
    config
    launch
    urdf
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/
)

install(
    FILES resource/prop_arm_hardware_interface_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

# Export info for downstream packages
ament_export_dependencies(
    rclcpp
    hardware_interface
    pluginlib
    gz_ros2_control
    controller_interface
)

ament_export_include_directories(include)
ament_export_libraries(prop_arm_hardware_interface)

ament_package()
