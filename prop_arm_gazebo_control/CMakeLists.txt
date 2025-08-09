cmake_minimum_required(VERSION 3.16)
project(prop_arm_gazebo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(Eigen3 REQUIRED)

# Add Gazebo dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Include directories
include_directories(include)

# Hardware Interface Library
add_library(prop_arm_hardware_interface SHARED
    src/prop_arm_hardware_interface.cpp
)

target_compile_features(prop_arm_hardware_interface PUBLIC c_std_99 cxx_std_17)
target_include_directories(prop_arm_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(prop_arm_hardware_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    gz_ros2_control
    gz-sim8
    gz-transport13
    gz-msgs10
)

# Export hardware plugins
pluginlib_export_plugin_description_file(gz_ros2_control prop_arm_hardware_interface_plugin.xml)

# Angle Hold Controller as both executable and component library
add_library(angle_hold_controller_component SHARED
    src/angle_hold_controller.cpp
)

target_compile_features(angle_hold_controller_component PUBLIC c_std_99 cxx_std_17)
target_include_directories(angle_hold_controller_component PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(angle_hold_controller_component
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
)

target_link_libraries(angle_hold_controller_component
    Eigen3::Eigen
)

# Component registration for angle_hold_controller
rclcpp_components_register_nodes(angle_hold_controller_component "prop_arm_ctrl::AngleHoldController")

# FIXED: Create the executable with the correct name that matches the launch file
add_executable(angle_hold_controller_node 
    src/angle_hold_controller_main.cpp
)

target_compile_features(angle_hold_controller_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(angle_hold_controller_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(angle_hold_controller_node
    rclcpp
)

target_link_libraries(angle_hold_controller_node
    angle_hold_controller_component
)

# Also create a generic angle_hold_controller executable for compatibility
add_executable(angle_hold_controller 
    src/angle_hold_controller_main.cpp
)

target_compile_features(angle_hold_controller PUBLIC c_std_99 cxx_std_17)
target_include_directories(angle_hold_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(angle_hold_controller
    rclcpp
)

target_link_libraries(angle_hold_controller
    angle_hold_controller_component
)

# Motor Commander executable
add_executable(motor_commander_node 
    src/motor_commander.cpp
)

target_compile_features(motor_commander_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(motor_commander_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(motor_commander_node
    rclcpp
    std_msgs
    trajectory_msgs
    gz-transport13
    gz-msgs10
)

# Install targets
install(TARGETS prop_arm_hardware_interface angle_hold_controller_component
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# FIXED: Install both executables
install(TARGETS angle_hold_controller_node angle_hold_controller motor_commander_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
)

# Install header files
install(DIRECTORY include/
    DESTINATION include/
)

# Install plugin description
install(FILES prop_arm_hardware_interface_plugin.xml
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    gz_ros2_control
)

ament_package()