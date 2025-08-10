cmake_minimum_required(VERSION 3.16)
project(prop_arm_gazebo_control)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
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

# Gazebo/Ign
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

include_directories(include)

# Proper hardware interface implementation first 
add_library(prop_arm_hardware_interface SHARED
    src/prop_arm_hardware_interface.cpp  # This file needs to be created
)
target_compile_features(prop_arm_hardware_interface PUBLIC c_std_99 cxx_std_17)
target_include_directories(prop_arm_hardware_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(prop_arm_hardware_interface
    hardware_interface pluginlib rclcpp rclcpp_lifecycle gz_ros2_control std_msgs
    gz-sim8 gz-transport13 gz-msgs10
)

# Create plugin XML file if it doesn't exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/prop_arm_hardware_interface_plugin.xml")
    pluginlib_export_plugin_description_file(gz_ros2_control prop_arm_hardware_interface_plugin.xml)
endif()

# --- Angle-hold controller (exe) ---
add_executable(angle_hold_ss_controller
    src/angle_hold_ss_controller.cpp
)
target_compile_features(angle_hold_ss_controller PUBLIC c_std_99 cxx_std_17)
target_include_directories(angle_hold_ss_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(angle_hold_ss_controller rclcpp std_msgs)
target_link_libraries(angle_hold_ss_controller Eigen3::Eigen)

# --- VEmfPublisher (exe) ---
add_executable(v_emf_publisher
    src/v_emf_publisher.cpp
)
target_compile_features(v_emf_publisher PUBLIC c_std_99 cxx_std_17)
target_include_directories(v_emf_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(v_emf_publisher rclcpp std_msgs)

# --- Motor commander (exe) ---
add_executable(motor_commander_node
    src/motor_commander.cpp
)
target_compile_features(motor_commander_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(motor_commander_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_commander_node
    rclcpp std_msgs trajectory_msgs gz-transport13 gz-msgs10
)

# --- Install targets ---
install(TARGETS
    prop_arm_hardware_interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Executables
install(TARGETS
    angle_hold_ss_controller
    v_emf_publisher
    motor_commander_node
    DESTINATION lib/${PROJECT_NAME}
)

# Other assets
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/ OPTIONAL)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/ OPTIONAL)
install(DIRECTORY launch urdf config include src 
    DESTINATION share/${PROJECT_NAME}
    )
install(DIRECTORY include/ DESTINATION include/)

# Install plugin XML only if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/prop_arm_hardware_interface_plugin.xml")
    install(FILES prop_arm_hardware_interface_plugin.xml DESTINATION share/${PROJECT_NAME})
endif()

# --- Export for downstream consumers ---
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    hardware_interface pluginlib rclcpp rclcpp_lifecycle gz_ros2_control
)

# Tests
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()