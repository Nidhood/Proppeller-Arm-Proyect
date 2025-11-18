cmake_minimum_required(VERSION 3.5)

# Project name:
project(prop_arm_control)

# Set C++ standard:
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies:
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories:
include_directories(include)

# --------------------- Build executables ------------------------

# PID Controller Node library build:
add_executable(pid_controller_node src/pid_controller_node.cpp)
ament_target_dependencies(pid_controller_node rclcpp std_msgs)
target_include_directories(pid_controller_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# ---------------------- Install targets -------------------------

# Install executables:
install(TARGETS pid_controller_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories:
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

# Ament package:
ament_package()