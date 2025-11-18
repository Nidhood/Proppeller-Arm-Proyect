cmake_minimum_required(VERSION 3.5)

# Project name:
project(prop_arm_gazebo_plugins)

# Set C++ standard:
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ros dependencies:
find_package(ament_cmake REQUIRED)

# ---------------------- Install targets -------------------------

# Install directories inside the package:
install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

# Ament package configuration:
ament_package()