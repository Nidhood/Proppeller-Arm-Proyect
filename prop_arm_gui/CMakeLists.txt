cmake_minimum_required(VERSION 3.16)

# Project name:
project(prop_arm_gui)

# Set C++ standard:
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Qt6 dependencies with proper components:
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts OpenGL)

# Automatically handle Qt's MOC, RCC and UIC:
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(include)

# ------------------ Set sources and headers --------------------

set(GUI_SOURCES
    src/main.cpp
    src/prop_arm_gui_node.cpp
    src/chart_base.cpp
    src/data_exporter.cpp
    src/data_recorder.cpp
    src/aerospace_data_visualizer.cpp
    src/real_time_plot.cpp
    src/data_exporter.cpp
)

set(GUI_HEADERS
    include/prop_arm_gui/main_window.hpp
    include/prop_arm_gui/prop_arm_gui_node.hpp
    include/prop_arm_gui/chart_base.hpp
    include/prop_arm_gui/data_exporter.hpp
    include/prop_arm_gui/data_recorder.hpp 
    include/prop_arm_gui/aerospace_data_visualizer.hpp
    include/prop_arm_gui/real_time_plot.hpp
)

# --------------------- Build executables ------------------------

# Prop Arm GUI executable:
add_executable(prop_arm_gui ${GUI_SOURCES} ${GUI_HEADERS})
ament_target_dependencies(prop_arm_gui rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(prop_arm_gui Qt6::Core Qt6::Widgets Qt6::Charts Qt6::OpenGL)

# ---------------------- Install targets -------------------------

# Install targets:
install(TARGETS prop_arm_gui
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files:
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Install config files:
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Install resource files:
install(DIRECTORY resources
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Package setup
ament_package()