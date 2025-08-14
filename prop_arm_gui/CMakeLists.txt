cmake_minimum_required(VERSION 3.16)
project(prop_arm_gui)

# Set C++ standard to C++17 for Qt6 compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler settings
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

# Qt6 dependencies with proper components
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts OpenGL)

# Automatically handle Qt's MOC, RCC and UIC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(include)

# Qt6 executable sources - CLEANED UP (removed dashboard components)
set(GUI_SOURCES
    src/main.cpp
    src/main_window.cpp
    src/prop_arm_gui_node.cpp
    src/real_time_plot.cpp
    src/aerospace_data_visualizer.cpp
)

set(GUI_HEADERS
    include/prop_arm_gui/main_window.hpp
    include/prop_arm_gui/prop_arm_gui_node.hpp
    include/prop_arm_gui/real_time_plot.hpp
    include/prop_arm_gui/aerospace_data_visualizer.hpp
)

# Create executable
add_executable(prop_arm_gui
    ${GUI_SOURCES}
    ${GUI_HEADERS}
)

# ROS dependencies
ament_target_dependencies(prop_arm_gui
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# Qt6 libraries
target_link_libraries(prop_arm_gui
    Qt6::Core
    Qt6::Widgets
    Qt6::Charts
    Qt6::OpenGL
)

# Ensure Qt6 definitions are applied
target_compile_definitions(prop_arm_gui PRIVATE 
    QT_CHARTS_LIB
    QT_WIDGETS_LIB
    QT_CORE_LIB
    QT_OPENGL_LIB
)

# Install targets
install(TARGETS prop_arm_gui
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Install config files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Install resource files (if any)
install(DIRECTORY resources
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Package setup
ament_package()