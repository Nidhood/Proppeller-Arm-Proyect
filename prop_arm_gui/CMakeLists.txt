cmake_minimum_required(VERSION 3.16)

# Project name:
project(prop_arm_gui
    VERSION 1.0.0
    DESCRIPTION "Propeller Arm GUI for ROS2"
    LANGUAGES CXX
)

# Set C++ standard:
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
    )
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find Qt6 dependencies
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts Svg)

# Enable Qt's automatic tools
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ----------------------- Header Files ---------------------------
set(APP_HEADERS
    include/prop_arm_gui/app/app_context.hpp
    include/prop_arm_gui/app/main_window.hpp
    include/prop_arm_gui/app/ros_qt_bridge.hpp
)

set(CORE_HEADERS
    include/prop_arm_gui/core/logging/data_exporter.hpp
    include/prop_arm_gui/core/logging/data_recorder.hpp
    include/prop_arm_gui/core/models/calibration_state.hpp
    include/prop_arm_gui/core/models/command_type.hpp
    include/prop_arm_gui/core/models/control_node.hpp
    include/prop_arm_gui/core/models/csv_export_options.hpp
    include/prop_arm_gui/core/models/domain_buffer.hpp
    include/prop_arm_gui/core/models/domain.hpp
    include/prop_arm_gui/core/models/recording_state.hpp
    include/prop_arm_gui/core/models/recording_stats.hpp
    include/prop_arm_gui/core/models/telemetry_signal_type.hpp
    include/prop_arm_gui/core/models/plot_signal_type.hpp
    include/prop_arm_gui/core/models/telemetry_sample.hpp
    include/prop_arm_gui/core/models/telemetry_store.hpp
    include/prop_arm_gui/core/models/theme_id.hpp
    include/prop_arm_gui/core/models/calibration_types.hpp
    include/prop_arm_gui/core/services/recorder_service.hpp
    include/prop_arm_gui/core/services/calibration_service.hpp
    include/prop_arm_gui/core/services/urdf_updater.hpp
    include/prop_arm_gui/core/services/signal_alignment.hpp
    include/prop_arm_gui/core/services/calibration_optimizer.hpp
    include/prop_arm_gui/core/math/units.hpp
    include/prop_arm_gui/core/charts/chart_window.hpp
    include/prop_arm_gui/core/charts/curve_builder.hpp
    include/prop_arm_gui/core/charts/hover_interpolator.hpp
    include/prop_arm_gui/core/charts/sample_rate_estimator.hpp
    include/prop_arm_gui/core/charts/value_filter.hpp
)

set(INFRA_HEADERS
    include/prop_arm_gui/infra/ros/prop_arm_ros_bridge.hpp
    include/prop_arm_gui/infra/ros/topic_registry.hpp
    include/prop_arm_gui/infra/ros/limits_registry.hpp
    include/prop_arm_gui/infra/ros/prop_arm_ros_bridge_command_sink.hpp
)

set(UI_HEADERS
    include/prop_arm_gui/ui/pages/calibration_page.hpp
    include/prop_arm_gui/ui/pages/control_panel_page.hpp
    include/prop_arm_gui/ui/pages/dashboard_page.hpp
    include/prop_arm_gui/ui/style/theme.hpp
    include/prop_arm_gui/ui/style/theme_manager.hpp
    include/prop_arm_gui/ui/widgets/chart_base.hpp
    include/prop_arm_gui/ui/widgets/chart_frame.hpp
    include/prop_arm_gui/ui/widgets/hover_chart_view.hpp
    include/prop_arm_gui/ui/widgets/export_preview_dialog.hpp
    include/prop_arm_gui/ui/widgets/panel.hpp
    include/prop_arm_gui/ui/widgets/section_header.hpp
    include/prop_arm_gui/ui/widgets/value_tile.hpp
    include/prop_arm_gui/ui/style/chart_theme_applier.hpp
    include/prop_arm_gui/ui/widgets/duty_plot.hpp
    include/prop_arm_gui/ui/widgets/angle_plot.hpp
    include/prop_arm_gui/ui/widgets/angle_ref_plot.hpp
    include/prop_arm_gui/ui/widgets/error_plot.hpp
    include/prop_arm_gui/ui/widgets/tracking_error_plot.hpp
    include/prop_arm_gui/ui/widgets/speed_plot.hpp
    include/prop_arm_gui/ui/widgets/pwm_plot.hpp
    include/prop_arm_gui/ui/widgets/arm_visualizer.hpp
)

# ----------------------- Source Files ---------------------------
set(APP_SOURCES
    src/app/app_context.cpp
    src/app/main_window.cpp
    src/app/ros_qt_bridge.cpp
)

set(CORE_SOURCES
    src/core/logging/data_exporter.cpp
    src/core/logging/data_recorder.cpp
    src/core/models/telemetry_store.cpp
    src/core/services/recorder_service.cpp
    src/core/services/calibration_service.cpp
    src/core/services/urdf_updater.cpp
    src/core/services/signal_alignment.cpp
    src/core/services/calibration_optimizer.cpp
    src/core/charts/chart_window.cpp
    src/core/charts/curve_builder.cpp
    src/core/charts/hover_interpolator.cpp
    src/core/charts/sample_rate_estimator.cpp
    src/core/charts/value_filter.cpp
    src/ui/style/chart_theme_applier.cpp
)

set(INFRA_SOURCES
    src/infra/ros/prop_arm_ros_bridge.cpp
    src/infra/ros/topic_registry.cpp
    src/infra/ros/limits_registry.cpp
    src/infra/ros/prop_arm_ros_bridge_command_sink.cpp
)

set(UI_SOURCES
    src/ui/pages/calibration_page.cpp
    src/ui/pages/control_panel_page.cpp
    src/ui/pages/dashboard_page.cpp
    src/ui/style/theme_manager.cpp
    src/ui/widgets/chart_base.cpp
    src/ui/widgets/chart_frame.cpp
    src/ui/widgets/hover_chart_view.cpp
    src/ui/widgets/export_preview_dialog.cpp
    src/ui/widgets/panel.cpp
    src/ui/widgets/section_header.cpp
    src/ui/widgets/value_tile.cpp
    src/ui/widgets/duty_plot.cpp
    src/ui/widgets/angle_plot.cpp
    src/ui/widgets/angle_ref_plot.cpp
    src/ui/widgets/error_plot.cpp
    src/ui/widgets/tracking_error_plot.cpp
    src/ui/widgets/speed_plot.cpp
    src/ui/widgets/pwm_plot.cpp
    src/ui/widgets/arm_visualizer.cpp
)

qt_add_resources(PROP_ARM_GUI_RCC
    resources/resources.qrc
)

# --------------------- Build executables ------------------------
add_executable(${PROJECT_NAME}_node
    src/main.cpp
    ${PROP_ARM_GUI_RCC}
    ${APP_SOURCES}
    ${CORE_SOURCES}
    ${INFRA_SOURCES}
    ${UI_SOURCES}
    ${APP_HEADERS}
    ${CORE_HEADERS}
    ${INFRA_HEADERS}
    ${UI_HEADERS}
)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    std_msgs
)

target_link_libraries(${PROJECT_NAME}_node
    Qt6::Core
    Qt6::Widgets
    Qt6::Charts
    Qt6::Svg
)

# ---------------------- Install targets -------------------------

# Install targets:
install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)


# Install additional directories
install(DIRECTORY
    launch
    config
    resources
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Package setup
ament_package()