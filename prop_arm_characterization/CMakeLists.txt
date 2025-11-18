cmake_minimum_required(VERSION 3.5)

# Project name:
project(prop_arm_characterization)

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

# Include directories:
include_directories(include)

# =============================================================== #
#                        Build libraries                          #
# =============================================================== #

# Motor speed model library build:
add_library(motor_speed_model SHARED
    src/motor_speed_model.cpp
)
target_compile_features(motor_speed_model PUBLIC c_std_99 cxx_std_17)
target_include_directories(motor_speed_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> 
)

# =============================================================== #
#                        Install targets                          #                  
# =============================================================== #

# Install headers:
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Install libraries:
install(TARGETS motor_speed_model
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Export targets:
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies()

# Ament package:
ament_package()