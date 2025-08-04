cmake_minimum_required(VERSION 3.8)
project(prop_arm_description)


# Compiler options:
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ros dependencies:
find_package(ament_cmake REQUIRED)

# Install directories inside the package:
install(DIRECTORY launch models rviz
    DESTINATION share/${PROJECT_NAME}
    )

# Testing options:
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Ament package configuration:
ament_package()