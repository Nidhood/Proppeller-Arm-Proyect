# Changelog

All notable changes to the Prop Arm project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Complete project structure for multi-package ROS 2 workspace
- Core prop_arm meta-package with organized dependencies
- Comprehensive URDF model system with visual and collision meshes
- Advanced Gazebo simulation environment with custom world
- Specialized control and plugin systems for enhanced functionality

### New important command:

- colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
