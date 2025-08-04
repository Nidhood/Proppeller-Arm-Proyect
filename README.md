# 🚁 PROP ARM SWARM PROJECT 🚁

## Multi-Agent Drone System for Strategic Fire Detection and Management

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge&logo=gazebo&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-In_Development-yellow?style=for-the-badge)

**Leveraging aerodynamics and dynamics to develop a fleet of connected drones capable of detecting and managing fires strategically**

</div>

---

## 🎯 Project Vision

Welcome to the **Prop Arm Swarm Project**! This cutting-edge initiative focuses on developing an intelligent multi-agent drone system that combines advanced aerodynamics, sophisticated control systems, and collaborative algorithms to revolutionize fire detection and management strategies.

---

## 🔧 Technology Stack

<div align="center">

|     **Technology**     |             **Purpose**              | **Version** |
| :--------------------: | :----------------------------------: | :---------: |
|   🐢 **ROS 2 Jazzy**   | Robotic middleware and communication |   Latest    |
| 🛠️ **Gazebo Harmonic** |    Physics simulation and testing    |   Latest    |
|      🌐 **RViz**       |       Real-time visualization        |   Latest    |
|     🐍 **Python**      |      Launch systems and control      |    3.8+     |
|       ⚙️ **C++**       |       High-performance plugins       |     17+     |

</div>

---

## ✨ Key Features

### 🤝 **Multi-Agent Collaboration**

- Coordinated swarm behavior for efficient area coverage
- Dynamic task allocation and load balancing
- Fault-tolerant communication protocols

### 🌬️ **Advanced Aerodynamics**

- Optimized propeller arm design for stability
- Real-time flight pattern adaptation
- Energy-efficient trajectory planning

### 🔥 **Dynamic Fire Mapping**

- Real-time fire detection using advanced sensors
- Collaborative mapping and data fusion
- Predictive fire spread modeling

### 🛩️ **Scalable Architecture**

- Modular design for easy system expansion
- Terrain-adaptive flight capabilities
- Configurable for various mission scenarios

---

## 🎥 Project Showcase

<div align="center">

|                               **Simulation Environment**                                |                             **3D Model Visualization**                             |
| :-------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------: |
| ![Simulation](https://via.placeholder.com/400x250/0066cc/ffffff?text=Gazebo+Simulation) | ![Model](https://via.placeholder.com/400x250/ff6600/ffffff?text=3D+Prop+Arm+Model) |
|                     Complete physics simulation with fire scenarios                     |                Detailed URDF model with collision and visual meshes                |

</div>

---

## 📦 Project Architecture

```
🏗️ prop_arm_ws/
├── 📜 LICENSE                          # MIT License
├── 📖 README.md                        # This file
└── 📁 src/
    ├── 🎯 prop_arm/                    # Meta-package & project coordination
    ├── 🎨 prop_arm_description/        # Robot models & visualization
    │   ├── 🚀 launch/                  # Visualization launch files
    │   ├── 🎭 models/prop_arm/         # Complete 3D models
    │   │   ├── 🔺 meshes/collisions/   # STL collision models
    │   │   ├── 🎨 meshes/visuals/      # DAE visual models
    │   │   └── 🤖 urdf/                # Robot descriptions
    │   └── 👁️ rviz/                    # Visualization configs
    ├── 🌍 prop_arm_gazebo/            # Simulation environment
    │   ├── 🚀 launch/                  # Simulation launchers
    │   └── 🗺️ worlds/                  # Custom world files
    ├── 🎛️ prop_arm_gazebo_control/    # Control systems
    └── 🔌 prop_arm_gazebo_plugins/    # Custom sensor plugins
```

---

## 🚀 Quick Start Guide

### 📋 Prerequisites

<details>
<summary><b>🔧 System Requirements</b></summary>

- **OS**: Ubuntu 22.04 LTS (Recommended)
- **ROS 2**: Jazzy Jalopy distribution
- **Gazebo**: Harmonic version
- **Python**: 3.8 or higher
- **Compiler**: GCC 9+ with C++17 support

</details>

### 🛠️ Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/Nidhood/SWARM_PROJECT.git prop_arm_ws
   cd prop_arm_ws
   ```

2. **Install Dependencies**

   ```bash
   sudo apt update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### 🎮 Launch Commands

<details>
<summary><b>🎯 Visualization & Description</b></summary>

```bash
# Launch RViz with prop arm model
ros2 launch prop_arm_description display_model.launch.py

# Display in RViz only
ros2 launch prop_arm_description display_rviz.launch.xml

# Publish URDF for external tools
ros2 launch prop_arm_description publish_urdf.launch.py
```

</details>

<details>
<summary><b>🌍 Simulation Environment</b></summary>

```bash
# Start custom world
ros2 launch prop_arm_gazebo start_world.launch.py

# Spawn prop arm in simulation
ros2 launch prop_arm_gazebo spawn_prop_arm.launch.py

# Full simulation with description
ros2 launch prop_arm_gazebo spawn_prop_arm_description.launch.py
```

</details>

---

## 🔬 Technical Specifications

### 🤖 **Robot Model Details**

- **Visual Models**: High-fidelity DAE meshes for realistic rendering
- **Collision Models**: Optimized STL meshes for physics simulation
- **Components**: Base platform, articulated arm, motor unit, propeller
- **Sensors**: Configurable sensor suite via plugin system

### 🎛️ **Control Systems**

- **Flight Controller**: Advanced PID-based stabilization
- **Mission Planner**: Autonomous waypoint navigation
- **Swarm Coordinator**: Multi-agent task distribution
- **Safety Systems**: Emergency landing and collision avoidance

### 🔌 **Plugin Architecture**

- **Sensor Plugins**: Camera, LIDAR, IMU, GPS integration
- **Physics Plugins**: Custom aerodynamics and thrust models
- **Communication Plugins**: Inter-drone data exchange
- **Visualization Plugins**: Real-time status and telemetry

---

## 🎯 Roadmap & Future Development

### 🚧 **Phase 1: Foundation** _(Current)_

- [x] Complete project structure
- [x] Basic URDF model implementation
- [x] Gazebo simulation integration
- [ ] Control system implementation

### 🔥 **Phase 2: Fire Detection**

- [ ] Thermal sensor integration
- [ ] Fire detection algorithms
- [ ] Real-time mapping system
- [ ] Data fusion algorithms

### 🤝 **Phase 3: Swarm Intelligence**

- [ ] Multi-agent coordination
- [ ] Collaborative pathfinding
- [ ] Dynamic task allocation
- [ ] Fault tolerance systems

### 🌟 **Phase 4: Advanced Features**

- [ ] Machine learning integration
- [ ] Predictive fire modeling
- [ ] Emergency response protocols
- [ ] Real-world deployment testing

---

## 🤝 Contributing

We welcome contributions from the community! Here's how you can help:

<div align="center">

|         **Type**          |                           **How to Contribute**                            |
| :-----------------------: | :------------------------------------------------------------------------: |
|    🐛 **Bug Reports**     |      [Open an Issue](https://github.com/Nidhood/SWARM_PROJECT/issues)      |
|  ✨ **Feature Requests**  | [Start a Discussion](https://github.com/Nidhood/SWARM_PROJECT/discussions) |
| 🔧 **Code Contributions** |  [Submit a Pull Request](https://github.com/Nidhood/SWARM_PROJECT/pulls)   |
|   📖 **Documentation**    |     [Improve our Docs](https://github.com/Nidhood/SWARM_PROJECT/wiki)      |

</div>

### 🔄 **Development Workflow**

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

---

## 📜 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

---

## 🏆 Team & Acknowledgments

<div align="center">

### 🤖 **Proudly Developed by IEEE RAS Javeriana**

![IEEE RAS Javeriana](https://raw.githubusercontent.com/Nidhood/SWARN_PROJECT/main/img/RASJaveriana.png)

_Advancing robotics and automation through innovative research and development_

</div>

### 📞 **Connect With Us**

<div align="center">

[![GitHub](https://img.shields.io/badge/GitHub-Nidhood-black?style=for-the-badge&logo=github)](https://github.com/Nidhood)
[![Email](https://img.shields.io/badge/Email-Contact_Us-red?style=for-the-badge&logo=gmail)](mailto:your-email@example.com)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-IEEE_RAS_Javeriana-blue?style=for-the-badge&logo=linkedin)](https://linkedin.com/company/ieee-ras-javeriana)

</div>

---

<div align="center">

### 🌍 **Together, we're making fire management more efficient and impactful** 🔥

_Thank you for your interest in the Prop Arm Swarm Project!_

⭐ **Don't forget to star this repository if you find it useful!** ⭐

</div>
