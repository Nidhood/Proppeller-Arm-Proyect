Below is an updated **README.md** that reflects the true goal of your project—modeling and controlling a single-axis propeller arm per the MIT IntroControl Prelab 10 model—and cites that page as a reference.

````markdown
# ⚙️ Propeller Arm Control System

> **Reference**: “Prelab 10: Modeling a Propeller and Arm”  
> MIT 6.011 Introduction to Control (Fall 2021)  
> https://introcontrol.mit.edu/fall21/prelabs/prelab10/model

---

<div align="center">
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-orange?style=for-the-badge&logo=gazebo)
![License MIT](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status WIP](https://img.shields.io/badge/Status-In_Development-yellow?style=for-the-badge)
</div>

A ROS 2 + Gazebo simulation of a single-axis **Propeller Arm** that can hold a specified angle against gravity using a closed-loop controller. We implement:

1. **Dynamic mechanical model** (motor torque ↔ arm inertia & gravity)
2. **Electrical model** (brushless motor + potentiometer feedback)
3. **ros2_control** plugin & PID controller to maintain any angle (e.g. 0°, 45°) in simulation.

---

## 📐 Project Layout

```text
prop_arm_ws/
├── LICENSE
├── README.md                ← this file
└── src/
    ├── prop_arm_description/
    │   ├── launch/          ← rviz & URDF publishers
    │   ├── models/prop_arm/ ← mesh & URDF
    │   └── rviz/            ← display configs
    ├── prop_arm_gazebo/
    │   ├── launch/          ← world & spawn launchers
    │   └── worlds/          ← custom SDF world
    ├── prop_arm_gazebo_control/
    │   ├── config/          ← ros2_controllers.yaml
    │   ├── launch/          ← controller spawner
    │   ├── src/             ← hardware_interface.cpp
    │   ├── include/         ← hardware_interface.hpp
    │   └── urdf/            ← prop_arm_control.xacro
    └── prop_arm_gazebo_plugins/
        └── urdf/            ← sensor & motor plugins
```
````

---

## 🛠️ Technology Stack

| Component        | Role                                    | Version      |
| ---------------- | --------------------------------------- | ------------ |
| **ROS 2**        | Middleware & lifecycle management       | Jazzy        |
| **ros2_control** | Hardware-interface & controller_manager | latest       |
| **Gazebo**       | Physics-based simulation                | Harmonic     |
| **C++17**        | System interfaces & plugins             | GCC 9+       |
| **Python 3.8+**  | Launch files & scripting                | ROS 2 launch |

---

## ⚡ Quick Start

### 1. Clone & install prerequisites

```bash
git clone https://github.com/Nidhood/prop_arm_ws.git
cd prop_arm_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build and source

```bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch simulation

```bash
# 1) Start Gazebo world & spawn the arm
ros2 launch prop_arm_gazebo start_world.launch.py
ros2 launch prop_arm_gazebo spawn_prop_arm_description.launch.py

# 2) Load and activate the position controller
ros2 launch prop_arm_gazebo_control load_controller.launch.py
```

Once up:

- The **arm** will hold the default angle (0°).
- Send a new setpoint via `/arm_position_controller/joint_trajectory` or use `ros2 topic pub` to change to, e.g., 45°.

---

## 🧮 Modeling & Control

1. **Mechanical dynamics**
   • Arm treated as rigid rod + motor torque → nonlinear equation
   • Linearized about operating point via Taylor expansion, yielding `τ ≈ Kₜ·i` and small-angle gravity torque ≈ `m·g·l·θ`

2. **Electrical & motor model**
   • Brushless motor: electrical time constant + torque constant `Kₜ`
   • Potentiometer as angle sensor with known gain

3. **ros2_control integration**
   • Custom `PropArmHardware` implementing `GazeboSimSystemInterface`
   • PID gains tuned to achieve stable angle tracking

For full derivation and step-by-step modeling, see the MIT prelab:
[https://introcontrol.mit.edu/fall21/prelabs/prelab10/model](https://introcontrol.mit.edu/fall21/prelabs/prelab10/model)

---

## 📜 License

This project is released under the **MIT License**. See [LICENSE](LICENSE).

---

<div align="center">
⭐ If this helped your control-systems learning, please give it a star! ⭐
</div>
```
::contentReference[oaicite:0]{index=0}
