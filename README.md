
# 🤖 Articulated Bot (ROS 2 Jazzy)

A complete autonomous differential drive robot built with **ROS 2 Jazzy**. This project demonstrates Simulation, Simultaneous Localization and Mapping (SLAM), and Navigation using Gazebo Harmonic.

## ✨ Features

- **🕹️ Teleoperation**: Manual control using keyboard teleop.
- **🗺️ SLAM (Simultaneous Localization & Mapping)**: Generates 2D occupancy grid maps using `slam_toolbox`.
- **🧠 Autonomous Navigation**: Ready for integration with the Nav2 stack for path planning.
- **👁️ Perception**: Equipped with Lidar sensors (simulated in Gazebo).
- **🏗️ Custom Worlds**: Includes a specialized maze environment optimized for mapping tests.

## 🛠️ Prerequisites

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS 2 Distro**: Jazzy Jalisco
- **Simulator**: Gazebo Harmonic (`ros-jazzy-ros-gz`)

### Dependencies
Install the required ROS 2 packages:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-ros-gz

```

## 🚀 Installation

1. **Clone the repository:**

```bash
mkdir -p ~/bot_ws/src
cd ~/bot_ws/src
# Replace with your repo URL
git clone [https://github.com/sudoaptinstalltarun/articulat_bot.git](https://github.com/sudoaptinstalltarun/articulat_bot.git) .

```

2. **Build the workspace:**

```bash
cd ~/bot_ws
colcon build --symlink-install
source install/setup.bash

```

## 🏃 Usage Guide

### 1. Launch Simulation

Start the robot in the custom Gazebo Maze environment.

**Standard Launch:**

```bash
ros2 launch my_bot launch_sim.launch.py

```

**⚠️ NVIDIA Users (ASUS TUF / Gaming Laptops):**
Use this command to prevent Lidar lag and rendering crashes:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch my_bot launch_sim.launch.py

```

### 2. Mapping Mode (SLAM)

To create a new map of the environment:

**Step A: Start SLAM**

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

```

**Step B: Control the Robot**
Open a new terminal and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**Step C: Visualize**
Open RViz to see the map building in real-time:

```bash
rviz2

```

*(Add the "Map" and "LaserScan" topics in the RViz window)*

**Step D: Save Map**
When finished mapping, run:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map

```

## 📂 Project Structure

```text
my_bot/
├── launch/             # Launch files for Simulation and SLAM
├── description/        # Robot URDF/Xacro files (lidar, chassis, properties)
├── worlds/             # Custom Gazebo world files (maze.world)
├── config/             # Configuration parameters (SLAM toolbox, etc.)
└── package.xml

```

## 👨‍💻 Author

**Tarun** GitHub: [@sudoaptinstalltarun](https://www.google.com/search?q=https://github.com/sudoaptinstalltarun)

```

```
