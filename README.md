# SLAM & Path Planning (ROS2)

This repository contains my work on SLAM and path planning using ROS2 (Humble) and TurtleBot3 in Gazebo simulation. It includes configuration files, generated maps, and parameter tuning for the Nav2 stack.

Detailed progress and planner comparisons are documented in [my_progress.md](my_progress.md).

## 🚀 Step-by-Step Setup Instructions to use turtlebot3 in my custom world

This guide walks you through creating a ROS 2 workspace, pulling this repository, building it, and launching the TurtleBot3 simulation with Navigation2 using the provided map.

---
### 📁 1. Create a ROS 2 Workspace
```bash
mkdir -p ~/nav2_ws
cd ~/nav2_ws
```
### ⬇️ 2. Clone this repo
```bash
git clone https://github.com/Hossam-Eldin-Omar/SLAM_and_path_planning.git
```
### 🔨 3. Build the Workspace
```bash
cd ~/nav2_ws
colcon build
source install/setup.bash
```

### 🐢 4. Launch TurtleBot3 in Gazebo
```bash
export TURTLEBOT3_MODEL=waffle
```
#### Launch Gazebo simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_my_house_world.launch.py
```

### 🗺️ 5. Launch Navigation2 with Your Map
```bash
cd ~/nav2_ws
source install/setup.bash
```

#### Launch Nav2 using your provided map and localization
```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_house_map.yaml
```

## ✅ You're Ready!
Your TurtleBot3 should now spawn inside Gazebo and localize using the pre-built map with Navigation2.


