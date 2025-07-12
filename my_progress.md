# SLAM and Path Planning with TurtleBot3

This file documents my learning and implementation journey with **ROS2**, **TurtleBot3**, **SLAM**, and **Path Planning** using simulation tools like **Gazebo** and **RViz**.

## 🚀 Progress Overview

### ✅ Mapping and Navigation
- Successfully generated **two environment maps** using:
  - `turtlebot3` in **Gazebo**
  - Visualization and navigation through **RViz**
  - ROS2 packages and tools (e.g., `slam_toolbox`, `nav2`, etc.)
- Enabled **autonomous navigation** of the robot through the mapped environments.

### 🔍 Nav2 Stack & Costmap Configuration
- Understood the structure and components of the **Nav2 stack**.
- Explored how:
  - The **costmap layers**
  - The **inflation radius**
  - Obstacle padding  
  affect the robot's ability to plan and execute paths toward a goal.

### 🧠 Path Planning Algorithms
- Studied and implemented basic **global** and **local** path planning strategies on the `turtlebot3`:
  - Global planners: e.g., A*, Dijkstra
  - Local planners: e.g., DWB, TEB
- Analyzed their behavior and effect on the robot’s movement.

### 📄 Research Exploration
- Read and understood the concepts behind the **Dynamic Window Approach (DWA)**.
- Reference: [Real-Time Obstacle Avoidance for Fast Mobile Robots (Fox et al., 1997)](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)

---

## 🛠 Tools & Technologies
- ROS2 (Humble/Foxy)
- TurtleBot3
- Gazebo
- RViz
- Nav2
- SLAM Toolbox