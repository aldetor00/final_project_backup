# final_project_back

# Autonomous Multi-Robot Logistics: Fra2mo & KUKA iiwa Integration

##  Project Overview

This project simulates an **Industry 4.0 collaborative logistics scenario** using ROS 2 Humble and Gazebo. It features a Multi-Robot System (MRS) composed of:
1.  **Fra2mo**: A differential drive mobile robot equipped with LiDAR and a custom transport basket.
2.  **KUKA LBR iiwa (x2)**: Two 7-DOF redundant manipulators for pick-and-place tasks.

The system performs a complete **Pick-Transport-Sort** cycle autonomously.

###  The Workflow
1.  **Loading (Station 1)**: `iiwa_1` picks a package and loads it into the Fra2mo basket.
2.  **Autonomous Navigation**: Fra2mo navigates the warehouse using **Nav2** to reach the sorting station.
3.  **Smart Sorting (Station 2)**: `iiwa_2` uses an **Eye-in-Hand camera** to detect ArUco tags. Based on the ID (1 or 2), it sorts the package into the designated box.

---

##  System Architecture

The architecture relies on **ROS 2 Composition** and modular namespaces to handle multiple robots simultaneously.

* **Navigation Stack**: Uses `Nav2` with AMCL localization and `slam_toolbox` mapping.
* **Manipulation**: Custom C++ nodes utilizing **KDL (Kinematics and Dynamics Library)** for inverse kinematics and trajectory generation.
* **Perception**: `aruco_ros` for marker detection and pose estimation.

### Build
Clone the repository into your workspace and build:

```
cd ~/ros2_ws/src
git clone https://github.com/aldetor00/final_project_backup.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Usage
To run the full simulation, you need to open multiple terminals to launch the different subsystems.
1. Launch Simulation Environment
Starts Gazebo, spawns the robots (Fra2mo + 2 iiwa arms), and loads the warehouse world.

```
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
```
2. Launch Navigation Stack (Nav2)
Starts the AMCL localization, costmaps, and the navigation lifecycle manager for Fra2mo.

```
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```
3. Launch Manipulation Server
Starts the Kinematics Server (ros2_kdl_node) which listens for Cartesian goals and computes joint trajectories
```
ros2 launch ros2_kdl_package launching.launch.py
```
4. Execute the Mission
Once everything is running, launch the logic nodes to start the autonomous cycle.

Terminal A: Start Mobile Navigation Logic This node sends waypoints to the Nav2 action server to move Fra2mo between stations.
```
ros2 run ros2_fra2mo follow_waypoints.py
```
Terminal B: Start Manipulation Clients Run the pick & place logic. Note: You must run these in separate tabs.

Loader Arm
```
ros2 run ros2_kdl_package iiwa1_pick_place_client
```
Sorter Arm
```
ros2 run ros2_kdl_package dual_marker_client
```
