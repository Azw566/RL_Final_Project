# 🤖🍹 Robotic Bar Scenario – Robotics Lab 2025

**Author:** Antonio Polito (Matr: P38000330)

---

## 📖 Project Overview

This repository contains the final technical project for the **Robotics Lab 2025** course.  
The system simulates a **multi-robot collaborative environment** — the *Robotic Bar* — built using **ROS 2** and **Gazebo**.

The robotic setup includes:

1. **🚗 Mobile Waiter – Fra2Mo**  
   A differential-drive robot equipped with LiDAR and an RGB camera for autonomous navigation, mapping, and docking.

2. **🦾 Bartender – KUKA iiwa**  
   A 7-DOF industrial manipulator used for object handling at the bar station.

The goal is to perform a complete robotic service workflow: autonomous exploration of an unknown environment and coordinated docking between the two robots for object exchange using ArUco markers.

---

## 📦 Getting Started

Clone the repository inside your ROS 2 workspace, build it, and source the environment:

```shell
cd ~/ros2_ws
git clone https://github.com/P0l1702/RL2025TechnicalProject.git
colcon build
source install/setup.bash
```

## 🗺️ Exploration & Mapping

### 🔍 Exploration

To start autonomous exploration of the unknown environment:

```shell
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```
After the exploration is complete, save the generated map:
```shell
cd ~/ros2_ws/src/ros2_fra2mo/maps/
ros2 run nav2_map_server map_saver_cli -f map
```
### 🎥 Video
[![Watch the video](https://img.youtube.com/vi/j3ZU4Mi3NBg/maxresdefault.jpg)](https://www.youtube.com/watch?v=j3ZU4Mi3NBg)

## ▶️ Running the Simulation

Launch the full Robotic Bar scenario in Gazebo, including the KUKA iiwa manipulator and all controllers:
```shell
ros2 launch robotic_bar_package robotic_bar.launch.py
```

### 🍹 Running the Robotic Bar Logic
To start the delivery and coordination pipeline between the robots:
```shell
ros2 launch robotic_bar_package delivery.launch.py
```
### 🖱️ Start the Interface
To open user interface:
```shell
ros2 run robotic_bar_package bar_button.py --ros-args -p use_sim_time:=true
```

### 🎥 Video
[![Watch the video](https://img.youtube.com/vi/wgDxlO-bMEQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=wgDxlO-bMEQ)

---

## 📂 Repository Structure

```text
├── robotic_bar_package/    # Main logic nodes (Brains), Launch files, World
├── ros2_fra2mo/            # Mobile robot description, Nav2 config, SLAM
├── ros2_iiwa/              # KUKA manipulator description and control
├── aruco_ros/              # Vision and marker detection packages
└── m-explore-ros2/         # Autonomous frontier exploration
```# RL_Final_Project
