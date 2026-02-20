# maze_ws — Leader-Follower Maze Exploration System

**ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL**

Two PX4 drones (leaders) autonomously map a maze using 2D LiDAR + `slam_toolbox`
and frontier exploration via `m-explore-ros2`. A TurtleBot 4 (follower) receives
the merged map and navigates to detected ArUco tags using Nav2 + AMCL.

---

## Workspace layout

```
maze_ws/
├── src/
│   ├── maze_msgs/          Custom messages (ArucoDetection, ArucoDetectionArray)
│   ├── maze_mission/       All Python nodes + configs + launch files
│   ├── maze_gazebo/        Gazebo world SDF + launch
│   ├── maze_nav2/          Standalone Nav2 config for TurtleBot
│   ├── px4_msgs/           ← git clone https://github.com/PX4/px4_msgs.git
│   ├── m-explore-ros2/     ← git clone https://github.com/robo-friends/m-explore-ros2.git
│   └── PX4-ROS2-SLAM-Control/ ← git clone https://github.com/ainolf/PX4-ROS2-SLAM-Control.git
├── scripts/
│   ├── launch_px4_sitl.sh  Launches 2 PX4 SITL + XRCE-DDS agents
│   └── generate_aruco_tags.py  Generates ArUco PNGs for Gazebo
└── README.md
```

---

## Quick start

### 1. Install dependencies

```bash
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-nav2-amcl
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-navigation
sudo apt install python3-opencv
```

### 2. Clone external packages

```bash
cd ~/maze_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ainolf/PX4-ROS2-SLAM-Control.git
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### 3. Build PX4 SITL

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_lidar_2d
```

### 4. Generate ArUco tag textures

```bash
cd ~/maze_ws
python3 scripts/generate_aruco_tags.py --count 5
```

### 5. Build the workspace

```bash
cd ~/maze_ws
colcon build --symlink-install
source install/setup.bash
```

### 6. Launch

**Terminal 1 — Gazebo world:**
```bash
ros2 launch maze_gazebo gazebo.launch.py
```

**Terminal 2 — PX4 SITL + XRCE-DDS agents:**
```bash
bash ~/maze_ws/scripts/launch_px4_sitl.sh
```

**Terminal 3 — Full mission (map merge + drones + TurtleBot + mission planner):**
```bash
ros2 launch maze_mission sim_full.launch.py
```

Or launch components individually (see Section 10 of `architecture_leader_follower.md`).

---

## Key topics

| Topic | Type | Description |
|---|---|---|
| `/drone1/map` | OccupancyGrid | Drone 1 SLAM map (ENU) |
| `/drone2/map` | OccupancyGrid | Drone 2 SLAM map (ENU) |
| `/map` | OccupancyGrid | Merged map (from map_merge) |
| `/aruco/detections` | ArucoDetectionArray | Detected ArUco tags (map frame) |
| `/mission/turtlebot_goal` | PoseStamped | Goal for TurtleBot |
| `/mission/status` | String | Mission state machine status |

---

## Development roadmap

See `~/architecture_leader_follower.md` Section 11 for the 7-phase roadmap.
