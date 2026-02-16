# PX4 Drone Mapping Package

This package converts the original ground robot exploration system to use PX4 drones for aerial mapping using the same frontier-based exploration algorithm.

## Overview

The package provides:
- **PX4 Integration**: Uses PX4 SITL with uXRCE-DDS bridge for drone control
- **Frontier Exploration**: Same m-explore algorithm adapted for aerial mapping
- **Nav2 Integration**: Nav2 for path planning, bridged to PX4 TrajectorySetpoint messages
- **SLAM**: SLAM Toolbox for mapping at fixed altitude

## Architecture

The system follows the architecture described in `architecture.md`:

```
┌─────────────────────────────────────────┐
│         Exploration Algorithm           │
│         (m-explore-ros2)                │
│  - Finds frontiers in occupancy grid    │
│  - Sends goals to Nav2                  │
└──────────────┬──────────────────────────┘
               │ NavigateToPose
               ▼
┌─────────────────────────────────────────┐
│           Nav2 Stack                   │
│  - Planner: computes path              │
│  - Controller: follows path            │
│  - Publishes cmd_vel                   │
└──────────────┬──────────────────────────┘
               │ cmd_vel (Twist)
               ▼
┌─────────────────────────────────────────┐
│      Nav2-PX4 Bridge                   │
│  - Converts cmd_vel to TrajectorySetpoint│
│  - Maintains fixed altitude            │
│  - Handles NED/ENU conversion         │
└──────────────┬──────────────────────────┘
               │ PX4 Messages
               ▼
┌─────────────────────────────────────────┐
│      PX4 SITL (via uXRCE-DDS)          │
│  - Offboard control mode               │
│  - Position/velocity control            │
└─────────────────────────────────────────┘
```

## Prerequisites

1. **ROS 2 Humble** (or compatible)
2. **PX4 Autopilot** (v1.15+)
   ```bash
   git clone https://github.com/PX4/PX4-Autopilot
   cd PX4-Autopilot
   make px4_sitl
   ```
3. **MicroXRCE-DDS Agent**
   ```bash
   sudo apt install ros-humble-micro-xrce-dds-agent
   ```
4. **px4_msgs** (matching your PX4 version)
   ```bash
   git clone https://github.com/PX4/px4_msgs
   ```
5. **Nav2** and **SLAM Toolbox**
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-slam-toolbox
   ```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_px4_drone
source install/setup.bash
```

## Running

### 1. Start PX4 SITL

In a terminal:
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4 -i 0
```

### 2. Start uXRCE-DDS Agent

In another terminal:
```bash
MicroXRCEAgent udp4 -p 8888 -n px4_drone
```

### 3. Launch Exploration

In another terminal:
```bash
ros2 launch ros2_px4_drone px4_drone_explore.launch.py
```

This will:
- Start SLAM Toolbox for mapping
- Launch Nav2 stack for navigation
- Start the exploration algorithm
- Bridge Nav2 commands to PX4

## Configuration

### Altitude Control

Edit `config/explore.yaml` or pass parameters:
```bash
ros2 launch ros2_px4_drone px4_drone_explore.launch.py target_altitude:=3.0
```

### Exploration Parameters

The exploration algorithm parameters can be adjusted in `config/explore.yaml`:
- `planner_frequency`: How often to plan new goals (Hz)
- `progress_timeout`: Timeout before blacklisting a frontier (seconds)
- `min_frontier_size`: Minimum frontier size to explore (meters)

### Nav2 Parameters

Nav2 parameters are in `config/navigation.yaml`:
- `desired_linear_vel`: Maximum horizontal velocity (m/s)
- `xy_goal_tolerance`: Goal tolerance (meters)

## Key Differences from Ground Robot

1. **Fixed Altitude**: The drone maintains a fixed altitude (default 2m) while exploring
2. **3D to 2D Mapping**: The LiDAR scans are projected to a 2D occupancy grid at the flight altitude
3. **PX4 Control**: Uses PX4's offboard mode instead of direct velocity commands
4. **Frame Conversion**: Handles NED (PX4) to ENU (ROS) coordinate conversions

## Nodes

### `nav2_px4_bridge`
Bridges Nav2's `cmd_vel` messages to PX4 `TrajectorySetpoint` messages. Also maintains altitude control.

### `px4_drone_controller`
Alternative controller that can receive direct pose goals (for future use with mission planner).

### `dynamic_tf_publisher`
Publishes TF transforms from PX4 odometry.

## Troubleshooting

### PX4 not entering offboard mode
- Ensure the bridge is publishing `OffboardControlMode` at ≥10 Hz
- Check that PX4 is armed: `ros2 topic echo /px4_drone/fmu/out/vehicle_status`

### Drone not maintaining altitude
- Check altitude controller parameters in `nav2_px4_bridge`
- Verify LiDAR is providing valid scans

### Exploration not starting
- Ensure SLAM is running and publishing a map
- Check that frontiers are being detected: `ros2 topic echo /explore/frontiers`

## Future Enhancements

- Multi-drone coordination (as per architecture.md)
- ArUco tag detection from downward camera
- 3D mapping with OctoMap
- Integration with mission planner for multi-agent scenarios
