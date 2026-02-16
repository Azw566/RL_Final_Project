# Conversion Notes: Ground Robot to PX4 Drone Mapping

## Summary

This document describes the conversion of the original ground robot (`ros2_fra2mo`) exploration system to use PX4 drones for aerial mapping while preserving the same exploration algorithm.

## Key Changes

### 1. Robot Model
- **Before**: Custom differential-drive robot (`fra2mo`) with Gazebo plugins
- **After**: PX4 quadcopter model (`x500_depth`) using PX4 SITL with uXRCE-DDS bridge

### 2. Control Interface
- **Before**: Direct `cmd_vel` commands via Gazebo diff-drive plugin
- **After**: PX4 `TrajectorySetpoint` messages via uXRCE-DDS bridge
- **Bridge**: New `nav2_px4_bridge` node converts Nav2's `cmd_vel` to PX4 messages

### 3. Coordinate Frames
- **Before**: Standard ROS ENU frames
- **After**: PX4 uses NED internally, conversion handled in bridge
- **TF Tree**: `map → px4_drone/odom → base_footprint`

### 4. Altitude Control
- **Before**: Not applicable (ground robot)
- **After**: Fixed altitude maintained by PID controller in bridge node

### 5. Navigation Parameters
- **Before**: Optimized for ground robot (lower speeds, tighter tolerances)
- **After**: Adapted for aerial navigation (higher speeds, larger tolerances)

## Preserved Components

### Exploration Algorithm
- **Unchanged**: Same `m-explore-ros2` package and algorithm
- **Frontier Detection**: Works identically on 2D occupancy grid
- **Goal Planning**: Same frontier selection and cost calculation

### SLAM
- **Unchanged**: Same SLAM Toolbox configuration
- **Adaptation**: LiDAR scans projected to 2D at flight altitude

### Nav2 Stack
- **Unchanged**: Same Nav2 components (planner, controller, costmaps)
- **Adaptation**: Parameters tuned for aerial navigation

## File Mapping

| Original (fra2mo) | New (px4_drone) | Changes |
|-------------------|-----------------|---------|
| `fra2mo.urdf.xacro` | Not needed | PX4 model handled by SITL |
| `gazebo_fra2mo.launch.py` | Not needed | PX4 SITL launched separately |
| `fra2mo_slam.launch.py` | `px4_drone_slam.launch.py` | Namespace changed |
| `fra2mo_explore.launch.py` | `px4_drone_explore.launch.py` | Added bridge node |
| `config/explore.yaml` | `config/explore.yaml` | Namespace + Nav2 params |
| `config/slam.yaml` | `config/slam.yaml` | Namespace + topic names |
| `src/odom_bl_tf.cpp` | `src/odom_bl_tf.cpp` | Namespace updated |
| N/A | `src/nav2_px4_bridge.cpp` | **NEW**: Bridge node |
| N/A | `src/px4_drone_controller.cpp` | **NEW**: Direct controller |

## Usage Comparison

### Original (Ground Robot)
```bash
# Single launch file starts everything
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
```

### New (PX4 Drone)
```bash
# Terminal 1: Start PX4 SITL
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth \
  ./build/px4_sitl_default/bin/px4 -i 0

# Terminal 2: Start uXRCE-DDS bridge
MicroXRCEAgent udp4 -p 8888 -n px4_drone

# Terminal 3: Launch exploration
ros2 launch ros2_px4_drone px4_drone_explore.launch.py
```

## Architecture Alignment

The conversion follows the architecture described in `architecture.md`:

1. ✅ Uses PX4 SITL with uXRCE-DDS bridge
2. ✅ Maintains same exploration algorithm
3. ✅ Nav2 for path planning (2D at fixed altitude)
4. ✅ Bridge converts Nav2 commands to PX4 messages
5. ✅ Fixed altitude exploration

## Future Enhancements

Based on `architecture.md`, future work could include:

1. **Multi-Drone Support**: Launch multiple PX4 instances with different namespaces
2. **Mission Planner**: Central coordinator for multi-agent scenarios
3. **ArUco Detection**: Downward camera for tag detection
4. **Map Merging**: Combine maps from multiple drones
5. **3D Mapping**: Use OctoMap for full 3D representation

## Testing Checklist

- [ ] PX4 SITL launches successfully
- [ ] uXRCE-DDS bridge connects
- [ ] Drone arms and enters offboard mode
- [ ] Altitude controller maintains fixed altitude
- [ ] SLAM generates map from LiDAR
- [ ] Nav2 plans paths correctly
- [ ] Exploration algorithm finds frontiers
- [ ] Drone navigates to frontier goals
- [ ] Map saves correctly after exploration

## Known Limitations

1. **2D Mapping Only**: Currently projects 3D LiDAR to 2D occupancy grid
2. **Single Drone**: Multi-drone coordination not yet implemented
3. **Fixed Altitude**: Altitude must be set before launch (no dynamic adjustment)
4. **No ArUco Detection**: Tag detection not yet integrated

## Dependencies

New dependencies added:
- `px4_msgs`: PX4 message definitions
- `micro-xrce-dds-agent`: uXRCE-DDS bridge
- PX4 Autopilot: For SITL simulation

Removed dependencies:
- Gazebo-specific plugins (handled by PX4)
