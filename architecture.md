# Multi-Agent Maze Exploration & ArUco Retrieval — System Architecture

## 1. System Overview

Three PX4 vehicles coordinated by a centralized ROS 2 mission planner:

| Agent | Role | PX4 Frame | Gazebo Model |
|-------|------|-----------|--------------|
| **Drone 1** | Aerial scout — explores maze, detects ArUco tags from above | `mc_quad` (iris) | `x500_depth` (with depth camera) |
| **Drone 2** | Aerial scout — same role, covers different zone | `mc_quad` (iris) | `x500_depth` |
| **Rover** | Ground retriever — navigates maze corridors to physically reach ArUco tags | `rover` (r1_rover) | `r1_rover` |

**Key design principle:** all three agents run PX4 firmware and communicate via the same uXRCE-DDS → ROS 2 bridge. This means one unified interface for offboard control, no custom bridges needed.

---

## 2. Communication Stack

```
┌──────────────────────────────────────────────────────┐
│                  ROS 2 Humble (DDS)                  │
│                                                      │
│  Mission Planner ◄──► Nav2 (rover) ◄──► ArUco Node  │
│        │                                     │       │
│        ▼                                     ▼       │
│   Shared Map                          OpenCV ArUco   │
│  (OccupancyGrid)                      Detections     │
└──────┬───────────────┬───────────────┬───────────────┘
       │               │               │
   ┌───▼───┐       ┌───▼───┐       ┌───▼───┐
   │ XRCE  │       │ XRCE  │       │ XRCE  │
   │ Agent │       │ Agent │       │ Agent │
   └───┬───┘       └───┬───┘       └───┬───┘
       │               │               │
   ┌───▼───┐       ┌───▼───┐       ┌───▼───┐
   │ PX4   │       │ PX4   │       │ PX4   │
   │SITL 1 │       │SITL 2 │       │SITL 3 │
   │Drone1 │       │Drone2 │       │ Rover │
   └───────┘       └───────┘       └───────┘
```

### How it works

**uXRCE-DDS** is PX4's native micro-ROS bridge. Each PX4 SITL instance runs a `MicroXRCEAgent` that exposes PX4 internal topics (position, velocity, sensor data) as ROS 2 topics. This is the **officially recommended** way to interface PX4 with ROS 2 — MAVLink/MAVROS is the legacy approach.

Each SITL instance gets a unique **namespace** (`/drone1`, `/drone2`, `/rover`) so their topics don't collide. The XRCE agent handles this via the `-n` (namespace) flag.

### Multi-Instance SITL Setup

Each PX4 instance needs a unique set of ports. PX4 SITL uses three ports per instance:

```bash
# Terminal 1 — Drone 1 (default ports)
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth \
  ./build/px4_sitl_default/bin/px4 -i 0

# Terminal 2 — Drone 2 (offset by 1)
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500_depth \
  ./build/px4_sitl_default/bin/px4 -i 1

# Terminal 3 — Rover (offset by 2)
PX4_SYS_AUTOSTART=4009 PX4_GZ_MODEL=r1_rover \
  ./build/px4_sitl_default/bin/px4 -i 2
```

The `-i N` flag tells PX4 to offset all its ports by `N * 2` (UDP base port starts at 14540). This is the **only** thing you need for multi-vehicle — PX4 handles the rest.

Then one XRCE agent per vehicle:

```bash
# Each agent bridges one PX4 instance to ROS 2 under a namespace
MicroXRCEAgent udp4 -p 8888 -n drone1   # bridges PX4 instance 0
MicroXRCEAgent udp4 -p 8890 -n drone2   # bridges PX4 instance 1
MicroXRCEAgent udp4 -p 8892 -n rover    # bridges PX4 instance 2
```

> **Port mapping:** PX4 instance `-i N` sends XRCE traffic to UDP port `8888 + N*2`. So instance 0 → 8888, instance 1 → 8890, instance 2 → 8892.

---

## 3. ROS 2 Node Architecture

### 3.1 Nodes Overview

```
┌─────────────────────────────────────────────────┐
│              MISSION PLANNER NODE                │
│  /mission_planner                                │
│                                                  │
│  • Holds global OccupancyGrid (merged map)       │
│  • Assigns frontier cells to drones              │
│  • Dispatches rover to detected ArUco positions  │
│  • Implements state machine (EXPLORE → RETRIEVE) │
└────┬────────────────┬───────────────────┬────────┘
     │                │                   │
     ▼                ▼                   ▼
┌─────────┐    ┌─────────┐    ┌──────────────────┐
│ DRONE   │    │ DRONE   │    │ ROVER CONTROLLER │
│ CTRL 1  │    │ CTRL 2  │    │ /rover/ctrl      │
│/drone1/ │    │/drone2/ │    │                  │
│ ctrl    │    │ ctrl    │    │ • Nav2 for path  │
│         │    │         │    │   planning in     │
│• Offboard│   │• Offboard│   │   maze corridors │
│  mode   │    │  mode   │    │ • ArUco approach │
│• Waypoint│   │• Waypoint│   │   + "grab" logic │
│  follow │    │  follow │    └──────────────────┘
└─────────┘    └─────────┘
     │                │                   │
     ▼                ▼                   ▼
┌─────────┐    ┌─────────┐    ┌──────────────────┐
│ ARUCO   │    │ ARUCO   │    │ ARUCO DETECTOR   │
│DETECTOR │    │DETECTOR │    │ /rover/aruco     │
│/drone1/ │    │/drone2/ │    │                  │
│ aruco   │    │ aruco   │    │ • Front camera   │
│         │    │         │    │ • Close-range    │
│• Down   │    │• Down   │    │   confirmation   │
│  camera │    │  camera │    └──────────────────┘
│• Tag ID │    │• Tag ID │
│  + pose │    │  + pose │
└─────────┘    └─────────┘
```

### 3.2 Key ROS 2 Topics

```yaml
# ---- PX4 ↔ ROS 2 (auto-generated by uXRCE-DDS) ----
/<ns>/fmu/in/offboard_control_mode    # OffboardControlMode
/<ns>/fmu/in/trajectory_setpoint      # TrajectorySetpoint
/<ns>/fmu/in/vehicle_command          # VehicleCommand (arm, mode switch)
/<ns>/fmu/out/vehicle_local_position  # VehicleLocalPosition (NED frame)
/<ns>/fmu/out/vehicle_status          # VehicleStatus (armed, mode)
/<ns>/fmu/out/sensor_combined         # IMU data

# ---- Custom application topics ----
/map/occupancy_grid          # nav_msgs/OccupancyGrid — merged global map
/aruco/detections            # custom_msgs/ArucoDetectionArray — all detected tags
/mission/drone_assignment    # custom_msgs/DroneAssignment — frontier targets
/mission/rover_goal          # geometry_msgs/PoseStamped — where rover should go
/mission/status              # custom_msgs/MissionStatus — state machine state
```

### 3.3 Custom Message Definitions

You'll need a `maze_msgs` package with:

```
# ArucoDetection.msg
uint32 tag_id
geometry_msgs/PoseStamped pose_in_map    # tag position in map frame
string detected_by                        # "drone1" or "drone2"
float32 confidence                        # detection confidence [0-1]
bool retrieved                            # has rover picked it up?

# ArucoDetectionArray.msg
std_msgs/Header header
ArucoDetection[] detections

# DroneAssignment.msg
string drone_ns                           # "drone1" or "drone2"
geometry_msgs/Point[] waypoints           # frontier cells to explore
float32 altitude                          # exploration altitude (m)
```

---

## 4. Exploration Strategy

### 4.1 Frontier-Based Exploration (Drones)

The drones perform **frontier-based exploration**: they fly at a fixed altitude above the maze and use their downward-facing depth camera to build an occupancy grid of the maze layout. "Frontiers" are the boundaries between known-free and unknown cells.

**Algorithm (runs in Mission Planner):**

```
1. Merge occupancy grids from both drones into global map
2. Extract frontier cells (known-free neighbors of unknown cells)
3. Cluster frontiers into frontier regions
4. Assign each drone the nearest unassigned frontier region
   (use Hungarian algorithm or greedy nearest-neighbor)
5. Drone flies to centroid of assigned frontier at fixed altitude
6. Repeat until no frontiers remain (maze fully mapped)
```

Why this works: the maze is bounded and has finite area, so frontier-based exploration is guaranteed to converge. Two drones roughly halve the exploration time since they cover non-overlapping zones.

### 4.2 ArUco Detection Pipeline

Each drone runs an ArUco detector node that:

1. Subscribes to the downward camera: `/<ns>/camera/image_raw`
2. Runs `cv2.aruco.detectMarkers()` with the appropriate dictionary (e.g., `DICT_4X4_50`)
3. Uses `cv2.aruco.estimatePoseSingleMarkers()` + camera intrinsics to get the tag pose relative to the camera
4. Transforms camera-frame pose → map-frame pose using the drone's known position (`/vehicle_local_position`) and the TF tree
5. Publishes to `/aruco/detections` with tag ID, map-frame pose, and confidence

**Critical detail:** the camera-to-body transform must be calibrated and published to TF. In Gazebo this is defined in the SDF model — you extract it and publish a static transform.

### 4.3 Rover Retrieval

When a tag is detected:

1. Mission Planner receives the detection on `/aruco/detections`
2. It checks if the tag is new (not already retrieved)
3. It plans a path from the rover's current position to the tag's map-frame position **through the maze corridors** (the global occupancy grid is used for this)
4. It publishes the goal to Nav2 via `/rover/navigate_to_pose`
5. Nav2 handles local obstacle avoidance and path following
6. Once the rover is within a threshold distance of the tag, the rover's front-facing ArUco detector confirms the tag visually
7. The "grab" action is triggered (this depends on your hardware — in sim you can just mark it as retrieved)

---

## 5. Nav2 Configuration for the Rover

The rover uses **Nav2** (Navigation 2) for autonomous navigation inside the maze. Nav2 needs the global occupancy grid as its costmap.

### Why Nav2 and not just PX4 waypoints?

PX4's rover mode can follow GPS waypoints, but inside a maze you need:
- **Path planning** around walls (Nav2's NavFn or Smac planner)
- **Local obstacle avoidance** (Nav2's DWB or MPPI controller)
- **Recovery behaviors** (spin, back up, wait)

PX4 still handles low-level motor control. Nav2 sends `cmd_vel` → a bridge node converts that to PX4 `TrajectorySetpoint` messages.

### Nav2 ↔ PX4 Bridge

```python
"""
Minimal bridge: subscribes to Nav2's /cmd_vel (Twist)
and publishes PX4 TrajectorySetpoint.

This is the glue between Nav2's velocity commands
and PX4's offboard control interface.
"""

class Nav2PX4Bridge(Node):
    def __init__(self):
        super().__init__('nav2_px4_bridge')

        # Nav2 publishes Twist on /cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/rover/cmd_vel', self.cmd_vel_callback, 10)

        # PX4 expects TrajectorySetpoint in NED frame
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/rover/fmu/in/trajectory_setpoint', 10)

        # Must continuously publish OffboardControlMode
        # to keep PX4 in offboard mode
        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            '/rover/fmu/in/offboard_control_mode', 10)

        self.timer = self.create_timer(0.1, self.publish_mode)

    def cmd_vel_callback(self, msg: Twist):
        sp = TrajectorySetpoint()
        # Nav2 Twist.linear.x → forward velocity (NED X)
        # Nav2 Twist.angular.z → yaw rate
        sp.velocity[0] = msg.linear.x   # NED X (forward)
        sp.velocity[1] = 0.0            # NED Y (lateral)
        sp.velocity[2] = 0.0            # NED Z (down, 0 for rover)
        sp.yawspeed = msg.angular.z
        self.setpoint_pub.publish(sp)

    def publish_mode(self):
        mode = OffboardControlMode()
        mode.velocity = True       # we're controlling velocity
        mode.position = False
        mode.acceleration = False
        self.mode_pub.publish(mode)
```

This pattern is reusable: the drone controller nodes do the same thing but with 3D position/velocity setpoints.

---

## 6. TF Tree

```
map
 └── odom (per vehicle, from PX4 EKF2)
      └── base_link (vehicle body frame)
           ├── camera_link (downward cam on drones, front cam on rover)
           └── lidar_link (if using lidar for rover local avoidance)
```

PX4 publishes `odom → base_link` via the XRCE bridge (from EKF2 state estimate). You publish `map → odom` using a localization node — in simulation this can be identity since PX4 SITL gives perfect-ish odometry, but for real hardware you'd use AMCL or similar.

---

## 7. Gazebo World Setup

### 7.1 Maze World

You already have a mesh. To integrate it:

```xml
<!-- maze_world.sdf -->
<sdf version="1.9">
  <world name="maze">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>

    <!-- Your maze mesh -->
    <model name="maze">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh><uri>model://maze/meshes/maze.dae</uri></mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh><uri>model://maze/meshes/maze.dae</uri></mesh>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- ArUco tags as small textured planes placed in the maze -->
    <!-- Generate these programmatically — see Section 7.2 -->
  </world>
</sdf>
```

### 7.2 ArUco Tags in Gazebo

ArUco tags are small flat textured planes. For each tag:

1. Generate the tag image: `cv2.aruco.generateImageMarker(dictionary, tag_id, size)`
2. Save as PNG (e.g., `aruco_0.png`)
3. Create a simple SDF model with a textured plane:

```xml
<model name="aruco_tag_0">
  <static>true</static>
  <pose>3.5 2.1 0.01 0 0 0</pose>  <!-- slightly above ground -->
  <link name="link">
    <visual name="visual">
      <geometry><plane><normal>0 0 1</normal><size>0.15 0.15</size></plane></geometry>
      <material>
        <diffuse>1 1 1 1</diffuse>
        <pbr><metal><albedo_map>model://aruco_tags/textures/aruco_0.png</albedo_map></metal></pbr>
      </material>
    </visual>
  </link>
</model>
```

Place 5-10 of these at known positions inside the maze corridors.

---

## 8. Project Structure

```
maze_ws/
├── src/
│   ├── maze_msgs/                    # Custom message definitions
│   │   ├── msg/
│   │   │   ├── ArucoDetection.msg
│   │   │   ├── ArucoDetectionArray.msg
│   │   │   └── DroneAssignment.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── maze_mission/                 # Core mission logic (Python)
│   │   ├── maze_mission/
│   │   │   ├── __init__.py
│   │   │   ├── mission_planner.py    # Central coordinator + state machine
│   │   │   ├── drone_controller.py   # Offboard control for one drone
│   │   │   ├── rover_controller.py   # Nav2 ↔ PX4 bridge + retrieval logic
│   │   │   ├── aruco_detector.py     # OpenCV ArUco detection node
│   │   │   ├── map_merger.py         # Merges per-drone maps into global map
│   │   │   └── frontier_explorer.py  # Frontier extraction + assignment
│   │   ├── config/
│   │   │   ├── drone_params.yaml     # Altitude, speed, camera intrinsics
│   │   │   ├── rover_params.yaml     # Nav2 config, approach distance
│   │   │   └── aruco_params.yaml     # Dictionary type, tag size
│   │   ├── launch/
│   │   │   ├── sim_full.launch.py    # Launches everything for simulation
│   │   │   ├── drones.launch.py      # Just the 2 drones + their nodes
│   │   │   └── rover.launch.py       # Just the rover + Nav2
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── maze_gazebo/                  # Gazebo world + models
│   │   ├── worlds/
│   │   │   └── maze.sdf
│   │   ├── models/
│   │   │   ├── maze/meshes/maze.dae
│   │   │   └── aruco_tags/           # Generated tag models
│   │   ├── launch/
│   │   │   └── gazebo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── maze_nav2/                    # Nav2 config for rover
│       ├── config/
│       │   ├── nav2_params.yaml      # Planner, controller, costmap config
│       │   └── costmap_params.yaml
│       ├── launch/
│       │   └── nav2.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
│
├── scripts/
│   ├── generate_aruco_tags.py        # Generate tag PNGs + SDF models
│   └── launch_px4_sitl.sh            # Start 3 PX4 SITL instances
│
└── README.md
```

---

## 9. Mission State Machine

The Mission Planner runs a state machine:

```
                    ┌──────────┐
                    │   INIT   │
                    │ Wait for │
                    │ all 3 to │
                    │   arm    │
                    └────┬─────┘
                         │ all armed
                         ▼
                    ┌──────────┐
              ┌────►│ EXPLORE  │◄────────────────┐
              │     │          │                  │
              │     │ Drones   │                  │
              │     │ fly to   │                  │
              │     │ frontiers│                  │
              │     └────┬─────┘                  │
              │          │ tag detected           │
              │          ▼                        │
              │     ┌──────────┐                  │
              │     │ DISPATCH │                  │
              │     │          │                  │
              │     │ Send     │                  │
              │     │ rover to │                  │
              │     │ tag pose │                  │
              │     └────┬─────┘                  │
              │          │ rover arrives          │
              │          ▼                        │
              │     ┌──────────┐    more tags     │
              │     │ RETRIEVE │──────────────────┘
              │     │          │
              │     │ Confirm  │
              │     │ tag,     │
              │     │ "grab"   │
              │     └────┬─────┘
              │          │ all tags retrieved
              │          │ AND no frontiers left
              │          ▼
              │     ┌──────────┐
              │     │ COMPLETE │
              │     │          │
              │     │ RTL all  │
              │     │ vehicles │
              │     └──────────┘
              │
              │ exploration not done
              └──────────────────────
```

**Concurrency note:** the drones keep exploring even while the rover is retrieving. The state machine is per-rover, while exploration runs continuously. Think of it as two parallel threads: `explore_loop` (always running) and `retrieve_loop` (triggered by detections, uses a queue).

---

## 10. Key Dependencies & Versions

| Component | Version | Install |
|-----------|---------|---------|
| ROS 2 | Humble | `apt install ros-humble-desktop` |
| PX4 Autopilot | v1.15+ | `git clone https://github.com/PX4/PX4-Autopilot` |
| Gazebo | Harmonic (gz-sim 8) | Ships with PX4 v1.15 |
| Nav2 | Humble | `apt install ros-humble-navigation2` |
| XRCE-DDS Agent | 2.x | `apt install ros-humble-micro-xrce-dds-agent` (or build from source) |
| OpenCV + ArUco | 4.x | `pip install opencv-contrib-python` |
| px4_msgs | match PX4 ver | `git clone https://github.com/PX4/px4_msgs` (branch matching your PX4) |

---

## 11. Launch Order (Simulation)

```bash
# 1. Start Gazebo with maze world
ros2 launch maze_gazebo gazebo.launch.py

# 2. Start 3 PX4 SITL instances (script runs them in background)
bash scripts/launch_px4_sitl.sh

# 3. Start 3 XRCE agents (one per vehicle)
MicroXRCEAgent udp4 -p 8888 -n drone1 &
MicroXRCEAgent udp4 -p 8890 -n drone2 &
MicroXRCEAgent udp4 -p 8892 -n rover &

# 4. Launch drone nodes (controllers + ArUco detectors)
ros2 launch maze_mission drones.launch.py

# 5. Launch rover nodes (Nav2 + controller + bridge)
ros2 launch maze_mission rover.launch.py

# 6. Launch mission planner (starts the state machine)
ros2 launch maze_mission mission_planner.launch.py
```

---

## 12. Development Roadmap

### Phase 1 — Single Drone Flies in Maze (Week 1-2)
- [ ] Set up PX4 SITL + Gazebo with maze world
- [ ] Spawn one `x500_depth` drone
- [ ] Write `drone_controller.py`: arm → takeoff → offboard mode → fly to a hardcoded waypoint
- [ ] Verify camera feed in RViz2

### Phase 2 — ArUco Detection (Week 2-3)
- [ ] Place ArUco tag models in Gazebo maze
- [ ] Write `aruco_detector.py`: subscribe to camera, detect tags, publish detections
- [ ] Visualize detections in RViz2 as markers

### Phase 3 — Rover Navigation (Week 3-4)
- [ ] Spawn rover in Gazebo
- [ ] Configure Nav2 with the maze occupancy grid as static map
- [ ] Write `nav2_px4_bridge.py`
- [ ] Send rover to a hardcoded goal, verify it navigates corridors

### Phase 4 — Two Drones + Frontier Exploration (Week 4-5)
- [ ] Multi-SITL with 2 drones
- [ ] Implement `frontier_explorer.py` and `map_merger.py`
- [ ] Verify drones explore non-overlapping zones

### Phase 5 — Full Integration (Week 5-6)
- [ ] Implement `mission_planner.py` state machine
- [ ] Connect detection → dispatch → retrieval pipeline
- [ ] End-to-end test: drones explore, find tags, rover retrieves them

---

## 13. Common Pitfalls

**PX4 Offboard timeout:** PX4 requires you to publish `OffboardControlMode` at ≥2 Hz *before* switching to offboard mode, and continuously after. If you stop publishing for >500ms, PX4 will failsafe. Always use a timer-based publisher.

**NED vs ENU:** PX4 uses NED (North-East-Down) internally. ROS 2 convention is ENU (East-North-Up). The XRCE bridge does **not** convert between them — you must handle this in your nodes. Key conversion: `x_enu = y_ned`, `y_enu = x_ned`, `z_enu = -z_ned`.

**Namespace collisions:** make sure every topic, TF frame, and node name is namespaced per vehicle. The XRCE `-n` flag handles PX4 topics, but your custom nodes need explicit namespacing in the launch file.

**Nav2 costmap source:** for the rover, the global costmap should come from the merged occupancy grid (published by Mission Planner), not from the rover's own sensors alone. Configure the global costmap to subscribe to `/map/occupancy_grid` as a static layer.

**ArUco pose accuracy from altitude:** at 3-5m altitude with a standard camera, ArUco tag localization error is typically ±20-30cm. This is fine for dispatching the rover to the general area — the rover's front-facing camera handles precision approach.
