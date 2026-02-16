# Adapted Architecture with m-explore-ros2

## 1. What m-explore-ros2 Gives Us (and What It Doesn't)

The `m-explore-ros2` repo provides two ROS 2 packages:

| Package | What it does | Replaces |
|---------|-------------|----------|
| **explore_lite** | Frontier-based exploration: reads a costmap, finds frontier cells, picks the best one, sends a `NavigateToPose` goal to Nav2. Repeats until no frontiers remain. | Our custom `frontier_explorer.py` |
| **multirobot_map_merge** | Subscribes to `/<ns>/map` for N robots, merges them into a single `/map` (merged). Supports known initial poses (TF-based) and unknown poses (feature matching). | Our custom `map_merger.py` |

### The catch: explore_lite speaks Nav2

`explore_lite` is designed for ground robots. Internally it:
1. Reads the Nav2 global costmap (`/global_costmap/costmap`)
2. Finds frontiers on that costmap
3. Sends a `NavigateToPose` action goal to the Nav2 BT navigator
4. Waits for the robot to reach the goal
5. Picks the next frontier

For our **drones**, there's no Nav2 stack — they fly via PX4 offboard mode. So we need an **adapter layer** that makes each drone look like a Nav2 robot from explore_lite's perspective.

For the **rover**, explore_lite works natively since the rover already has a full Nav2 stack.

---

## 2. Adapter Strategy: Nav2ActionBridge

The adapter is a single node per drone called `nav2_action_bridge.py`. It implements a **fake `NavigateToPose` action server** — from explore_lite's point of view, it's talking to Nav2. But under the hood, the bridge converts goals to PX4 offboard position setpoints.

```
explore_lite                    nav2_action_bridge              PX4
    │                                  │                         │
    │  NavigateToPose goal             │                         │
    │  (x=3.2, y=5.1, z=0)            │                         │
    ├─────────────────────────────────►│                         │
    │                                  │  Convert to NED + add   │
    │                                  │  flight altitude:       │
    │                                  │  TrajectorySetpoint     │
    │                                  │  (x=5.1, y=3.2, z=-4)  │
    │                                  ├────────────────────────►│
    │                                  │                         │
    │                                  │  Monitor position       │
    │                                  │◄────────────────────────┤
    │                                  │  vehicle_local_position │
    │                                  │                         │
    │  NavigateToPose result           │  When within threshold: │
    │  (status=SUCCEEDED)              │  report succeeded       │
    │◄─────────────────────────────────┤                         │
    │                                  │                         │
    │  (picks next frontier)           │                         │
```

**Why this works:** explore_lite doesn't care *how* the robot navigates — it only cares that the `NavigateToPose` action eventually succeeds or fails. The bridge fakes the whole Nav2 stack in ~100 lines of code.

**What the bridge does concretely:**
1. Creates an action server: `NavigateToPose` (same interface Nav2 uses)
2. On goal received: extracts (x, y) from the goal pose
3. Adds the flight altitude (from config, e.g. 4.0m) → converts ENU to NED
4. Publishes `TrajectorySetpoint` to PX4 at 10 Hz
5. Publishes `OffboardControlMode` heartbeat at 10 Hz
6. Monitors `vehicle_local_position` — when drone is within threshold of goal → returns SUCCESS
7. Publishes `NavigateToPose` feedback (distance remaining) so explore_lite can track progress

---

## 3. SLAM for the Drones

`explore_lite` needs a costmap, and the costmap needs a map. For ground robots, `slam_gmapping` or `slam_toolbox` generates this map from lidar. For drones flying above a maze, we need a different approach.

### Option A: 2D projection from depth camera (recommended for sim)

Each drone has a downward-facing depth camera. A custom node `aerial_mapper.py` converts the depth image into a 2D occupancy grid by projecting the depth readings onto the ground plane:

- Pixels where depth ≈ maze_height → **occupied** (wall top)
- Pixels where depth ≈ ground_level → **free** (corridor floor)
- Unobserved pixels → **unknown**

This node publishes `/<ns>/map` (nav_msgs/OccupancyGrid) — which is exactly what `multirobot_map_merge` expects.

### Option B: slam_toolbox with simulated 2D lidar

If you add a virtual 2D lidar to the drone model (a horizontal ray fan at flight altitude), you can use `slam_toolbox` directly. This is hackier but lets you use the standard SLAM pipeline without any custom mapping code. The lidar would detect maze walls as if the drone were a ground robot at altitude.

**Recommendation:** Start with Option A. It's more physically accurate and gives you full control. Option B is a quick-and-dirty fallback.

---

## 4. Adapted Node Architecture

### What changed vs. the original architecture

```
REMOVED (replaced by m-explore-ros2):
  ✗ map_merger.py          → replaced by multirobot_map_merge
  ✗ frontier_explorer.py   → replaced by explore_lite (×2, one per drone)

ADDED (integration layer):
  ✓ nav2_action_bridge.py  → fake Nav2 for drones (×2, one per drone)
  ✓ aerial_mapper.py       → depth cam → OccupancyGrid (×2, one per drone)
  ✓ drone_costmap           → Nav2 costmap_2d node per drone (reads local map)

KEPT (unchanged):
  ✓ mission_planner.py     → central coordinator (adjusted inputs)
  ✓ drone_controller.py    → REMOVED as standalone; logic merged into nav2_action_bridge
  ✓ rover_controller.py    → rover retrieval manager
  ✓ nav2_px4_bridge.py     → Nav2 cmd_vel → PX4 for rover
  ✓ aruco_detector.py      → tag detection (×3)
```

### Full Node Graph

```
┌─────────────────────────────────────────────────────────────┐
│                    DRONE 1 (/drone1/)                       │
│                                                             │
│  Gazebo depth cam                                           │
│       │                                                     │
│       ▼                                                     │
│  aerial_mapper ──► /drone1/map (OccupancyGrid)              │
│                         │                                   │
│                         ▼                                   │
│                    costmap_2d node ──► /drone1/costmap       │
│                                            │                │
│                                            ▼                │
│                                      explore_lite           │
│                                            │                │
│                                   NavigateToPose action     │
│                                            │                │
│                                            ▼                │
│                                   nav2_action_bridge        │
│                                            │                │
│                              TrajectorySetpoint + Offboard  │
│                                            │                │
│                                            ▼                │
│                                        PX4 SITL 1           │
│                                                             │
│  aruco_detector ──────────────► /aruco/detections            │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                    DRONE 2 (/drone2/)                       │
│                                                             │
│           (identical stack, different namespace)             │
└─────────────────────────────────────────────────────────────┘

         /drone1/map ──┐
                       ├──► multirobot_map_merge ──► /map (merged)
         /drone2/map ──┘              │
                                      │
                          ┌───────────┴────────────┐
                          ▼                        ▼
                    Nav2 (rover)            mission_planner
                    global costmap          (monitors /map
                          │                 for completeness)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                     ROVER (/rover/)                         │
│                                                             │
│  mission_planner ──► /mission/rover_goal                    │
│                              │                              │
│                              ▼                              │
│                       rover_controller                      │
│                              │                              │
│                     NavigateToPose action                    │
│                              │                              │
│                              ▼                              │
│                      Nav2 (full stack)                       │
│                     planner + controller                     │
│                              │                              │
│                         /rover/cmd_vel                       │
│                              │                              │
│                              ▼                              │
│                       nav2_px4_bridge                        │
│                              │                              │
│                    TrajectorySetpoint + Offboard             │
│                              │                              │
│                              ▼                              │
│                          PX4 SITL 3                         │
│                                                             │
│  aruco_detector ──────────────► /aruco/detections            │
└─────────────────────────────────────────────────────────────┘
```

---

## 5. Adapted Folder Structure

```
maze_ws/
├── src/
│   │
│   ├── 📦 maze_msgs/                          # Custom messages (unchanged)
│   │   ├── msg/
│   │   │   ├── ArucoDetection.msg
│   │   │   └── ArucoDetectionArray.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │   # NOTE: DroneAssignment.msg REMOVED — explore_lite handles this now
│   │
│   ├── 📡 m-explore-ros2/                     # Git clone — DO NOT MODIFY
│   │   ├── explore/                            #   explore_lite package
│   │   │   ├── config/
│   │   │   │   └── params.yaml                 #   frontier exploration params
│   │   │   ├── launch/
│   │   │   │   └── explore.launch.py
│   │   │   └── src/                            #   C++ frontier search + goal sender
│   │   └── map_merge/                          #   multirobot_map_merge package
│   │       ├── config/
│   │       ├── launch/
│   │       │   ├── map_merge.launch.py
│   │       │   └── multi_tb3_simulation_launch.py  # reference, not used directly
│   │       └── src/
│   │
│   ├── 🧠 maze_mission/                       # Core mission logic
│   │   ├── maze_mission/
│   │   │   ├── __init__.py
│   │   │   ├── mission_planner.py              #   🎯 Central brain (adapted)
│   │   │   ├── nav2_action_bridge.py           #   🔌 NEW — fake Nav2 for drones
│   │   │   ├── aerial_mapper.py                #   🗺️ NEW — depth cam → OccupancyGrid
│   │   │   ├── rover_controller.py             #   🤖 Rover retrieval (unchanged)
│   │   │   ├── nav2_px4_bridge.py              #   🔌 Nav2 cmd_vel → PX4 for rover
│   │   │   └── aruco_detector.py               #   👁️ ArUco detection (unchanged)
│   │   ├── config/
│   │   │   ├── drone_explore.yaml              #   NEW — explore_lite params per drone
│   │   │   ├── drone_costmap.yaml              #   NEW — costmap_2d params per drone
│   │   │   ├── rover_nav2_params.yaml          #   Nav2 full config for rover
│   │   │   ├── map_merge_params.yaml           #   NEW — multirobot_map_merge config
│   │   │   └── aruco_params.yaml               #   dictionary type, tag size
│   │   ├── launch/
│   │   │   ├── sim_full.launch.py              #   🚀 Launches everything
│   │   │   ├── drone.launch.py                 #   1 drone: mapper+costmap+explore+bridge+aruco
│   │   │   ├── drones.launch.py                #   Calls drone.launch.py ×2 with namespaces
│   │   │   ├── rover.launch.py                 #   Rover: Nav2 + controller + bridge + aruco
│   │   │   ├── map_merge.launch.py             #   Launches multirobot_map_merge
│   │   │   └── mission.launch.py               #   Launches mission_planner
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── 🌍 maze_gazebo/                        # Simulation (unchanged)
│   │   ├── worlds/maze.sdf
│   │   ├── models/
│   │   │   ├── maze/meshes/maze.dae
│   │   │   └── aruco_tags/
│   │   ├── launch/gazebo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── 🧭 maze_nav2/                          # Nav2 config for ROVER ONLY
│       ├── config/
│       │   └── nav2_params.yaml                #   planner, controller, costmaps
│       ├── launch/
│       │   └── nav2.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
│
├── scripts/
│   ├── generate_aruco_tags.py
│   └── launch_px4_sitl.sh
│
└── README.md
```

---

## 6. File-by-File: What Changed and Why

### REMOVED FILES

#### `map_merger.py` → replaced by `multirobot_map_merge`

Our custom map merger did a simple cell-by-cell merge. `multirobot_map_merge` does the same thing but also supports **unknown initial poses** via feature matching (OpenCV ORB descriptors). It's battle-tested C++ code.

The merge node subscribes to:
```
/drone1/map    (nav_msgs/OccupancyGrid)
/drone2/map    (nav_msgs/OccupancyGrid)
```

And publishes:
```
/map           (nav_msgs/OccupancyGrid)   ← the merged global map
/map_updates   (map_msgs/OccupancyGridUpdate)
```

#### `frontier_explorer.py` → replaced by `explore_lite`

Our custom frontier extraction + assignment is no longer needed. `explore_lite` handles the full loop: find frontiers → pick best → send goal → wait → repeat.

#### `drone_controller.py` → merged into `nav2_action_bridge.py`

The standalone drone controller that took waypoints from the mission planner is gone. Its PX4 offboard logic (arm, takeoff, fly to position, heartbeat) is now inside `nav2_action_bridge.py`, which receives goals from explore_lite instead of the mission planner.

---

### NEW FILES

#### `nav2_action_bridge.py` — 🔌 Fake Nav2 for Drones

**Role:** Makes a drone look like a Nav2 robot. Implements a `NavigateToPose` action server so explore_lite can send it goals, then converts those goals to PX4 offboard commands.

**Lifecycle:**
1. On startup: sends ARM command, switches to OFFBOARD mode, takes off to configured altitude
2. Creates `NavigateToPose` action server
3. On goal received:
   - Extracts (x, y) from goal pose (explore_lite sends 2D goals)
   - Adds flight altitude → (x, y, -altitude) in NED
   - Publishes `TrajectorySetpoint` at 10 Hz targeting that position
   - Monitors `vehicle_local_position` for convergence
   - When horizontal distance < threshold (e.g., 0.5m) → return SUCCESS
   - Publishes feedback with distance remaining
4. explore_lite receives SUCCESS → picks next frontier → sends new goal

**Subscribes to:**
| Topic | From | Why |
|-------|------|-----|
| `/<ns>/fmu/out/vehicle_local_position` | PX4 | Current position to check goal reached |
| `/<ns>/fmu/out/vehicle_status` | PX4 | Confirm armed + offboard |

**Publishes to:**
| Topic | To | Why |
|-------|------|-----|
| `/<ns>/fmu/in/trajectory_setpoint` | PX4 | Position commands |
| `/<ns>/fmu/in/offboard_control_mode` | PX4 | Offboard heartbeat |
| `/<ns>/fmu/in/vehicle_command` | PX4 | ARM, mode switch |

**Action server:**
| Action | Client | Interface |
|--------|--------|-----------|
| `/<ns>/navigate_to_pose` | explore_lite | `nav2_msgs/NavigateToPose` |

**Key implementation detail:** explore_lite by default sends goals to `navigate_to_pose`. When launched under a namespace (e.g., `/drone1`), it becomes `/drone1/navigate_to_pose`. The bridge must advertise its action server on the **same namespaced topic**.

---

#### `aerial_mapper.py` — 🗺️ Depth Camera → 2D Map

**Role:** Converts the drone's downward-facing depth camera into a 2D occupancy grid suitable for explore_lite and map_merge.

**How it works:**
1. Subscribes to depth image: `/<ns>/camera/depth/image_raw`
2. Subscribes to camera info: `/<ns>/camera/depth/camera_info`
3. Gets drone pose from `/<ns>/fmu/out/vehicle_local_position`
4. For each depth pixel:
   - Projects pixel to 3D point using camera intrinsics
   - Transforms to map frame using drone pose
   - Classifies: wall (depth ≈ flight_alt - wall_height) → OCCUPIED, floor (depth ≈ flight_alt) → FREE
5. Maintains a running `OccupancyGrid` and publishes at ~2 Hz

**Subscribes to:**
| Topic | From | Why |
|-------|------|-----|
| `/<ns>/camera/depth/image_raw` | Gazebo depth cam | Raw depth data |
| `/<ns>/camera/depth/camera_info` | Gazebo depth cam | Intrinsics for projection |
| `/<ns>/fmu/out/vehicle_local_position` | PX4 | Drone pose for map-frame transform |

**Publishes to:**
| Topic | To | Why |
|-------|------|-----|
| `/<ns>/map` | multirobot_map_merge | Per-drone occupancy grid |

**Why this is needed:** `multirobot_map_merge` expects each robot to publish a standard `nav_msgs/OccupancyGrid` on `/<ns>/map`. For ground robots, `slam_toolbox` or `slam_gmapping` does this from lidar. For aerial drones, there's no standard SLAM node that generates a top-down maze map from a depth camera — so we write this custom projection.

---

### MODIFIED FILES

#### `mission_planner.py` — adapted inputs

**What changed:**
- No longer publishes `/mission/drone_assignment` (explore_lite handles drone goals)
- No longer runs frontier extraction logic
- Still subscribes to `/aruco/detections` and dispatches the rover
- Now subscribes to `/map` (from map_merge) instead of a custom merged map topic
- Uses `explore_lite`'s `/explore/resume` topic to pause/resume drone exploration
- Can pause drone exploration while rover retrieves a nearby tag (avoid interference)

**New control flow:**
```
OLD: mission_planner → frontier_explorer → drone_assignment → drone_controller
NEW: explore_lite autonomously explores → mission_planner just monitors + manages rover
```

The planner is now simpler — it's mainly a **rover dispatcher** and **tag tracker**:
1. Monitors `/aruco/detections` for new tags
2. Maintains a retrieval queue
3. Sends rover goals via `rover_controller`
4. Optionally pauses explore_lite via `/drone1/explore/resume` and `/drone2/explore/resume` if needed
5. Detects exploration complete (no more frontiers = explore_lite stops) + all tags retrieved

---

## 7. Configuration Files

### `drone_explore.yaml` — explore_lite per drone

```yaml
explore_node:
  ros__parameters:
    robot_base_frame: base_link        # drone's base frame (namespaced via TF)
    costmap_topic: global_costmap/costmap
    costmap_updates_topic: global_costmap/costmap_updates
    visualize: true                    # publish frontier markers
    planner_frequency: 0.33            # replan every 3 sec (drones are faster)
    progress_timeout: 30.0             # seconds before giving up on a goal
    potential_scale: 3.0               # weight for distance to frontier
    orientation_scale: 0.0             # drones don't care about heading during transit
    gain_scale: 1.0                    # weight for frontier information gain
    transform_tolerance: 0.5
    min_frontier_size: 0.75            # ignore tiny frontiers (meters)
    return_to_init: false              # don't return after exploration
```

**Key tuning:**
- `min_frontier_size`: set this high enough to ignore noise but low enough to catch real corridors. For a maze with 1m-wide corridors, 0.75m is a good start.
- `progress_timeout`: if the drone hasn't made progress in 30s, explore_lite will abort and pick a new frontier. Important for drones since the bridge might overshoot.
- `orientation_scale: 0.0`: drones can fly in any direction without turning, so heading cost is irrelevant.

### `drone_costmap.yaml` — costmap_2d per drone

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 2.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.3               # drone effective radius for inflation
      resolution: 0.1                  # 10cm cells
      track_unknown_space: true        # CRITICAL — explore_lite needs unknown cells
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: map                 # reads /<ns>/map from aerial_mapper
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

**Critical setting:** `track_unknown_space: true`. Without this, explore_lite treats unknown cells as free and won't find frontiers.

### `map_merge_params.yaml` — multirobot_map_merge

```yaml
map_merge:
  ros__parameters:
    robot_map_topic: map                      # each robot publishes /<ns>/map
    robot_namespace: ""                       # auto-discover from TF
    known_init_poses: true                    # we know where drones start
    merging_rate: 2.0                         # merge every 0.5s
    discovery_rate: 0.5
    estimation_rate: 0.5
    estimation_confidence: 0.6
    # Known initial poses (set per robot via TF or params):
    drone1:
      init_pose_x: 0.0
      init_pose_y: 0.0
      init_pose_z: 0.0
      init_pose_yaw: 0.0
    drone2:
      init_pose_x: 2.0                        # start offset
      init_pose_y: 0.0
      init_pose_z: 0.0
      init_pose_yaw: 0.0
```

**`known_init_poses: true`** is strongly recommended for your case. Since you control where the drones spawn in Gazebo, you know their exact initial positions. The unknown-pose mode uses feature matching which requires overlap and is less reliable.

---

## 8. Updated Topic Map

```yaml
# ═══════ PER DRONE (×2, namespaced) ═══════

# Depth camera → aerial_mapper → per-drone map
/<ns>/camera/depth/image_raw           # Gazebo → aerial_mapper
/<ns>/camera/depth/camera_info         # Gazebo → aerial_mapper
/<ns>/map                              # aerial_mapper → map_merge + costmap_2d

# Costmap for explore_lite
/<ns>/global_costmap/costmap           # costmap_2d → explore_lite
/<ns>/global_costmap/costmap_updates   # costmap_2d → explore_lite

# Exploration control
/<ns>/explore/frontiers                # explore_lite → RViz (visualization)
/<ns>/explore/resume                   # mission_planner → explore_lite (pause/resume)

# Nav2 action (fake, served by bridge)
/<ns>/navigate_to_pose                 # explore_lite → nav2_action_bridge (action)

# PX4 interface
/<ns>/fmu/in/offboard_control_mode     # nav2_action_bridge → PX4
/<ns>/fmu/in/trajectory_setpoint       # nav2_action_bridge → PX4
/<ns>/fmu/in/vehicle_command           # nav2_action_bridge → PX4
/<ns>/fmu/out/vehicle_local_position   # PX4 → nav2_action_bridge + aerial_mapper
/<ns>/fmu/out/vehicle_status           # PX4 → nav2_action_bridge

# ArUco
/<ns>/camera/image_raw                 # Gazebo → aruco_detector (downward RGB)
/aruco/detections                      # aruco_detector → mission_planner


# ═══════ GLOBAL ═══════

# Merged map
/map                                   # map_merge → Nav2 rover + mission_planner
/map_updates                           # map_merge → Nav2 rover

# Mission coordination
/mission/rover_goal                    # mission_planner → rover_controller
/mission/status                        # mission_planner → (logging/RViz)


# ═══════ ROVER ═══════

/rover/navigate_to_pose                # rover_controller → Nav2 (real action)
/rover/cmd_vel                         # Nav2 controller → nav2_px4_bridge
/rover/fmu/in/trajectory_setpoint      # nav2_px4_bridge → PX4
/rover/fmu/in/offboard_control_mode    # nav2_px4_bridge → PX4
/rover/fmu/out/vehicle_local_position  # PX4 → Nav2 (odometry)
/rover/camera/image_raw                # Gazebo → aruco_detector (front cam)
/aruco/detections                      # aruco_detector → mission_planner
```

---

## 9. Launch Order (Simulation)

```bash
# 1. Gazebo world
ros2 launch maze_gazebo gazebo.launch.py

# 2. PX4 SITL instances (3 vehicles)
bash scripts/launch_px4_sitl.sh

# 3. XRCE-DDS agents
MicroXRCEAgent udp4 -p 8888 -n drone1 &
MicroXRCEAgent udp4 -p 8890 -n drone2 &
MicroXRCEAgent udp4 -p 8892 -n rover &

# 4. Map merge (needs to be running before explore_lite)
ros2 launch maze_mission map_merge.launch.py

# 5. Both drones (mapper + costmap + explore_lite + bridge + aruco)
ros2 launch maze_mission drones.launch.py

# 6. Rover (Nav2 + controller + bridge + aruco)
ros2 launch maze_mission rover.launch.py

# 7. Mission planner (starts monitoring + dispatching)
ros2 launch maze_mission mission.launch.py
```

---

## 10. Updated Development Roadmap

### Phase 1 — Single Drone + aerial_mapper (Week 1-2)
- [ ] PX4 SITL + Gazebo with maze
- [ ] Write `aerial_mapper.py`: depth cam → OccupancyGrid
- [ ] Verify map in RViz2 (should see maze walls as occupied cells)
- [ ] Write `nav2_action_bridge.py`: arm → takeoff → accept hardcoded NavigateToPose goal → fly there

### Phase 2 — Single Drone Autonomous Exploration (Week 2-3)
- [ ] Launch costmap_2d node reading aerial_mapper's map
- [ ] Launch explore_lite pointed at that costmap
- [ ] Verify explore_lite finds frontiers and sends goals
- [ ] Verify nav2_action_bridge flies drone to each frontier
- [ ] Watch the maze get progressively mapped in RViz2

### Phase 3 — Two Drones + Map Merge (Week 3-4)
- [ ] Multi-SITL with 2 drones
- [ ] Launch multirobot_map_merge with known init poses
- [ ] Verify merged /map in RViz2 shows both drones' contributions
- [ ] Both explore_lite instances running — drones should naturally explore different areas
  (explore_lite picks nearest frontier, so spatially separated drones self-organize)

### Phase 4 — ArUco Detection (Week 4)
- [ ] Place ArUco models in Gazebo maze
- [ ] Write aruco_detector.py
- [ ] Verify detections published with correct map-frame poses

### Phase 5 — Rover + Retrieval (Week 4-5)
- [ ] Rover with Nav2 using merged map as global costmap
- [ ] rover_controller sends NavigateToPose to Nav2 on detection
- [ ] nav2_px4_bridge converts cmd_vel → PX4
- [ ] End-to-end: rover navigates maze corridors to tag

### Phase 6 — Full Integration (Week 5-6)
- [ ] mission_planner: monitor detections → dispatch rover → track retrieval
- [ ] Pause/resume explore_lite as needed
- [ ] Complete run: drones explore → tags found → rover retrieves → mission complete

---

## 11. Key Advantages of This Architecture

**Less custom code:** we deleted ~400 lines of custom frontier extraction + map merging and replaced it with battle-tested open-source C++ code.

**Decoupled exploration:** explore_lite runs autonomously per drone. The mission planner doesn't need to micromanage exploration — it just monitors progress and handles the rover. This separation makes debugging much easier.

**Standard interfaces:** by making drones look like Nav2 robots (via the action bridge), we can use any Nav2-compatible exploration package, not just explore_lite. If you later want to switch to a different frontier strategy, you just swap the exploration node.

**Self-organizing multi-drone:** explore_lite picks the nearest frontier. Since the two drones start at different positions, they naturally explore different regions without explicit coordination. This isn't optimal (a centralized assigner would be better), but it's simple and works well for two drones.

---

## 12. Known Limitations & Workarounds

**explore_lite doesn't coordinate between robots.** Each instance greedily picks its nearest frontier. Two drones could occasionally target the same area. Mitigation: the spatial separation from different start positions usually prevents this. For better coordination, you could later write a thin wrapper that filters one drone's costmap to exclude the other drone's vicinity.

**multirobot_map_merge with slam_toolbox is experimental.** The main branch uses slam_gmapping. Since we're using a custom `aerial_mapper.py` instead of SLAM, this doesn't affect us — we publish standard OccupancyGrid directly. But if you switch to slam_toolbox for the drones, use the `feature/slam_toolbox_compat` branch.

**explore_lite's `progress_timeout` must be tuned.** If the drone overshoots or oscillates around a goal, explore_lite might time out and abort. Make the nav2_action_bridge's position threshold generous (0.5m-1.0m) and the progress_timeout long enough (30s+) to avoid false timeouts.

**The rover can't start navigating until the merged map has enough coverage.** The mission planner should wait until a tag is detected AND a valid path exists in the merged map before dispatching the rover. If the path doesn't exist yet (area around tag not mapped), queue the tag and retry after more exploration.
