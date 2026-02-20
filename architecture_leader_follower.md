# Adapted Architecture: Leader-Follower with PX4-ROS2-SLAM-Control + m-explore-ros2

## 0. System Overview — Leader-Follower Paradigm

This architecture implements a **Leader-Follower** system for autonomous maze exploration and object retrieval:

- **Leader — PX4 Drone (×2):** Maps the maze using 2D LiDAR + `slam_toolbox`, with autonomous frontier-based exploration via `m-explore-ros2`.
- **Follower — TurtleBot 4:** Receives the merged map, localizes within it using `nav2_amcl`, and navigates to detected ArUco tags for retrieval.

The core challenge is that these two platforms live in **different coordinate universes**: PX4 speaks NED (North-East-Down), while ROS 2 / TurtleBot speaks ENU (East-North-Up). A dedicated bridge layer handles this translation.

### What the `PX4-ROS2-SLAM-Control` repo gives us

The [`ainolf/PX4-ROS2-SLAM-Control`](https://github.com/ainolf/PX4-ROS2-SLAM-Control) repository provides pre-built, tested components for the PX4 → ROS 2 Jazzy → Gazebo Harmonic → SLAM pipeline. After cloning and inspecting it, here is what is directly reusable:

| Repo component | Role | Replaces in our architecture |
|---|---|---|
| **`drone_slam/odom_converter.py`** | Converts `VehicleOdometry` (PX4 NED/FRD) → `nav_msgs/Odometry` (ROS 2 ENU/FLU) + broadcasts TF `odom → base_link` | Custom NED→ENU code we would have written in `nav2_action_bridge.py` or `aerial_mapper.py` |
| **`drone_slam/slam.launch.py`** | Full launch: `ros_gz_bridge` (clock + lidar), odom_converter, static TF, `slam_toolbox`, RViz2 | Our custom SLAM setup from scratch |
| **`drone_slam/config/slam_params.yaml`** | Pre-tuned `slam_toolbox` config for PX4 drones | Trial-and-error SLAM tuning |
| **`drone_slam_cam/slam_3d.launch.py`** | RGBD alternative: depth camera bridge, `depth_image_proc`, `rtabmap_slam` | Original `aerial_mapper.py` (Option A) |
| **`drone_slam_cam/config/rtabmap_params.yaml`** | Tuned `rtabmap` config with `RGBD/CreateOccupancyGrid: true` | N/A — kept as fallback |
| **`worlds/walls.sdf`** | Gazebo Harmonic world with walls (starting point for the maze) | Building a world from scratch |
| **`ros_gz_bridge` topic patterns** | Exact Gazebo Harmonic topic names for `x500_lidar_2d` and `x500_depth` models | Manual topic discovery |

### What the repo does NOT provide

- No multi-drone support (single drone, no namespacing)
- No autonomous exploration (`explore_lite`, frontiers)
- No map merging (`multirobot_map_merge`)
- No Nav2 action bridge for drones
- No ArUco detection
- No TurtleBot 4 / ground robot integration
- No mission coordination

→ We keep the full **m-explore-ros2 + nav2_action_bridge + mission planner** layer from the original architecture, and add a **Map Bridge + Origin Sync** layer for the TurtleBot 4.

---

## 1. Key Change: 2D LiDAR SLAM Instead of `aerial_mapper.py`

### Before (original architecture) — Option A

Each drone had a downward-facing depth camera. A custom `aerial_mapper.py` node projected depth pixels onto a ground plane to create an `OccupancyGrid`. This required significant custom code: camera intrinsics handling, 3D→2D projection, wall/floor classification, incremental grid maintenance.

### Now — 2D LiDAR + slam_toolbox (from the repo)

We use PX4's `x500_lidar_2d` model, which includes a horizontal 2D LiDAR. `slam_toolbox` (already integrated in the repo) directly generates the `OccupancyGrid` from laser scans — zero custom mapping code.

**Why this works for a drone above a maze:** The 2D LiDAR emits a horizontal fan of rays at flight altitude. If the drone flies at an altitude between the floor and the top of the walls (~1.0–1.5m for a maze with 2m walls), the LiDAR detects walls exactly as a ground robot would. `slam_toolbox` performs Ceres-based scan matching to simultaneously localize and map.

**Critical trade-off:** The LiDAR only sees in its horizontal plane. If the drone flies too high (above the walls) or too low (ground occlusion), it detects nothing. Flight altitude must be carefully chosen and maintained by the `nav2_action_bridge`.

### Fallback kept in reserve — RGBD + rtabmap

The repo also provides `drone_slam_cam` using an RGBD camera + `rtabmap_slam`. Advantage: rtabmap can produce both a 3D map and a 2D projection (`RGBD/CreateOccupancyGrid: true`). This is heavier on compute but available if we need richer perception.

---

## 2. Deep Dive: `odom_converter.py` — The PX4 ↔ ROS 2 Bridge

This node is the **single most critical component** borrowed from the repo. Here is exactly what it does and where you might want to modify it.

### The Problem It Solves

PX4 publishes drone odometry in its own format (`px4_msgs/VehicleOdometry`) using NED/FRD coordinates:
- **NED** (North-East-Down): X→North, Y→East, Z→Down (world frame)
- **FRD** (Forward-Right-Down): body frame

ROS 2 and all its tools (slam_toolbox, Nav2, explore_lite, TF, AMCL…) use ENU/FLU:
- **ENU** (East-North-Up): X→East, Y→North, Z→Up (world frame)
- **FLU** (Forward-Left-Up): body frame

### How It Performs the Conversion

Two rotation matrices are applied:

```
World frame (NED → ENU):          Body frame (FRD → FLU):
    ┌ 0  1  0 ┐                      ┌ 1  0  0 ┐
    │ 1  0  0 │                      │ 0 -1  0 │
    └ 0  0 -1 ┘                      └ 0  0 -1 ┘

In practice:  X_enu = Y_ned,  Y_enu = X_ned,  Z_enu = -Z_ned
```

For orientation (quaternion), it decomposes to a rotation matrix, applies:
```
R_enu_flu = world_transform × R_ned_frd × body_transform
```
Then converts back to quaternion (with positive-w normalization).

### What It Publishes

1. **`/odom`** (`nav_msgs/Odometry`): position, orientation, linear and angular velocities — all in ENU/FLU
2. **TF `odom → base_link`**: the transform required by `slam_toolbox` to localize the drone in the map frame

### Modification Points for Our Project

| What | Current state | Needed change |
|---|---|---|
| **Source topic** | Hardcoded `/fmu/out/vehicle_odometry` | Must be remapped to `/<ns>/fmu/out/vehicle_odometry` for multi-drone |
| **Output topics** | Hardcoded `/odom` | Must be remapped to `/<ns>/odom` |
| **Covariances** | Fixed at 0.01 diagonal | Fine for simulation; derive from PX4 uncertainty estimates for real hardware |
| **QoS** | `qos_profile_sensor_data` (best-effort, volatile) | Correct for PX4 — do not change |
| **sim_time** | Hardcoded `use_sim_time: True` | Fine for simulation; parameterize for real hardware |

**Recommended approach:** Use ROS 2 launch-time remapping rather than forking the file:

```python
Node(
    package='drone_slam',
    executable='odom_converter',
    namespace='drone1',
    remappings=[
        ('/fmu/out/vehicle_odometry', '/drone1/fmu/out/vehicle_odometry'),
        ('/odom', '/drone1/odom'),
    ],
)
```

---

## 3. The TurtleBot 4 Integration — Map Bridge, Origin Sync, Nav2

This is entirely new relative to the original architecture (which used a PX4 rover). The TurtleBot 4 runs its own ROS 2 stack natively, so we don't need `nav2_px4_bridge.py`. Instead, we need:

1. **Map Bridge:** Feed the drone's merged `OccupancyGrid` to the TurtleBot's Nav2 stack.
2. **Coordinate Alignment:** Ensure the map from the drone (originally NED, converted to ENU by `odom_converter`) is correctly oriented for the TurtleBot.
3. **Origin Synchronization:** Align the TurtleBot's starting pose with the drone's map origin.
4. **Nav2 Configuration:** Configure the TurtleBot's costmaps to ingest the external map.

### 3.1 Map Bridge

`multirobot_map_merge` publishes the merged map on `/map` as a standard `nav_msgs/OccupancyGrid`. Since `odom_converter.py` has already converted all drone data to ENU before it reaches `slam_toolbox`, the resulting `/map` is **already in the ENU frame**. No additional rotation of the map itself is needed.

The TurtleBot's `nav2_map_server` is normally used to load a static map from a YAML file. In our case, we don't need `map_server` at all — we use `nav2_amcl` to localize directly against the live `/map` topic, and the `global_costmap` subscribes to it via its `StaticLayer`.

```
multirobot_map_merge ──► /map (OccupancyGrid, ENU frame)
                              │
                    ┌─────────┼──────────────┐
                    ▼         ▼              ▼
              TurtleBot    TurtleBot      mission_planner
              AMCL         global_costmap  (monitors completeness)
```

### 3.2 Coordinate Transformation — Why It's Simpler Than You'd Think

A common pitfall is to worry about NED→ENU conversion at the TurtleBot level. But in our architecture, **that conversion already happened upstream**:

```
PX4 (NED) ──► odom_converter (NED→ENU) ──► slam_toolbox (ENU) ──► /droneN/map (ENU)
                                                                         │
                                                                         ▼
                                                              map_merge ──► /map (ENU)
                                                                              │
                                                                              ▼
                                                                      TurtleBot Nav2 (ENU) ✓
```

The `/map` that reaches the TurtleBot is already fully ENU-compliant. The map's `origin` field in the `OccupancyGrid` message specifies where `(0,0)` is in the map frame, and the `resolution` gives meters-per-cell. `slam_toolbox` fills these correctly because it operates in ENU thanks to `odom_converter`.

**However**, if the TurtleBot and drones don't share a common physical reference (e.g., in a mixed sim/real setup), you may need a static transform between the drone's `map` frame and the TurtleBot's `map` frame. For simulation (same Gazebo world), they naturally share the same origin.

### 3.3 Origin Synchronization — `initialpose` Publisher

When the TurtleBot starts, `nav2_amcl` needs an initial pose estimate to begin localizing in the drone's map. We create a small node that publishes a `geometry_msgs/PoseWithCovarianceStamped` on `/initialpose`:

**Concept:** The TurtleBot's physical starting position in the real world (or Gazebo) must be expressed as coordinates in the drone's map frame. Since both platforms share the same ENU world frame in simulation, we simply provide the TurtleBot's spawn coordinates.

```python
# origin_sync_node.py — publishes initialpose once at startup
# See full implementation in Section 6
```

For a real-world deployment, you'd use a known marker, QR code, or manual measurement to determine the TurtleBot's starting position relative to the drone's map origin.

### 3.4 Nav2 Configuration for External Map

The TurtleBot's Nav2 stack must be configured to:
1. **NOT** launch its own `map_server` (the map comes from `map_merge` on `/map`)
2. Use `nav2_amcl` to localize against the live `/map`
3. Configure `global_costmap` to subscribe to `/map` via the `StaticLayer`
4. Keep `local_costmap` using its own sensors (TurtleBot's LiDAR / depth camera)

```yaml
# turtlebot_nav2_params.yaml — key sections
amcl:
  ros__parameters:
    use_sim_time: true
    # AMCL localizes the TurtleBot within the drone's merged map
    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: /scan                # TurtleBot's own LiDAR
    # Initial pose (overridden by origin_sync_node at runtime)
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      yaw: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.17             # TurtleBot 4 radius
      resolution: 0.05               # Match slam_toolbox resolution
      track_unknown_space: true
      # CRITICAL: subscribe to the drone's merged map
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: /map              # From multirobot_map_merge (drone's merged map)
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan               # TurtleBot's own LiDAR for local obstacles
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55       # Wider than drone (TurtleBot is wider)

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.17
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

---

## 4. Adapter Strategy: Nav2ActionBridge for Drones

The concept is unchanged from the original architecture. `explore_lite` sends `NavigateToPose` goals; the bridge converts them to PX4 offboard commands. What changes: the bridge is **simplified** because `odom_converter` now handles all NED↔ENU conversion for position monitoring.

```
explore_lite              nav2_action_bridge           odom_converter        PX4
    │                            │                          │                 │
    │  NavigateToPose goal       │                          │                 │
    │  (x=3.2, y=5.1) [ENU]     │                          │                 │
    ├───────────────────────────►│                          │                 │
    │                            │  Convert ENU → NED:      │                 │
    │                            │  x_ned=y_enu=5.1         │                 │
    │                            │  y_ned=x_enu=3.2         │                 │
    │                            │  z_ned=-altitude         │                 │
    │                            │  TrajectorySetpoint      │                 │
    │                            ├──────────────────────────────────────────►│
    │                            │                          │                 │
    │                            │              VehicleOdometry (NED)         │
    │                            │                          │◄────────────────┤
    │                            │                          │                 │
    │                            │     /odom (ENU) + TF     │                 │
    │                            │◄─────────────────────────┤                 │
    │                            │  Compare ENU position    │                 │
    │                            │  with ENU goal           │                 │
    │  NavigateToPose result     │                          │                 │
    │  (SUCCEEDED)               │  distance < threshold    │                 │
    │◄───────────────────────────┤                          │                 │
```

**What the bridge does concretely:**
1. Creates a `NavigateToPose` action server
2. On goal received: extracts (x, y) from the goal pose (ENU coordinates from explore_lite)
3. Converts to NED for PX4: `x_ned = y_enu`, `y_ned = x_enu`, `z_ned = -altitude`
4. Publishes `TrajectorySetpoint` + `OffboardControlMode` at 10 Hz
5. Reads `/<ns>/odom` (already ENU thanks to `odom_converter`) to check convergence
6. When horizontal distance < threshold (0.5m) → returns SUCCESS

---

## 5. Full Node Architecture

### Change summary vs. original m-explore architecture

```
REMOVED (replaced by PX4-ROS2-SLAM-Control repo):
  ✗ aerial_mapper.py           → replaced by slam_toolbox + odom_converter from repo
  ✗ Custom NED/ENU conversion  → odom_converter.py from repo handles it

REMOVED (replaced by m-explore-ros2 — unchanged):
  ✗ map_merger.py              → multirobot_map_merge
  ✗ frontier_explorer.py       → explore_lite (×2)

REMOVED (replaced by TurtleBot 4 native stack):
  ✗ nav2_px4_bridge.py         → TurtleBot runs Nav2 natively, no PX4 bridge needed
  ✗ rover_controller.py        → replaced by turtlebot_controller.py (sends Nav2 goals)

ADDED FROM REPO:
  ✓ odom_converter.py          → PX4 VehicleOdometry → /odom + TF (×2, namespaced)
  ✓ slam_params.yaml           → pre-tuned slam_toolbox config for drone
  ✓ ros_gz_bridge patterns     → Gazebo Harmonic topic names for x500_lidar_2d

ADDED (new for TurtleBot 4 integration):
  ✓ origin_sync_node.py        → publishes /initialpose for TurtleBot AMCL
  ✓ turtlebot_nav2_params.yaml → Nav2 config accepting external map on /map
  ✓ turtlebot_controller.py    → receives goals from mission_planner, sends NavigateToPose to Nav2

KEPT / ADAPTED:
  ✓ nav2_action_bridge.py      → fake Nav2 for drones (simplified: reads /odom ENU)
  ✓ drone_costmap               → costmap_2d node per drone (reads map from slam_toolbox)
  ✓ mission_planner.py          → central coordinator
  ✓ aruco_detector.py           → tag detection (×3)
```

### Full Node Graph

```
┌──────────────────────────────────────────────────────────────────┐
│                    DRONE 1 (/drone1/)                             │
│                                                                  │
│  PX4 SITL 1 (x500_lidar_2d)                                     │
│       │                                                          │
│       │ VehicleOdometry (NED)                                    │
│       ▼                                                          │
│  odom_converter ──► /drone1/odom (ENU)                           │
│       │                + TF: odom → base_link                    │
│       │                                                          │
│  Gazebo LiDAR                                                    │
│       │                                                          │
│       ▼                                                          │
│  ros_gz_bridge ──► /drone1/scan (LaserScan)                      │
│                         │                                        │
│       ┌─────────────────┤                                        │
│       ▼                 ▼                                        │
│  slam_toolbox ──► /drone1/map (OccupancyGrid, ENU)               │
│                    + TF: map → odom                              │
│                         │                                        │
│                         ▼                                        │
│                    costmap_2d ──► /drone1/global_costmap/costmap  │
│                                          │                       │
│                                          ▼                       │
│                                    explore_lite                   │
│                                          │                       │
│                                 NavigateToPose action             │
│                                          │                       │
│                                          ▼                       │
│                                 nav2_action_bridge                │
│                                    │         │                   │
│                              reads /drone1/odom                  │
│                              (ENU position)  │                   │
│                                    │   TrajectorySetpoint (NED)  │
│                                    │         │                   │
│                                    │         ▼                   │
│                                    │     PX4 SITL 1              │
│                                                                  │
│  aruco_detector ──────────────────► /aruco/detections             │
└──────────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│                    DRONE 2 (/drone2/)                             │
│           (identical stack, different namespace)                  │
└──────────────────────────────────────────────────────────────────┘

         /drone1/map ──┐
                       ├──► multirobot_map_merge ──► /map (merged, ENU)
         /drone2/map ──┘              │
                                      │
                          ┌───────────┴────────────┐
                          ▼                        ▼
                    TurtleBot 4              mission_planner
                    Nav2 stack               (monitors /map
                    (see below)              for completeness)

┌──────────────────────────────────────────────────────────────────┐
│                  TURTLEBOT 4 (/turtlebot/)                       │
│                                                                  │
│  /map (from map_merge) ──────► AMCL (localization)               │
│                                    │                             │
│                                    ▼                             │
│                              TF: map → odom                      │
│                                                                  │
│  /map (from map_merge) ──────► global_costmap (StaticLayer)      │
│  TurtleBot LiDAR (/scan) ───► global_costmap (ObstacleLayer)    │
│  TurtleBot LiDAR (/scan) ───► local_costmap (VoxelLayer)        │
│                                                                  │
│  origin_sync_node ──────────► /initialpose (at startup)          │
│                                                                  │
│  mission_planner                                                 │
│       │                                                          │
│       │ /mission/turtlebot_goal                                  │
│       ▼                                                          │
│  turtlebot_controller                                            │
│       │                                                          │
│       │ NavigateToPose action (real Nav2)                         │
│       ▼                                                          │
│  Nav2 (full stack)                                               │
│  bt_navigator → planner → controller                             │
│       │                                                          │
│       ▼                                                          │
│  /cmd_vel → TurtleBot 4 base driver                              │
│                                                                  │
│  aruco_detector (front cam) ─► /aruco/detections                 │
└──────────────────────────────────────────────────────────────────┘
```

---

## 6. New Files — TurtleBot 4 Integration Layer

### `origin_sync_node.py` — Initial Pose Publisher

This node publishes the TurtleBot's starting position in the drone's map frame so AMCL can begin localizing. It publishes once (with a brief retry loop to handle startup timing), then stays alive as a latched publisher.

```python
#!/usr/bin/env python3
"""
origin_sync_node.py — Publishes /initialpose for TurtleBot AMCL.

Concept: The TurtleBot's physical starting position (in the real world or
Gazebo) must be expressed as coordinates in the drone's map frame. Since
odom_converter has already transformed everything to ENU, and slam_toolbox
builds the map in ENU, the map frame is ENU-aligned. We simply provide the
TurtleBot's spawn coordinates in that frame.

Parameters:
    initial_x, initial_y, initial_yaw — TurtleBot's start pose in map frame
    publish_count — how many times to publish (AMCL might miss the first one)
    publish_interval — seconds between publishes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class OriginSyncNode(Node):
    def __init__(self):
        super().__init__('origin_sync_node')

        # Declare parameters with defaults
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_interval', 1.0)

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value
        self.max_count = self.get_parameter('publish_count').value
        self.interval = self.get_parameter('publish_interval').value

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.count = 0
        self.timer = self.create_timer(self.interval, self.publish_initial_pose)

        self.get_logger().info(
            f'OriginSync: will publish initialpose at '
            f'({self.x}, {self.y}, yaw={self.yaw}) {self.max_count} times'
        )

    def publish_initial_pose(self):
        if self.count >= self.max_count:
            self.timer.cancel()
            self.get_logger().info('OriginSync: done publishing initialpose')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (rotation around Z axis only)
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Covariance: moderate uncertainty — AMCL will refine from LiDAR
        # 6x6 matrix flattened, diagonal values only
        cov = [0.0] * 36
        cov[0] = 0.25   # x variance (0.5m std dev)
        cov[7] = 0.25   # y variance
        cov[35] = 0.07  # yaw variance (~15° std dev)
        msg.pose.covariance = cov

        self.publisher.publish(msg)
        self.count += 1
        self.get_logger().info(f'OriginSync: published initialpose ({self.count}/{self.max_count})')


def main(args=None):
    rclpy.init(args=args)
    node = OriginSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Why publish multiple times:** AMCL may not be fully initialized when the first message arrives. Publishing 5 times over 5 seconds ensures it catches at least one.

**Why these covariance values:** 0.25 for position (0.5m standard deviation) gives AMCL room to converge from the approximate starting position using its particle filter. Too tight and it might not converge; too loose and it takes longer.

### `turtlebot_controller.py` — Goal Dispatcher

Replaces `rover_controller.py`. The interface is simpler because the TurtleBot already runs a full Nav2 stack — we just send `NavigateToPose` goals directly:

```python
#!/usr/bin/env python3
"""
turtlebot_controller.py — Receives retrieval goals from mission_planner,
sends them as NavigateToPose actions to the TurtleBot's Nav2 stack.

Subscribes: /mission/turtlebot_goal (geometry_msgs/PoseStamped)
Action client: /navigate_to_pose (nav2_msgs/NavigateToPose)

The goals arrive in the map frame (ENU), which is the same frame the
TurtleBot's Nav2 is operating in. No coordinate conversion needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        # Action client to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to goals from mission planner
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/mission/turtlebot_goal',
            self.goal_callback,
            10
        )

        self.current_goal_handle = None
        self.get_logger().info('TurtleBotController: waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('TurtleBotController: Nav2 action server available')

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        # Cancel any active goal
        if self.current_goal_handle is not None:
            self.get_logger().info('Cancelling previous goal')
            self.current_goal_handle.cancel_goal_async()

        # Build Nav2 goal
        goal = NavigateToPose.Goal()
        goal.pose = msg
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        # Send goal
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        # Log distance remaining periodically
        remaining = feedback_msg.feedback.distance_remaining
        if remaining < 1.0:
            self.get_logger().info(f'Distance remaining: {remaining:.2f}m')

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED — at target')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        self.current_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 7. Folder Structure

```
maze_ws/
├── src/
│   │
│   ├── 📦 maze_msgs/                          # Custom messages
│   │   ├── msg/
│   │   │   ├── ArucoDetection.msg
│   │   │   └── ArucoDetectionArray.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── 📦 px4_msgs/                           # Git clone — PX4 message definitions
│   │   └── ...                                #   required by odom_converter
│   │
│   ├── 📡 m-explore-ros2/                     # Git clone — DO NOT MODIFY
│   │   ├── explore/                            #   explore_lite (frontier exploration)
│   │   └── map_merge/                          #   multirobot_map_merge
│   │
│   ├── 🔧 PX4-ROS2-SLAM-Control/             # Git clone — reusable components
│   │   ├── drone_slam/
│   │   │   ├── drone_slam/
│   │   │   │   └── odom_converter.py          #   ⭐ NED→ENU — reused as-is
│   │   │   ├── config/
│   │   │   │   └── slam_params.yaml           #   ⭐ slam_toolbox config — base to adapt
│   │   │   ├── launch/
│   │   │   │   └── slam.launch.py             #   Reference only (we write our own launch)
│   │   │   └── worlds/
│   │   │       └── walls.sdf                  #   Starting point for Gazebo world
│   │   ├── drone_slam_cam/                    #   Fallback: RGBD approach if needed
│   │   └── yolo_ros/                          #   Potentially useful for detection
│   │
│   ├── 🧠 maze_mission/                       # Core mission logic
│   │   ├── maze_mission/
│   │   │   ├── __init__.py
│   │   │   ├── mission_planner.py              #   🎯 Central brain
│   │   │   ├── nav2_action_bridge.py           #   🔌 Fake Nav2 for drones
│   │   │   ├── origin_sync_node.py             #   🆕 Publishes /initialpose for TurtleBot
│   │   │   ├── turtlebot_controller.py         #   🆕 Goal dispatcher for TurtleBot Nav2
│   │   │   └── aruco_detector.py               #   👁️ ArUco detection
│   │   ├── config/
│   │   │   ├── drone_explore.yaml              #   explore_lite params per drone
│   │   │   ├── drone_costmap.yaml              #   costmap_2d params per drone
│   │   │   ├── drone_slam_params.yaml          #   slam_toolbox params (from repo, adapted)
│   │   │   ├── turtlebot_nav2_params.yaml      #   🆕 TurtleBot Nav2 config (external map)
│   │   │   ├── map_merge_params.yaml           #   multirobot_map_merge config
│   │   │   └── aruco_params.yaml
│   │   ├── launch/
│   │   │   ├── sim_full.launch.py              #   🚀 Launches everything
│   │   │   ├── drone.launch.py                 #   1 drone: bridge+odom+slam+costmap+explore+nav2bridge+aruco
│   │   │   ├── drones.launch.py                #   Calls drone.launch.py ×2 with namespaces
│   │   │   ├── turtlebot.launch.py             #   🆕 TurtleBot: Nav2 + AMCL + controller + origin_sync + aruco
│   │   │   ├── map_merge.launch.py
│   │   │   └── mission.launch.py
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── 🌍 maze_gazebo/                        # Simulation
│   │   ├── worlds/maze.sdf                    #   Based on walls.sdf from repo, extended
│   │   ├── models/
│   │   │   ├── maze/meshes/maze.dae
│   │   │   └── aruco_tags/
│   │   ├── launch/gazebo.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── 🧭 maze_nav2/                          # Nav2 config for TURTLEBOT ONLY
│       ├── config/nav2_params.yaml
│       ├── launch/nav2.launch.py
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

## 8. Configuration Files

### `drone_slam_params.yaml` — slam_toolbox per drone

Adapted from the repo's `slam_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    mode: mapping
    map_frame: map
    odom_frame: odom
    base_frame: base_link
    provide_odom_frame: false          # odom_converter already provides odom → base_link
    scan_topic: scan                   # auto-namespaced: /droneN/scan
    resolution: 0.05                   # 5cm — good balance of detail vs performance
    max_laser_range: 10.0              # matches x500_lidar_2d specs
    min_laser_range: 0.1
    transform_publish_period_sec: 0.05
    max_update_rate_hz: 20.0
    use_scan_matching_2d: true
    enable_loop_closure: true          # important for drift correction
    scan_queue_size: 10
    publish_period_sec: 0.1            # reduced vs repo (0.05) — CPU savings with 2 drones
```

**Changes from repo:** `publish_period_sec` increased from 0.05 to 0.1 to save CPU with two simultaneous SLAM instances.

### `drone_explore.yaml` — explore_lite per drone

```yaml
explore_node:
  ros__parameters:
    robot_base_frame: base_link
    costmap_topic: global_costmap/costmap
    costmap_updates_topic: global_costmap/costmap_updates
    visualize: true
    planner_frequency: 0.33            # replan every 3 sec
    progress_timeout: 30.0             # abort and re-plan after 30s of no progress
    potential_scale: 3.0               # weight for distance to frontier
    orientation_scale: 0.0             # drones don't care about heading during transit
    gain_scale: 1.0                    # weight for frontier information gain
    transform_tolerance: 0.5
    min_frontier_size: 0.75            # ignore tiny frontiers (meters)
    return_to_init: false
```

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
      robot_radius: 0.3
      resolution: 0.05                 # MUST match slam_toolbox resolution
      track_unknown_space: true        # CRITICAL — explore_lite needs unknown cells
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        map_topic: map                 # namespaced → /<ns>/map from slam_toolbox
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.5
```

### `map_merge_params.yaml`

```yaml
map_merge:
  ros__parameters:
    robot_map_topic: map
    robot_namespace: ""
    known_init_poses: true
    merging_rate: 2.0
    discovery_rate: 0.5
    estimation_rate: 0.5
    estimation_confidence: 0.6
    drone1:
      init_pose_x: 0.0
      init_pose_y: 0.0
      init_pose_z: 0.0
      init_pose_yaw: 0.0
    drone2:
      init_pose_x: 2.0
      init_pose_y: 0.0
      init_pose_z: 0.0
      init_pose_yaw: 0.0
```

---

## 9. Updated Topic Map

```yaml
# ═══════ PER DRONE (×2, namespaced) ═══════

# PX4 → odom_converter (from repo)
/<ns>/fmu/out/vehicle_odometry      # PX4 → odom_converter (NED format)
/<ns>/odom                          # odom_converter → slam_toolbox + nav2_action_bridge (ENU)
# TF: odom → base_link             # odom_converter broadcasts this

# Gazebo LiDAR → ros_gz_bridge → slam_toolbox
/<ns>/scan                          # ros_gz_bridge → slam_toolbox (LaserScan)

# slam_toolbox → per-drone map
/<ns>/map                           # slam_toolbox → map_merge + costmap_2d (OccupancyGrid)
# TF: map → odom                   # slam_toolbox broadcasts this

# Costmap for explore_lite
/<ns>/global_costmap/costmap        # costmap_2d → explore_lite
/<ns>/global_costmap/costmap_updates

# Exploration control
/<ns>/explore/frontiers             # explore_lite → RViz (visualization)
/<ns>/explore/resume                # mission_planner → explore_lite (pause/resume)

# Nav2 action (fake, served by bridge)
/<ns>/navigate_to_pose              # explore_lite → nav2_action_bridge (action)

# PX4 interface (from nav2_action_bridge)
/<ns>/fmu/in/offboard_control_mode  # nav2_action_bridge → PX4
/<ns>/fmu/in/trajectory_setpoint    # nav2_action_bridge → PX4
/<ns>/fmu/in/vehicle_command        # nav2_action_bridge → PX4
/<ns>/fmu/out/vehicle_status        # PX4 → nav2_action_bridge

# ArUco (drone cameras)
/<ns>/camera/image_raw              # Gazebo → aruco_detector
/aruco/detections                   # aruco_detector → mission_planner


# ═══════ GLOBAL ═══════

# Merged map
/map                                # map_merge → TurtleBot AMCL + global_costmap + mission_planner
/map_updates                        # map_merge → TurtleBot global_costmap

# Mission coordination
/mission/turtlebot_goal             # mission_planner → turtlebot_controller
/mission/status                     # mission_planner → logging/RViz


# ═══════ TURTLEBOT 4 ═══════

/initialpose                        # origin_sync_node → AMCL (PoseWithCovarianceStamped)
/navigate_to_pose                   # turtlebot_controller → Nav2 BT navigator (real action)
/scan                               # TurtleBot LiDAR → AMCL + local_costmap + global_costmap
/cmd_vel                            # Nav2 controller → TurtleBot base driver
/odom                               # TurtleBot wheel encoders → AMCL
/turtlebot/camera/image_raw         # TurtleBot front camera → aruco_detector
```

---

## 10. Launch Order (Simulation)

```bash
# 1. Gazebo world with maze + TurtleBot model
ros2 launch maze_gazebo gazebo.launch.py

# 2. PX4 SITL instances (2 drones only — TurtleBot is Gazebo-native)
#    Uses x500_lidar_2d model for both drones
bash scripts/launch_px4_sitl.sh

# 3. XRCE-DDS agents (PX4 ↔ ROS 2 bridge)
MicroXRCEAgent udp4 -p 8888 &    # drone1
MicroXRCEAgent udp4 -p 8890 &    # drone2

# 4. Map merge (must be running before explore_lite)
ros2 launch maze_mission map_merge.launch.py

# 5. Both drones (gz_bridge + odom_converter + slam_toolbox + costmap + explore_lite + nav2_bridge + aruco)
ros2 launch maze_mission drones.launch.py

# 6. TurtleBot (Nav2 + AMCL + origin_sync + controller + aruco)
#    NOTE: TurtleBot can start before drones, but won't navigate until /map is available
ros2 launch maze_mission turtlebot.launch.py

# 7. Mission planner (starts monitoring + dispatching)
ros2 launch maze_mission mission.launch.py
```

**Key difference vs. original:** No `nav2_px4_bridge` or XRCE-DDS agent for the TurtleBot. It communicates entirely through standard ROS 2 topics.

---

## 11. Development Roadmap

### Phase 1 — Single Drone + LiDAR SLAM (Week 1)
- [ ] PX4 SITL + Gazebo Harmonic with repo's `walls.sdf` world
- [ ] Launch repo's `slam.launch.py` as-is → verify map builds in RViz2
- [ ] Fly drone manually (keyboard via repo's `keyboard-mavsdk-test.py`)
- [ ] Validate: `/map` populates correctly, TF tree `map → odom → base_link` is consistent
- [ ] **Deliverable:** 2D map of `walls` world generated by slam_toolbox

### Phase 2 — Namespace + Nav2 Action Bridge (Week 1-2)
- [ ] Namespace `odom_converter` via launch remapping
- [ ] Write `nav2_action_bridge.py`: arm → takeoff → accept hardcoded NavigateToPose goal → fly there
- [ ] Test: send a goal via CLI, drone navigates to it
- [ ] **Deliverable:** drone flying to arbitrary points via NavigateToPose interface

### Phase 3 — Autonomous Exploration (Week 2-3)
- [ ] Replace `walls` world with the maze
- [ ] Launch costmap_2d reading slam_toolbox's map
- [ ] Launch explore_lite pointed at the costmap
- [ ] Verify: explore_lite finds frontiers, sends goals, drone explores autonomously
- [ ] **Deliverable:** single drone autonomously mapping the maze

### Phase 4 — Two Drones + Map Merge (Week 3-4)
- [ ] Multi-SITL with 2 x500_lidar_2d instances
- [ ] Launch multirobot_map_merge with known initial poses
- [ ] Verify merged `/map` in RViz2 shows both drones' contributions
- [ ] Both explore_lite instances running — drones self-organize by proximity
- [ ] **Deliverable:** complete merged maze map from 2 drones

### Phase 5 — TurtleBot 4 Integration (Week 4-5)
- [ ] Spawn TurtleBot 4 in Gazebo maze
- [ ] Configure Nav2 with external `/map` (no `map_server`)
- [ ] Launch AMCL + `origin_sync_node` → verify TurtleBot localizes correctly
- [ ] Send test goals → TurtleBot navigates maze corridors using drone's map
- [ ] **Deliverable:** TurtleBot navigating autonomously using the drone's map

### Phase 6 — ArUco Detection (Week 5)
- [ ] Add RGB cameras to drone models (alongside LiDAR)
- [ ] Place ArUco models in Gazebo maze
- [ ] Write `aruco_detector.py` → verify detections with correct map-frame poses
- [ ] **Deliverable:** detected tags with accurate global positions

### Phase 7 — Full Integration (Week 5-6)
- [ ] `mission_planner`: monitor detections → dispatch TurtleBot → track retrieval
- [ ] Pause/resume explore_lite as needed
- [ ] Complete run: drones explore → tags found → TurtleBot retrieves → mission complete
- [ ] **Deliverable:** end-to-end autonomous mission

---

## 12. Advantages of This Architecture

**Minimal custom code:** By combining the repo's `odom_converter` + `slam_toolbox` with m-explore's `explore_lite` + `map_merge`, we eliminated both `aerial_mapper.py` (~200 lines) and the original `frontier_explorer.py` + `map_merger.py` (~400 lines). The only custom code left is the `nav2_action_bridge` (~100 lines), `mission_planner`, and the thin TurtleBot integration layer.

**Proven PX4 → SLAM pipeline:** The `odom_converter` + `slam_toolbox` chain is already tested and functional with PX4 on Gazebo Harmonic. No custom depth-projection bugs to debug.

**Native TurtleBot 4 stack:** Unlike the original PX4 rover (which needed `nav2_px4_bridge` to translate cmd_vel → TrajectorySetpoint), the TurtleBot 4 runs Nav2 natively. We send `NavigateToPose` goals directly — no translation layer, no additional failure modes.

**Clean coordinate boundary:** NED→ENU conversion happens exactly once, in `odom_converter`. Everything downstream (slam_toolbox, explore_lite, map_merge, TurtleBot AMCL) operates in pure ENU. No "which frame am I in?" confusion.

**Version coherence:** The repo targets ROS 2 Jazzy + Gazebo Harmonic + recent PX4 — exactly our stack.

---

## 13. Known Limitations & Workarounds

**The 2D LiDAR requires precise flight altitude.** If the drone flies above the walls, the LiDAR sees nothing. If too low, ground clutter. Solution: fix altitude in `nav2_action_bridge` to mid-wall-height and use `TrajectorySetpoint`'s native altitude hold.

**The `x500_lidar_2d` model doesn't include an RGB camera by default.** For ArUco detection, you'll need to either modify the SDF to add a camera (the repo's `drone_slam_cam` shows how to bridge an RGBD camera on the `x500_depth` model) or create a composite `x500_lidar_cam` model.

**The TurtleBot cannot navigate until the merged map has sufficient coverage.** The `mission_planner` should wait until a tag is detected AND a valid path exists in `/map` before dispatching the TurtleBot. Queue tags and retry as more of the map becomes available.

**AMCL initial convergence may take a few seconds.** The TurtleBot needs to see some LiDAR features that match the drone's map. In an open area with few features, convergence is slow. Mitigation: start the TurtleBot near a distinctive area (corner, junction) and use `origin_sync_node`'s covariances to guide the particle filter.

**explore_lite doesn't coordinate between drones.** Each instance greedily picks its nearest frontier. Spatial separation from different start positions usually prevents overlap, but it's not optimal.

**`odom_converter` uses fixed covariances.** The hardcoded 0.01 diagonal values are fine for simulation but don't reflect real sensor uncertainty. For real hardware, derive these from PX4's EKF2 uncertainty estimates.

---

## 14. Quick Reference: Dependencies

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop

# Gazebo Harmonic + bridges
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# SLAM
sudo apt install ros-jazzy-slam-toolbox

# Nav2 (for TurtleBot + drone costmaps)
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# AMCL (part of Nav2, but listed for clarity)
sudo apt install ros-jazzy-nav2-amcl

# TurtleBot 4 simulation (if using sim)
sudo apt install ros-jazzy-turtlebot4-simulator ros-jazzy-turtlebot4-navigation

# Depth processing (if using RGBD fallback)
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-depth-image-proc

# PX4 messages
cd ~/maze_ws/src
git clone https://github.com/PX4/px4_msgs.git

# The repo
git clone https://github.com/ainolf/PX4-ROS2-SLAM-Control.git

# m-explore-ros2
git clone https://github.com/robo-friends/m-explore-ros2.git

# Micro XRCE-DDS Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/

# PX4 Autopilot
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gz_x500_lidar_2d

# Build the workspace
cd ~/maze_ws
colcon build --symlink-install
source install/setup.bash
```
