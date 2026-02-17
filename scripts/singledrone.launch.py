import subprocess
import time
import os

# PX4 paths
PX4_DIR = os.environ.get('PX4_DIR', os.path.expanduser('~/px4_workspace/PX4-Autopilot'))
BUILD_DIR = f"{PX4_DIR}/build/px4_sitl_default"

# Kill everything from previous runs: Gazebo, PX4, XRCE agent, AND all ROS 2 nodes
os.system("killall -9 MicroXRCEAgent px4 gzserver gzclient 2>/dev/null")
os.system("pkill -9 -f 'ros2|rviz2|slam_toolbox|nav2|lifecycle_manager|controller_server|"
          "planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|"
          "smoother_server|explore|mission_planner|dynamic_tf_publisher|costmap|"
          "rplidar_static_tf' 2>/dev/null")
os.system("sleep 2")

# This must be sourced BEFORE Gazebo so it can find PX4's plugins (e.g. libgazebo_mavlink_interface)
PX4_GAZEBO_SETUP = (
    f"source {PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash {PX4_DIR} {BUILD_DIR}"
)

def run_in_new_terminal(command, title):
    """
    Launches a command in a new gnome-terminal window.
    """
    full_cmd = f'gnome-terminal --title="{title}" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; {command}; exec bash"'
    subprocess.Popen(full_cmd, shell=True)

def main():
    # 1. Start Gazebo with the maze (source PX4 gazebo paths first so plugins are found)
    print("Launching Gazebo...")
    run_in_new_terminal(
        f"{PX4_GAZEBO_SETUP}; ros2 launch maze_gazebo gazebo.launch.py world:=rescue_maze.world",
        "1. Gazebo Simulation"
    )
    time.sleep(5)

    # 2. Start PX4 SITL (spawns the iris drone into Gazebo, then starts PX4)
    print("Launching PX4 SITL...")
    run_in_new_terminal("bash src/ros2_px4_drone/scripts/launch_px4_sitl.sh", "2. PX4 Flight Stack")
    time.sleep(10)

    # 3. MicroXRCE-DDS Agent
    print("Launching MicroXRCE Agent...")
    run_in_new_terminal("snap run micro-xrce-dds-agent udp4 -p 8888", "3. DDS Agent")
    time.sleep(2)

    # 4. Drone Adapter Nodes
    print("Launching Drone Adapters...")
    run_in_new_terminal("ros2 launch maze_mission drone.launch.py", "4. Drone Logic (Mapper & Bridge)")

    # RViz is already launched by px4_drone_explore.launch.py (via drone.launch.py)

    print("\nAll components launched in separate terminals.")

if __name__ == "__main__":
    main()
