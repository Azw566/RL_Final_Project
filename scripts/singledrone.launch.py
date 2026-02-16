import subprocess
import time
import os

def run_in_new_terminal(command, title):
    """
    Launches a command in a new gnome-terminal window.
    Change 'gnome-terminal' to your terminal of choice (e.g., 'xterm').
    """
    # The bash -c "source...; command; exec bash" keeps the terminal open after completion
    full_cmd = f'gnome-terminal --title="{title}" -- bash -c "source /opt/ros/humble/setup.bash; source install/setup.bash; {command}; exec bash"'
    subprocess.Popen(full_cmd, shell=True)

def main():
    # 1. Start Gazebo with the maze
    # Using your existing gazebo launch
    print("Launching Gazebo...")
    run_in_new_terminal("ros2 launch maze_gazebo gazebo.launch.py world:=maze.sdf", "1. Gazebo Simulation")
    time.sleep(5) # Wait for Gazebo to initialize

    # 2. Start PX4 SITL
    # Using your provided script
    print("Launching PX4 SITL...")
    run_in_new_terminal("bash scripts/launch_px4_sitl.sh", "2. PX4 Flight Stack")
    time.sleep(3)

    # 3. MicroXRCE-DDS Agent
    # Required for the drone to talk to ROS 2
    print("Launching MicroXRCE Agent...")
    run_in_new_terminal("MicroXRCEAgent udp4 -p 8888 -n drone1", "3. DDS Agent")
    time.sleep(2)

    # 4. Drone Adapter Nodes
    # Launches your mapper and the fake Nav2 bridge
    print("Launching Drone Adapters...")
    run_in_new_terminal("ros2 launch maze_mission drone.launch.py", "4. Drone Logic (Mapper & Bridge)")

    # 5. (Optional) RViz2
    print("Launching RViz2...")
    run_in_new_terminal("rviz2", "5. RViz Visualization")

    print("\nAll components launched in separate terminals.")

if __name__ == "__main__":
    main()
