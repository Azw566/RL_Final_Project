#!/bin/bash

# Script to launch PX4 SITL for drone mapping
# Usage: ./launch_px4_sitl.sh [instance_id] [model]

INSTANCE_ID=${1:-0}
MODEL=${2:-iris_rplidar}
PX4_DIR=${PX4_DIR:-$HOME/px4_workspace/PX4-Autopilot}
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at $PX4_DIR"
    echo "Please set PX4_DIR environment variable or install PX4"
    exit 1
fi

# Source PX4 Gazebo Classic paths (plugins, models, libraries)
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR"

export PX4_SIM_MODEL=gazebo-classic_${MODEL}
export PX4_UXRCE_DDS_NS=drone1

# Spawn the drone model into the already-running Gazebo
MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
echo "Spawning ${MODEL} into Gazebo..."
while gz model --verbose --spawn-file="${MODEL_PATH}/${MODEL}/${MODEL}.sdf" --model-name=${MODEL} -x 0 -y -8 -z 0.5 2>&1 | grep -q "An instance of Gazebo is not running."; do
    echo "Waiting for Gazebo..."
    sleep 1
done
echo "Model spawned."

# Launch PX4 SITL
cd "$PX4_DIR"
$BUILD_DIR/bin/px4 -i $INSTANCE_ID "$BUILD_DIR/etc"
