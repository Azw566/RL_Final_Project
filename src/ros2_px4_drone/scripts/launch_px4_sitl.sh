#!/bin/bash

# Script to launch PX4 SITL for drone mapping
# Usage: ./launch_px4_sitl.sh [instance_id] [model]

INSTANCE_ID=${1:-0}
MODEL=${2:-x500_depth}

# Set PX4 paths (adjust if needed)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}

if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at $PX4_DIR"
    echo "Please set PX4_DIR environment variable or install PX4"
    exit 1
fi

cd "$PX4_DIR"

# Set autostart ID (4001 = iris quadcopter)
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL=$MODEL

# Launch PX4 SITL with instance ID
# Instance ID offsets ports: instance 0 uses default ports, instance 1 uses +2, etc.
./build/px4_sitl_default/bin/px4 -i $INSTANCE_ID
