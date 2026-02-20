#!/usr/bin/env bash
# launch_px4_sitl.sh — Launches 2 PX4 SITL instances with x500_lidar_2d model
#                       and starts the XRCE-DDS agent for each.
#
# Prerequisites:
#   - PX4-Autopilot built:  cd ~/PX4-Autopilot && make px4_sitl gz_x500_lidar_2d
#   - MicroXRCEAgent installed and on PATH
#   - Gazebo Harmonic already running with the maze world
#
# Usage:
#   bash scripts/launch_px4_sitl.sh
#
# Each drone uses a separate UDP port for XRCE-DDS:
#   Drone 1: PX4 instance 0, port 8888
#   Drone 2: PX4 instance 1, port 8890
#
# Namespacing: PX4 SITL instances use PX4_GZ_MODEL_POSE and PX4_UXRCE_DDS_NS
#              to set their starting position and ROS 2 topic namespace.

set -euo pipefail

PX4_DIR="${HOME}/px4_workspace/PX4-Autopilot"
LOG_DIR="${HOME}/maze_ws/logs"
mkdir -p "${LOG_DIR}"

echo "[launch_px4_sitl] Starting Drone 1 (instance 0)..."
cd "${PX4_DIR}"
PX4_SYS_AUTOSTART=4017 \
PX4_GZ_MODEL=x500_lidar_2d \
PX4_GZ_MODEL_NAME=x500_lidar_2d_0 \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
PX4_UXRCE_DDS_NS=drone1 \
    ./build/px4_sitl_default/bin/px4 \
    -i 0 \
    -d "${LOG_DIR}/px4_drone1.log" &
PX4_PID1=$!
echo "[launch_px4_sitl] Drone 1 PID: ${PX4_PID1}"

# Brief pause to avoid port conflicts during startup
sleep 2

echo "[launch_px4_sitl] Starting Drone 2 (instance 1)..."
PX4_SYS_AUTOSTART=4017 \
PX4_GZ_MODEL=x500_lidar_2d \
PX4_GZ_MODEL_NAME=x500_lidar_2d_1 \
PX4_GZ_MODEL_POSE="2,0,0.3,0,0,0" \
PX4_UXRCE_DDS_NS=drone2 \
    ./build/px4_sitl_default/bin/px4 \
    -i 1 \
    -d "${LOG_DIR}/px4_drone2.log" &
PX4_PID2=$!
echo "[launch_px4_sitl] Drone 2 PID: ${PX4_PID2}"

sleep 3

echo "[launch_px4_sitl] Starting XRCE-DDS Agent for Drone 1 (port 8888)..."
MicroXRCEAgent udp4 -p 8888 &
AGENT1_PID=$!

echo "[launch_px4_sitl] Starting XRCE-DDS Agent for Drone 2 (port 8890)..."
MicroXRCEAgent udp4 -p 8890 &
AGENT2_PID=$!

echo ""
echo "╔══════════════════════════════════════════╗"
echo "║  PX4 SITL + XRCE-DDS Agents running      ║"
echo "║  Drone 1: PID=${PX4_PID1}, Agent port 8888   ║"
echo "║  Drone 2: PID=${PX4_PID2}, Agent port 8890   ║"
echo "║  Press Ctrl+C to stop all                 ║"
echo "╚══════════════════════════════════════════╝"

# Trap SIGINT/SIGTERM to cleanly kill all children
cleanup() {
    echo "[launch_px4_sitl] Stopping all processes..."
    kill "${PX4_PID1}" "${PX4_PID2}" "${AGENT1_PID}" "${AGENT2_PID}" 2>/dev/null || true
    wait
    echo "[launch_px4_sitl] Done."
}
trap cleanup SIGINT SIGTERM

# Wait for all background jobs
wait
