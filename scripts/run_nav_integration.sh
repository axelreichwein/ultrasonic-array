#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  ROS_DISTRO=humble
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash

WORLD_FILE="ultrasonic_array/simulation/corridor.world"

echo "[nav-integration] Starting Gazebo corridor simulation..."
gzserver --verbose "${WORLD_FILE}" >/tmp/gzserver_nav.log 2>&1 &
GZ_PID=$!
cleanup() {
  kill "${GZ_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 5

echo "[nav-integration] Running obstacle response tests..."
PYTHONPATH=ultrasonic_array python3 -m pytest -q \
  ultrasonic_array/test/test_nav_integration_logic.py \
  ultrasonic_array/test/test_vvr_008_static_obstacle_stop.py

echo "[nav-integration] Completed successfully."
