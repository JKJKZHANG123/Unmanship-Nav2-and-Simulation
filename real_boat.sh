#!/usr/bin/env bash
# Start the real-boat stack only: Unitree L1, RTK, Point-LIO, perception, and Nav2.
# This script intentionally does not start Gazebo/VRX or the thrust bridge.
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
exec ros2 launch my_mapping_launcher real_boat_system.launch.py "$@"
