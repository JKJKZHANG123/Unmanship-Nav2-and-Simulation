#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/../../local_setup.bash" 2>/dev/null || true
exec ros2 run usv_monitor usv_monitor_node "$@"
