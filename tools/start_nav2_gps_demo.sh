#!/usr/bin/env bash
set -euo pipefail

# Start Nav2 GPS waypoint demo + optional waypoint sender/logger.
#
# Usage:
#   ./tools/start_nav2_gps_demo.sh [mode] [file]
#
# Modes:
#   none (default)        Only launch the main stack
#   logged               Follow waypoints from YAML file
#   interactive           Listen to Mapviz clicks (wgs84)
#   logger               GUI to log waypoints to YAML
#
# File:
#   - For "logged": path to YAML (defaults to demo_waypoints.yaml)
#   - For "logger": path to output YAML (defaults to ~/gps_waypoints.yaml)
#
# Container override:
#   CONTAINER=mi_contenedor WS_IN_CONTAINER=/ros2_ws ./tools/start_nav2_gps_demo.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

MODE="${1:-none}"
FILE_ARG="${2:-}"

CONTAINER="${CONTAINER:-cuatri_navigation}"
WS_IN_CONTAINER="${WS_IN_CONTAINER:-/ros2_ws}"

use_container=false
if command -v docker >/dev/null 2>&1; then
  if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
    use_container=true
  fi
fi

run_in_context() {
  local cmd="$1"
  if [[ "${use_container}" == "true" ]]; then
    docker exec -it "${CONTAINER}" bash -lc "${cmd}"
  else
    bash -lc "${cmd}"
  fi
}

if [[ "${use_container}" == "true" ]]; then
  SETUP_PREFIX="${WS_IN_CONTAINER}"
else
  SETUP_PREFIX="${WS_DIR}"
fi

read -r -d '' COMMANDS <<'EOS' || true
set -euo pipefail
if [[ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
else
  echo "ROS 2 setup.bash not found. Is ROS 2 installed?"
  exit 1
fi

if [[ -f "__WS_DIR__/install/setup.bash" ]]; then
  source "__WS_DIR__/install/setup.bash"
else
  echo "Workspace install/setup.bash not found. Build with colcon first."
  exit 1
fi

ros2 launch navegacion_gps gps_waypoint_follower.launch.py \
  use_rviz:=True use_mapviz:=True &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}"
  fi
}
trap cleanup EXIT INT TERM

sleep 2

case "__MODE__" in
  none)
    wait "${LAUNCH_PID}"
    ;;
  logged)
    if [[ -n "__FILE_ARG__" ]]; then
      ros2 run navegacion_gps logged_waypoint_follower "__FILE_ARG__"
    else
      ros2 run navegacion_gps logged_waypoint_follower
    fi
    ;;
  interactive)
    ros2 run navegacion_gps interactive_waypoint_follower
    ;;
  logger)
    if [[ -n "__FILE_ARG__" ]]; then
      ros2 run navegacion_gps gps_waypoint_logger "__FILE_ARG__"
    else
      ros2 run navegacion_gps gps_waypoint_logger
    fi
    ;;
  *)
    echo "Unknown mode: __MODE__"
    exit 2
    ;;
esac
EOS

COMMANDS="${COMMANDS//__WS_DIR__/${SETUP_PREFIX}}"
COMMANDS="${COMMANDS//__MODE__/${MODE}}"
COMMANDS="${COMMANDS//__FILE_ARG__/${FILE_ARG}}"

run_in_context "${COMMANDS}"
