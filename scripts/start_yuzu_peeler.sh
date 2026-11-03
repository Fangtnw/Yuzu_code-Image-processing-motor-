#!/usr/bin/env bash
set -euo pipefail

usage() { echo "Usage: $0 --workspace /absolute/path/to/azd3a_ws [--ros-interface WIFI_IFACE]"; }
WORKSPACE=""
ROS_INTERFACE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; WORKSPACE="$2"; shift 2 ;;
    --ros-interface) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; ROS_INTERFACE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ -z "$WORKSPACE" || "$WORKSPACE" != /* ]]; then
  echo "--workspace must be an absolute path" >&2; usage >&2; exit 2
fi
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -x /opt/etherlab/bin/ethercat || ! -e /dev/EtherCAT0 ]]; then
  echo "EtherCAT is not ready. Start/configure the EtherCAT master first." >&2
  echo "Expected /opt/etherlab/bin/ethercat and /dev/EtherCAT0." >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "Workspace is not built: $WORKSPACE" >&2
  echo "Run scripts/setup_workspace.sh first." >&2
  exit 1
fi
set +u
source "$WORKSPACE/install/setup.bash"
set -u
if [[ -n "$ROS_INTERFACE" ]]; then
  if [[ ! "$ROS_INTERFACE" =~ ^[a-zA-Z0-9_.-]+$ || ! -d "/sys/class/net/$ROS_INTERFACE" ]]; then
    echo "Invalid ROS network interface: $ROS_INTERFACE" >&2
    exit 2
  fi
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$ROS_INTERFACE\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "ROS 2 Cyclone DDS using $ROS_INTERFACE (domain ${ROS_DOMAIN_ID:-0})"
fi
exec ros2 launch motor_controller yuzu_peeler.launch.py
