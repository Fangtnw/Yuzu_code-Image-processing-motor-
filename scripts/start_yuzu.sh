#!/usr/bin/env bash
set -euo pipefail

usage() { echo "Usage: $0 --workspace /absolute/path/to/azd3a_ws"; }
WORKSPACE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) WORKSPACE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ -z "$WORKSPACE" || "$WORKSPACE" != /* ]]; then
  echo "--workspace must be an absolute path" >&2; usage >&2; exit 2
fi
source /opt/ros/humble/setup.bash
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
source "$WORKSPACE/install/setup.bash"
exec ros2 launch motor_controller azd3a_motor1_motor2_gui.launch.py
