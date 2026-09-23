#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --workspace /absolute/path/to/azd3a_ws"
}

WORKSPACE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) WORKSPACE="${2:-}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$WORKSPACE" || "$WORKSPACE" != /* ]]; then
  echo "--workspace must be an absolute path" >&2
  usage >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble" >&2
  exit 1
fi
for command in vcs rosdep colcon; do
  command -v "$command" >/dev/null || { echo "Missing command: $command" >&2; exit 1; }
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$WORKSPACE/src"
vcs import "$WORKSPACE/src" < "$REPO_ROOT/azd3a_ws.repos"
ln -sfn "$REPO_ROOT/motion_cmd" "$WORKSPACE/src/motion_cmd"

source /opt/ros/humble/setup.bash
rosdep install --from-paths "$WORKSPACE/src" --ignore-src -r -y
cd "$WORKSPACE"
colcon build --packages-select motor_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Workspace ready: $WORKSPACE"
echo "Source it with: source $WORKSPACE/install/setup.bash"
