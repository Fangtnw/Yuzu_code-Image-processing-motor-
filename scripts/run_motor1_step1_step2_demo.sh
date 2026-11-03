#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [--workspace /absolute/path/to/azd3a_ws] [--execute]"
  echo "Without --execute, preview one live detector reading and both targets."
}

WORKSPACE="$HOME/kyutech/azd3a_ws"
EXECUTE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      [[ $# -ge 2 ]] || { usage >&2; exit 2; }
      WORKSPACE="$2"
      shift 2
      ;;
    --execute)
      EXECUTE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "$WORKSPACE" != /* || ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "ROS workspace setup not found: $WORKSPACE/install/setup.bash" >&2
  exit 1
fi

set +u
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export ROS_LOCALHOST_ONLY=0
unset RMW_IMPLEMENTATION CYCLONEDDS_URI

args=(--offset-from-detector --step demo --lift-mm 150)
if [[ "$EXECUTE" -eq 1 ]]; then
  args+=(--execute)
fi
exec python3 -m motor_controller.yuzu_motor1_reference_cli "${args[@]}"
