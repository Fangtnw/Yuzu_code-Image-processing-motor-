#!/usr/bin/env bash
# Apply the project driver extension to the pinned upstream revision.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/ethercat_driver_ros2" >&2
  exit 2
fi
DRIVER_ROOT="$1"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXPECTED_REVISION=97c6dc3afbb89928901043ad06d1d1e5f7be104f
PATCH_FILE="$REPO_ROOT/patches/ethercat-driver-multiaxis.patch"
if [[ "$(git -C "$DRIVER_ROOT" rev-parse HEAD)" != "$EXPECTED_REVISION" ]]; then
  echo "Unexpected EtherCAT driver revision; use the pinned manifest in a new workspace." >&2
  exit 1
fi
if git -C "$DRIVER_ROOT" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
  echo "Project EtherCAT driver patch is already applied."
elif git -C "$DRIVER_ROOT" apply --check "$PATCH_FILE"; then
  git -C "$DRIVER_ROOT" apply "$PATCH_FILE"
  echo "Applied project EtherCAT driver patch."
else
  echo "Driver patch conflicts with local changes; nothing was overwritten." >&2
  exit 1
fi
