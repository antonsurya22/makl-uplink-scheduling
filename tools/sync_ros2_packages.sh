#!/usr/bin/env bash
# Copy network_aware_nav and naw_interfaces from a ROS 2 workspace into ros2_ws/src
# (without build output and caches).
# usage: bash tools/sync_ros2_packages.sh [WORKSPACE]    default: ~/ros2_ws
set -euo pipefail
WS="${1:-$HOME/ros2_ws}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
for pkg in network_aware_nav naw_interfaces; do
  [ -d "$WS/src/$pkg" ] || { echo "not found: $WS/src/$pkg" >&2; exit 1; }
  rm -rf "$REPO/ros2_ws/src/$pkg"
  mkdir -p "$REPO/ros2_ws/src/$pkg"
  (cd "$WS/src/$pkg" && find . -type f ! -path '*/__pycache__/*' ! -name '*.pyc' ! -name '*~' ! -name '*.swp' \
     ! -path './build/*' ! -path './install/*' ! -path './log/*' -print0 \
     | xargs -0 -I{} cp --parents {} "$REPO/ros2_ws/src/$pkg/")
  echo "$pkg: $(find "$REPO/ros2_ws/src/$pkg" -type f | wc -l) files"
done
