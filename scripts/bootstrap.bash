#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE"

if command -v install_ros_dependencies.bash >/dev/null 2>&1; then
  install_ros_dependencies.bash \
    --skip-keys "ament_python ament_pytest velocity_force_controller"
else
  echo "install_ros_dependencies.bash is unavailable; relying on the MoveIt Pro image's ROS dependencies."
fi

echo "SO-101 workspace dependencies are ready."
