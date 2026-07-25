#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE"

pip_install_args=(--user)
if python3 -m pip help install | grep -q -- "--break-system-packages"; then
  pip_install_args+=(--break-system-packages)
fi

if ! command -v vcs >/dev/null 2>&1; then
  python3 -m pip install "${pip_install_args[@]}" vcstool
fi

if [[ ! -d src/topic_based_ros2_control/.git ]]; then
  vcs import src < so101.repos
fi

python3 -m pip install \
  "${pip_install_args[@]}" \
  --requirement requirements.txt

if command -v install_ros_dependencies.bash >/dev/null 2>&1; then
  install_ros_dependencies.bash \
    --skip-keys "ament_python ament_pytest velocity_force_controller"
fi

echo "SO-101 workspace dependencies are ready."
