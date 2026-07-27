#!/usr/bin/env bash
set -euo pipefail

ROBOT_HOST="${SO101_ROBOT_HOST:-so101-pi.tail337068.ts.net}"
ROS2DDS_ENDPOINT="${SO101_ROS2DDS_ENDPOINT:-tcp/${ROBOT_HOST}:7448}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2DDS_CONFIG="${SO101_ROS2DDS_CONFIG:-${SCRIPT_DIR}/ros2dds-moveit-control.json5}"

endpoint_pattern='^(tcp|tls)/[A-Za-z0-9._:-]+:[0-9]+$'
if [[ ! "$ROS2DDS_ENDPOINT" =~ $endpoint_pattern ]]; then
  echo "error: invalid ROS2DDS endpoint: ${ROS2DDS_ENDPOINT}" >&2
  exit 1
fi

command -v moveit_pro >/dev/null || {
  echo "error: moveit_pro CLI is unavailable" >&2
  exit 1
}

bridge_binary="$("${SCRIPT_DIR}/install-ros2dds-bridge.bash")"
state_dir="${XDG_STATE_HOME:-${HOME}/.local/state}/so101-moveit-pro"
mkdir -p "$state_dir"
bridge_log="${state_dir}/ros2dds_bridge.log"

cleanup() {
  local status=$?
  kill "${bridge_pid:-}" 2>/dev/null || true
  wait "${bridge_pid:-}" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

"$bridge_binary" \
  --config "$ROS2DDS_CONFIG" \
  --ros-localhost-only \
  client \
  --connect "$ROS2DDS_ENDPOINT" \
  --no-multicast-scouting \
  --domain "$ROS_DOMAIN_ID" \
  >"$bridge_log" 2>&1 &
bridge_pid=$!

sleep 2
kill -0 "$bridge_pid" 2>/dev/null || {
  echo "error: ROS2DDS bridge exited during startup" >&2
  tail -50 "$bridge_log" >&2
  exit 1
}

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID
export SO101_ROBOT_HOST="$ROBOT_HOST"
export CYCLONEDDS_NETWORK_INTERFACE="lo"
export CYCLONEDDS_PEER_ADDRESSES="127.0.0.1"
export CYCLONEDDS_USE_MULTICAST=false
unset ZENOH_CONFIG_OVERRIDE

moveit_pro run -c so101_moveit_hardware_config "$@"
