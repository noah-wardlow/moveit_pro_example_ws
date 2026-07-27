#!/usr/bin/env bash
set -eo pipefail

ROS2DDS_ENDPOINT="${SO101_ROS2DDS_ENDPOINT-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER_HOME="${HOME:-/home/moveit-pro-user}"
CYCLONEDDS_CONFIG="${SO101_CYCLONEDDS_CONFIG:-${CONTAINER_HOME}/user_ws/scripts/cyclonedds-local.xml}"
ROS2DDS_CONFIG="${SO101_ROS2DDS_CONFIG:-${CONTAINER_HOME}/user_ws/scripts/ros2dds-moveit-control.json5}"

endpoint_pattern='^(tcp|tls)/[A-Za-z0-9._:-]+:[0-9]+$'
if [[ -n "$ROS2DDS_ENDPOINT" && ! "$ROS2DDS_ENDPOINT" =~ $endpoint_pattern ]]; then
  echo "error: invalid ROS2DDS endpoint: $ROS2DDS_ENDPOINT" >&2
  exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /opt/overlay_ws/install/setup.bash
# shellcheck source=/dev/null
source "$CONTAINER_HOME/user_ws/install/setup.bash"
if [[ -n "${MOVEIT_PRO_EXTRA_OVERLAY:-}" ]]; then
  # shellcheck source=/dev/null
  source "$MOVEIT_PRO_EXTRA_OVERLAY"
fi
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID
export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
export MUJOCO_GL=egl
unset ZENOH_CONFIG_OVERRIDE

cleanup() {
  local status=$?
  kill "${driver_pid:-}" "${bridge_pid:-}" 2>/dev/null || true
  wait "${driver_pid:-}" "${bridge_pid:-}" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

if [[ -n "$ROS2DDS_ENDPOINT" ]]; then
  bridge_binary="$("${CONTAINER_HOME}/user_ws/scripts/install-ros2dds-bridge.bash")"
  "$bridge_binary" \
    --config "$ROS2DDS_CONFIG" \
    client \
    --connect "$ROS2DDS_ENDPOINT" \
    --no-multicast-scouting \
    --domain "$ROS_DOMAIN_ID" \
    >"$CONTAINER_HOME/ros2dds_bridge.log" 2>&1 &
  bridge_pid=$!

  sleep 2
  kill -0 "$bridge_pid" 2>/dev/null || {
    echo "error: ROS2DDS bridge exited during startup" >&2
    tail -50 "$CONTAINER_HOME/ros2dds_bridge.log" >&2
    exit 1
  }
fi

robot.app >"$CONTAINER_HOME/drivers.log" 2>&1 &
driver_pid=$!

sleep 18
kill -0 "$driver_pid" 2>/dev/null || {
  echo "error: robot driver exited during startup" >&2
  tail -50 "$CONTAINER_HOME/drivers.log" >&2
  exit 1
}

if [[ "${MOVEIT_CONFIG_PACKAGE:-}" == "so101_moveit_hardware_config" ]]; then
  controller_manager_ready=false
  for _ in $(seq 1 30); do
    if timeout 5 ros2 service call \
      /controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers \
      "{}" >/dev/null 2>&1; then
      controller_manager_ready=true
      break
    fi
    sleep 2
  done
  if [[ "$controller_manager_ready" != true ]]; then
    echo "error: controller manager was discovered but did not answer a typed service request" >&2
    tail -50 "$CONTAINER_HOME/ros2dds_bridge.log" >&2
    exit 1
  fi
fi

ros2 launch "$MOVEIT_CONFIG_PACKAGE" agent_bridge.launch.xml \
  enable_rosbridge:=true \
  rosbridge_port:=3204 >"$CONTAINER_HOME/agent.log" 2>&1
