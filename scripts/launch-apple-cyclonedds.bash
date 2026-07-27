#!/usr/bin/env bash
set -euo pipefail

export PATH="/opt/homebrew/bin:$HOME/.local/bin:/usr/local/bin:$PATH"

CONTAINER_NAME="${CONTAINER_NAME:-moveit-pro-so101}"
MOVEIT_CONFIG_PACKAGE="${MOVEIT_CONFIG_PACKAGE:-so101_moveit_hardware_config}"
ROBOT_HOST="${SO101_ROBOT_HOST:-so101-pi.tail337068.ts.net}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
MOVEIT_PRO_EXTRA_OVERLAY="${MOVEIT_PRO_EXTRA_OVERLAY-}"
MOVEIT_PRO_EXTRA_PREFIX="${MOVEIT_PRO_EXTRA_PREFIX-}"

if [[ "$MOVEIT_CONFIG_PACKAGE" == "so101_moveit_hardware_config" ]]; then
  ROS2DDS_ENDPOINT="${SO101_ROS2DDS_ENDPOINT:-tcp/${ROBOT_HOST}:7448}"
else
  ROS2DDS_ENDPOINT="${SO101_ROS2DDS_ENDPOINT-}"
fi

command -v container >/dev/null || {
  echo "error: Apple container CLI is unavailable" >&2
  exit 1
}

container stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
container start "$CONTAINER_NAME" >/dev/null

container_user=""
for _ in $(seq 1 60); do
  # USERNAME is intentionally expanded in the container, not by this shell.
  # shellcheck disable=SC2016
  container_user="$(container exec "$CONTAINER_NAME" sh -c 'echo "$USERNAME"' 2>/dev/null)"
  if [[ -n "$container_user" ]] &&
    container exec "$CONTAINER_NAME" id "$container_user" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

container exec "$CONTAINER_NAME" id "$container_user" >/dev/null 2>&1 || {
  echo "error: container user was not ready" >&2
  exit 1
}

container_home="/home/${container_user}"
cyclonedds_config="${SO101_CYCLONEDDS_CONFIG:-${container_home}/user_ws/scripts/cyclonedds-local.xml}"
container exec --interactive --user root "$CONTAINER_NAME" bash -s -- "$container_user" <<'CLEANUP'
set -euo pipefail
target_user="$1"
mapfile -t target_pids < <(ps -o pid= -u "$target_user" | awk '$1 != 1 {print $1}')
if (( ${#target_pids[@]} )); then
  kill -TERM "${target_pids[@]}" 2>/dev/null || true
  sleep 2
fi
mapfile -t target_pids < <(ps -o pid= -u "$target_user" | awk '$1 != 1 {print $1}')
if (( ${#target_pids[@]} )); then
  kill -KILL "${target_pids[@]}" 2>/dev/null || true
fi
CLEANUP

# The workspace path and HOME are intentionally expanded in the container.
# shellcheck disable=SC2016
container exec \
  --user "$container_user" \
  --env "HOME=${container_home}" \
  "$CONTAINER_NAME" \
  bash -c '
    source /opt/ros/jazzy/setup.bash
    source /opt/overlay_ws/install/setup.bash
    cd "$HOME/user_ws"
    colcon build --symlink-install --packages-select \
      so101_vla_adapter \
      so101_camera_bridge \
      so101_moveit_config \
      so101_moveit_hardware_config
  '

container exec --detach \
  --user "$container_user" \
  --env "HOME=${container_home}" \
  --env "MOVEIT_CONFIG_PACKAGE=${MOVEIT_CONFIG_PACKAGE}" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env "SO101_ROBOT_HOST=${ROBOT_HOST}" \
  --env "SO101_ROS2DDS_ENDPOINT=${ROS2DDS_ENDPOINT}" \
  --env "MOVEIT_PRO_EXTRA_OVERLAY=${MOVEIT_PRO_EXTRA_OVERLAY}" \
  --env "MOVEIT_PRO_EXTRA_PREFIX=${MOVEIT_PRO_EXTRA_PREFIX}" \
  --env "MOVEIT_TLS_CERT_DIR=${MOVEIT_TLS_CERT_DIR-}" \
  --env "MOVEIT_TLS_CERT_FILE=${MOVEIT_TLS_CERT_FILE-}" \
  --env "MOVEIT_TLS_KEY_FILE=${MOVEIT_TLS_KEY_FILE-}" \
  "$CONTAINER_NAME" \
  bash -c "exec bash '${container_home}/user_ws/scripts/start-cyclonedds-backend.bash' >'${container_home}/cyclonedds_backend.log' 2>&1"

controller_manager_ready() {
  if [[ "$MOVEIT_CONFIG_PACKAGE" != "so101_moveit_hardware_config" ]]; then
    return 0
  fi

  # Environment variables in this command are intentionally expanded in the container.
  # shellcheck disable=SC2016
  container exec \
    --user "$container_user" \
    --env "HOME=${container_home}" \
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
    --env "SO101_CYCLONEDDS_CONFIG=${cyclonedds_config}" \
    --env "MOVEIT_PRO_EXTRA_OVERLAY=${MOVEIT_PRO_EXTRA_OVERLAY}" \
    "$CONTAINER_NAME" \
    bash -c '
      set -eo pipefail
      source /opt/ros/jazzy/setup.bash
      source /opt/overlay_ws/install/setup.bash
      source "$HOME/user_ws/install/setup.bash"
      if [[ -n "${MOVEIT_PRO_EXTRA_OVERLAY:-}" ]]; then
        source "$MOVEIT_PRO_EXTRA_OVERLAY"
      fi
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export ROS_DOMAIN_ID
      export CYCLONEDDS_URI="file://${SO101_CYCLONEDDS_CONFIG}"
      export ROS2CLI_NO_DAEMON=1
      timeout 5 ros2 service call \
        /controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers \
        "{}" >/dev/null 2>&1
    '
}

for _ in $(seq 1 60); do
  bridge_ready=true
  if [[ -n "$ROS2DDS_ENDPOINT" ]] &&
    ! container exec "$CONTAINER_NAME" pgrep -f 'zenoh-bridge-ros2dds' >/dev/null 2>&1; then
    bridge_ready=false
  fi
  if [[ "$bridge_ready" == true ]] &&
    curl -kfsS -m 3 https://localhost:3200/health 2>/dev/null |
      grep -q '"status":"ok"' &&
    controller_manager_ready; then
    if [[ -n "$ROS2DDS_ENDPOINT" ]]; then
      echo "${MOVEIT_CONFIG_PACKAGE} is running on local CycloneDDS through the ROS2DDS bridge at ${ROS2DDS_ENDPOINT}."
    else
      echo "${MOVEIT_CONFIG_PACKAGE} is running on isolated local CycloneDDS."
    fi
    exit 0
  fi
  sleep 5
done

echo "error: timed out waiting for MoveIt Pro health and controller-manager service readiness" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/agent.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/drivers.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/ros2dds_bridge.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/cyclonedds_backend.log" >&2
exit 1
