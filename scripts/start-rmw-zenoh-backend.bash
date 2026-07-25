#!/usr/bin/env bash
set -eo pipefail

LOCAL_ROUTER_ENDPOINT="${SO101_ZENOH_LOCAL_ENDPOINT:-tcp/127.0.0.1:7447}"
ROBOT_HOST="${SO101_ROBOT_HOST:-so101-pi.tail337068.ts.net}"
REMOTE_ROUTER_ENDPOINT="${SO101_ZENOH_REMOTE_ENDPOINT-tcp/${ROBOT_HOST}:7447}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
CONTAINER_HOME="${HOME:-/home/moveit-pro-user}"

endpoint_pattern='^(tcp|tls)/[A-Za-z0-9._:-]+:[0-9]+$'
[[ "$LOCAL_ROUTER_ENDPOINT" =~ $endpoint_pattern ]] || {
  echo "error: invalid local Zenoh endpoint: $LOCAL_ROUTER_ENDPOINT" >&2
  exit 1
}
if [[ -n "$REMOTE_ROUTER_ENDPOINT" && ! "$REMOTE_ROUTER_ENDPOINT" =~ $endpoint_pattern ]]; then
  echo "error: invalid remote Zenoh endpoint: $REMOTE_ROUTER_ENDPOINT" >&2
  exit 1
fi

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /opt/overlay_ws/install/setup.bash
# shellcheck source=/dev/null
source "$CONTAINER_HOME/user_ws/install/setup.bash"
set -u

router_override="listen/endpoints=[\"${LOCAL_ROUTER_ENDPOINT}\"];scouting/multicast/enabled=false"
if [[ -n "$REMOTE_ROUTER_ENDPOINT" ]]; then
  router_override+=";connect/endpoints=[\"${REMOTE_ROUTER_ENDPOINT}\"]"
fi

client_override="mode=\"client\";connect/endpoints=[\"${LOCAL_ROUTER_ENDPOINT}\"];scouting/multicast/enabled=false"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID
export MUJOCO_GL=egl

cleanup() {
  local status=$?
  kill "${driver_pid:-}" "${router_pid:-}" 2>/dev/null || true
  wait "${driver_pid:-}" "${router_pid:-}" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

ZENOH_CONFIG_OVERRIDE="$router_override" \
  ros2 run rmw_zenoh_cpp rmw_zenohd >"$CONTAINER_HOME/rmw_zenoh_router.log" 2>&1 &
router_pid=$!

sleep 2
kill -0 "$router_pid" 2>/dev/null || {
  echo "error: rmw_zenoh router exited during startup" >&2
  tail -50 "$CONTAINER_HOME/rmw_zenoh_router.log" >&2
  exit 1
}

export ZENOH_CONFIG_OVERRIDE="$client_override"

robot.app >"$CONTAINER_HOME/drivers.log" 2>&1 &
driver_pid=$!

sleep 18
kill -0 "$driver_pid" 2>/dev/null || {
  echo "error: robot driver exited during startup" >&2
  tail -50 "$CONTAINER_HOME/drivers.log" >&2
  exit 1
}

ros2 launch "$MOVEIT_CONFIG_PACKAGE" agent_bridge.launch.xml \
  enable_rosbridge:=true \
  rosbridge_port:=3204 >"$CONTAINER_HOME/agent.log" 2>&1
