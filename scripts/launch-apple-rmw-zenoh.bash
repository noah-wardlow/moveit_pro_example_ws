#!/usr/bin/env bash
set -euo pipefail

export PATH="/opt/homebrew/bin:$HOME/.local/bin:/usr/local/bin:$PATH"

CONTAINER_NAME="${CONTAINER_NAME:-moveit-pro-so101}"
REMOTE_ROUTER_ENDPOINT="${SO101_ZENOH_REMOTE_ENDPOINT:-}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

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

container exec --detach \
  --user "$container_user" \
  --env "HOME=${container_home}" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  --env "SO101_ZENOH_REMOTE_ENDPOINT=${REMOTE_ROUTER_ENDPOINT}" \
  "$CONTAINER_NAME" \
  bash -c "exec bash '${container_home}/user_ws/scripts/start-rmw-zenoh-backend.bash' >'${container_home}/rmw_zenoh_backend.log' 2>&1"

for _ in $(seq 1 60); do
  if container exec "$CONTAINER_NAME" pgrep -f '/rmw_zenoh_cpp/rmw_zenohd' >/dev/null 2>&1 &&
    curl -kfsS -m 3 https://localhost:3200/health 2>/dev/null |
      grep -q '"status":"ok"'; then
    echo "MoveIt Pro is running with rmw_zenoh_cpp (ROS domain ${ROS_DOMAIN_ID})."
    exit 0
  fi
  sleep 5
done

echo "error: timed out waiting for MoveIt Pro health" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/agent.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/drivers.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/rmw_zenoh_router.log" >&2
echo "inspect: container exec $CONTAINER_NAME tail -50 $container_home/rmw_zenoh_backend.log" >&2
exit 1
