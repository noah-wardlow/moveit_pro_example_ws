#!/usr/bin/env bash
set -euo pipefail

ROBOT_HOST="${SO101_ROBOT_HOST:-so101-pi.tail337068.ts.net}"
REMOTE_ROUTER_ENDPOINT="${SO101_ZENOH_REMOTE_ENDPOINT:-tcp/${ROBOT_HOST}:7447}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

endpoint_pattern='^(tcp|tls)/[A-Za-z0-9._:-]+:[0-9]+$'
if [[ ! "$REMOTE_ROUTER_ENDPOINT" =~ $endpoint_pattern ]]; then
  echo "error: invalid SO101 Zenoh endpoint: $REMOTE_ROUTER_ENDPOINT" >&2
  exit 1
fi

command -v moveit_pro >/dev/null || {
  echo "error: moveit_pro CLI is unavailable" >&2
  exit 1
}

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID
export SO101_ROBOT_HOST="$ROBOT_HOST"
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${REMOTE_ROUTER_ENDPOINT}\"];scouting/multicast/enabled=false"

exec moveit_pro run -c so101_moveit_hardware_config "$@"
