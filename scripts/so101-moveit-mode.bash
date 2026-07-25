#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPS_REPO="${SO101_OPS_REPO:-/Users/noah/mujoco/so101-robot-ops}"
RUNTIME_DIR="${WORKSPACE}/.runtime"
ENABLE_FILE="${RUNTIME_DIR}/hardware-commands-enabled"
KEY_FILE="${RUNTIME_DIR}/control-auth.key"
TOGGLE="${OPS_REPO}/zenoh/tools/set_motion_writes.sh"
CHECK="${OPS_REPO}/scripts/check-robot.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/so101-moveit-mode.bash status
  scripts/so101-moveit-mode.bash enable --duration-seconds N --confirm ENABLE-SO101-MOVEIT-PRO
  scripts/so101-moveit-mode.bash disable

Enable is intentionally short-lived. It stages the existing Keychain-backed
HMAC key into the git-ignored workspace runtime directory, verifies the robot
without motion, opens the Pi's in-memory write window, then enables the
MoveIt ROS-to-Zenoh adapter. Disable removes local command authority first and
then invokes the Pi's fail-safe torque-off/write-window close operation.
EOF
}

require_sources() {
  [[ -x "$TOGGLE" ]] || {
    echo "error: missing write toggle: $TOGGLE" >&2
    exit 1
  }
  [[ -x "$CHECK" ]] || {
    echo "error: missing robot check: $CHECK" >&2
    exit 1
  }
}

status() {
  if [[ -f "$ENABLE_FILE" ]]; then
    echo "MoveIt adapter commands: enabled locally"
  else
    echo "MoveIt adapter commands: disabled locally"
  fi
  if [[ -f "$KEY_FILE" ]]; then
    echo "MoveIt control key: staged"
  else
    echo "MoveIt control key: not staged"
  fi
  "$CHECK"
}

enable_mode() {
  local duration=""
  local confirmation=""
  shift
  while (( $# )); do
    case "$1" in
      --duration-seconds)
        (( $# >= 2 )) || {
          echo "error: --duration-seconds requires a value" >&2
          exit 2
        }
        duration="${2:-}"
        shift 2
        ;;
      --confirm)
        (( $# >= 2 )) || {
          echo "error: --confirm requires a value" >&2
          exit 2
        }
        confirmation="${2:-}"
        shift 2
        ;;
      *)
        echo "error: unknown argument: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
  done

  if [[ ! "$duration" =~ ^[0-9]+$ ]] ||
    (( duration < 5 || duration > 300 )); then
    echo "error: --duration-seconds must be an integer from 5 through 300" >&2
    exit 2
  fi
  [[ "$confirmation" == "ENABLE-SO101-MOVEIT-PRO" ]] || {
    echo "error: exact confirmation is required: ENABLE-SO101-MOVEIT-PRO" >&2
    exit 2
  }

  "$CHECK"
  "$WORKSPACE/scripts/stage-control-key.bash" >/dev/null
  local enabled_at
  local expires_at_epoch_seconds
  enabled_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  expires_at_epoch_seconds="$(( $(date +%s) + duration ))"
  "$TOGGLE" enable \
    --duration-seconds "$duration" \
    --confirm ENABLE-SO101-RUNTIME-MOTION

  toggle_open=true
  rollback_enable() {
    if [[ "$toggle_open" == true ]]; then
      rm -f "$ENABLE_FILE"
      "$TOGGLE" disable >/dev/null 2>&1 || true
      rm -f "$KEY_FILE"
    fi
  }
  trap rollback_enable ERR INT TERM

  umask 077
  mkdir -p "$RUNTIME_DIR"
  printf 'schema=so101.moveit.hardware-mode.v1\nenabled_at=%s\nexpires_at_epoch_seconds=%s\nduration_seconds=%s\n' \
    "$enabled_at" "$expires_at_epoch_seconds" "$duration" > "$ENABLE_FILE"
  chmod 0600 "$ENABLE_FILE"
  toggle_open=false
  trap - ERR INT TERM
  echo "MoveIt Pro hardware mode enabled for at most ${duration}s."
}

disable_mode() {
  local toggle_status=0
  mkdir -p "$RUNTIME_DIR"
  rm -f "$ENABLE_FILE"
  "$TOGGLE" disable || toggle_status=$?
  rm -f "$KEY_FILE"
  if (( toggle_status != 0 )); then
    echo "error: local MoveIt authority was removed, but closing the Pi write window failed" >&2
    return "$toggle_status"
  fi
  echo "MoveIt Pro hardware mode disabled; Pi write authority is closed."
}

require_sources
case "${1:-}" in
  status)
    status
    ;;
  enable)
    enable_mode "$@"
    ;;
  disable)
    disable_mode
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
