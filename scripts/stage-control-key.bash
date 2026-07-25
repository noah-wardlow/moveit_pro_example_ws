#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME_DIR="${WORKSPACE}/.runtime"
KEY_FILE="${RUNTIME_DIR}/control-auth.key"
KEYCHAIN_SERVICE="${SO101_MOVEIT_KEYCHAIN_SERVICE:-so101-motion-control-key}"

command -v security >/dev/null 2>&1 || {
  echo "error: macOS Keychain command 'security' is unavailable" >&2
  exit 1
}

control_key="$(security find-generic-password -s "$KEYCHAIN_SERVICE" -w)"
if [[ ! "$control_key" =~ ^[0-9a-f]{64}$ ]]; then
  echo "error: Keychain item '$KEYCHAIN_SERVICE' is not a 256-bit lowercase hex key" >&2
  exit 1
fi

umask 077
mkdir -p "$RUNTIME_DIR"
printf '%s\n' "$control_key" > "$KEY_FILE"
chmod 0600 "$KEY_FILE"
echo "$KEY_FILE"
