"""Pure protocol and coordinate helpers for the SO-101 ROS adapter."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import math
from pathlib import Path
from typing import Any, Mapping


AUTH_SCHEMA = "so101.motion.authenticated-command.v1"
AUTH_DOMAIN = "so101.motion.hmac.v1"
ROBOT_ID = "so101_pi_follower"
HARDWARE_ARM = "right"
MAPPING_ID = "xlerobot-overhead-v2"
CALIBRATION_SHA256 = (
    "409228de0b511c46e44c5a4e5eeece5da09d6d58329b93889d611f75213c08b9"
)

JOINT_TO_FIELD = {
    "Rotation_R": "rotation_rad",
    "Pitch_R": "pitch_rad",
    "Elbow_R": "elbow_rad",
    "Wrist_Pitch_R": "wrist_pitch_rad",
    "Wrist_Roll_R": "wrist_roll_rad",
    "Jaw_R": "jaw_travel_m",
}
JOINT_NAMES = tuple(JOINT_TO_FIELD)
FIELD_TO_JOINT = {field: joint for joint, field in JOINT_TO_FIELD.items()}
FIELD_TO_MOTOR = {
    "rotation_rad": "shoulder_pan",
    "pitch_rad": "shoulder_lift",
    "elbow_rad": "elbow_flex",
    "wrist_pitch_rad": "wrist_flex",
    "wrist_roll_rad": "wrist_roll",
}
REGISTRATION_OFFSETS = {
    "Rotation_R": 0.0,
    "Pitch_R": 0.004203673205103398,
    "Elbow_R": -0.1107963267948966,
    "Wrist_Pitch_R": -0.00000006006107966527452,
    "Wrist_Roll_R": 0.0,
    "Jaw_R": 0.0,
}


class ProtocolError(RuntimeError):
    """A fail-closed identity, schema, or coordinate-contract error."""


def hardware_marker_is_valid(path: Path, *, now_epoch_seconds: float) -> bool:
    """Accept only a bounded, unexpired marker written by the mode switch."""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return False
    values: dict[str, str] = {}
    for line in lines:
        key, separator, value = line.partition("=")
        if not separator or key in values:
            return False
        values[key] = value
    if values.get("schema") != "so101.moveit.hardware-mode.v1":
        return False
    try:
        duration = int(values["duration_seconds"])
        expires_at = int(values["expires_at_epoch_seconds"])
    except (KeyError, ValueError):
        return False
    if not 5 <= duration <= 300:
        return False
    remaining = expires_at - float(now_epoch_seconds)
    return 0.0 < remaining <= duration + 5.0


def payload_bytes(value: Any) -> bytes:
    payload = value.payload
    if hasattr(payload, "to_bytes"):
        return payload.to_bytes()
    return bytes(payload)


def authenticated_command(
    body: Mapping[str, Any],
    command_id: str,
    issued_at_ms: int,
    ttl_ms: int,
    key_id: str,
    auth_key: bytes,
) -> dict[str, Any]:
    body_bytes = json.dumps(
        dict(body),
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    body_encoded = base64.urlsafe_b64encode(body_bytes).decode("ascii").rstrip("=")
    command: dict[str, Any] = {
        "schema": AUTH_SCHEMA,
        "id": command_id,
        "issued_at_ms": issued_at_ms,
        "ttl_ms": ttl_ms,
        "key_id": key_id,
        "body": body_encoded,
    }
    message = "\n".join(
        [
            AUTH_DOMAIN,
            key_id,
            command_id,
            str(issued_at_ms),
            str(ttl_ms),
            body_encoded,
        ]
    ).encode("utf-8")
    command["signature"] = hmac.new(auth_key, message, hashlib.sha256).hexdigest()
    return command


def validate_capabilities(capabilities: Mapping[str, Any]) -> None:
    expected = {
        "schema": "so101.motion.capabilities.v1",
        "robot_id": ROBOT_ID,
        "hardware_arm": HARDWARE_ARM,
        "calibration_sha256": CALIBRATION_SHA256,
    }
    for key, value in expected.items():
        if capabilities.get(key) != value:
            raise ProtocolError(
                f"capabilities {key} mismatch: expected {value!r}, "
                f"got {capabilities.get(key)!r}"
            )
    mapping = capabilities.get("mapping")
    if not isinstance(mapping, Mapping) or mapping.get("id") != MAPPING_ID:
        raise ProtocolError("capabilities mapping identity does not match MoveIt config")
    command_auth = capabilities.get("command_auth")
    if (
        not isinstance(command_auth, Mapping)
        or command_auth.get("required") is not True
        or command_auth.get("schema") != AUTH_SCHEMA
        or command_auth.get("domain") != AUTH_DOMAIN
        or not command_auth.get("key_id")
    ):
        raise ProtocolError("capabilities command authentication contract is invalid")
    sim_limits = capabilities.get("sim_limits")
    for field in FIELD_TO_JOINT:
        limits = sim_limits.get(field) if isinstance(sim_limits, Mapping) else None
        if (
            not isinstance(limits, list)
            or len(limits) != 2
            or not all(isinstance(value, (int, float)) for value in limits)
        ):
            raise ProtocolError(f"capabilities are missing limits for {field}")


def registered_from_canonical(canonical: Mapping[str, float]) -> dict[str, float]:
    return {
        joint: float(canonical[field]) + REGISTRATION_OFFSETS[joint]
        for joint, field in JOINT_TO_FIELD.items()
    }


def canonical_from_registered(registered: Mapping[str, float]) -> dict[str, float]:
    return {
        field: float(registered[joint]) - REGISTRATION_OFFSETS[joint]
        for joint, field in JOINT_TO_FIELD.items()
    }


def clamp_canonical_to_limits(
    target: Mapping[str, float],
    capabilities: Mapping[str, Any],
) -> dict[str, float]:
    result: dict[str, float] = {}
    for field in FIELD_TO_JOINT:
        low, high = capabilities["sim_limits"][field]
        value = float(target[field])
        result[field] = min(max(value, min(low, high)), max(low, high))
    return result


def _clamp_step(desired: float, reference: float, limit: float) -> float:
    return min(max(desired, reference - limit), reference + limit)


def slew_target_from_feedback(
    desired: Mapping[str, float],
    previous_applied: Mapping[str, float],
    measured: Mapping[str, float],
    capabilities: Mapping[str, Any],
) -> dict[str, float]:
    """Apply the same step and encoder-following envelopes as the dashboard."""
    result: dict[str, float] = {}
    mapping = capabilities["mapping"]
    for field in FIELD_TO_JOINT:
        if field == "jaw_travel_m":
            travel = float(mapping["jaw_travel_m"])
            step_limit = (
                float(capabilities["max_step_gripper_pct"]) / 100.0 * travel
            )
            following_limit = (
                float(capabilities["max_following_error_gripper_pct"])
                / 100.0
                * travel
                * 0.90
            )
        else:
            motor = FIELD_TO_MOTOR[field]
            scale = abs(float(mapping["joints"][field]["scale_deg_per_rad"]))
            step_limit = float(capabilities["max_step_deg"][motor]) / scale
            following_limit = (
                float(capabilities["max_following_error_deg"][motor])
                / scale
                * 0.90
            )
        stepped = _clamp_step(
            float(desired[field]),
            float(previous_applied[field]),
            step_limit,
        )
        result[field] = _clamp_step(
            stepped,
            float(measured[field]),
            following_limit,
        )
    return clamp_canonical_to_limits(result, capabilities)


def telemetry_is_fresh(
    telemetry: Mapping[str, Any],
    *,
    now_seconds: float,
    max_age_seconds: float,
) -> bool:
    timestamp = telemetry.get("_received_monotonic")
    return (
        isinstance(timestamp, (int, float))
        and math.isfinite(timestamp)
        and 0.0 <= now_seconds - float(timestamp) <= max_age_seconds
    )
