import hashlib
import hmac
from pathlib import Path

import pytest

from so101_zenoh_ros.protocol import (
    AUTH_DOMAIN,
    CALIBRATION_SHA256,
    JOINT_NAMES,
    MAPPING_ID,
    ProtocolError,
    authenticated_command,
    canonical_from_registered,
    hardware_marker_is_valid,
    registered_from_canonical,
    slew_target_from_feedback,
    validate_capabilities,
)


def capabilities():
    fields = {
        "rotation_rad": [-1.0, 1.0],
        "pitch_rad": [-0.2, 3.3],
        "elbow_rad": [-0.1, 3.2],
        "wrist_pitch_rad": [-1.7, 1.7],
        "wrist_roll_rad": [-1.5, 4.6],
        "jaw_travel_m": [0.0, 0.027],
    }
    joints = {
        field: {
            "motor": motor,
            "scale_deg_per_rad": 57.29577951308232,
            "offset_deg": 0.0,
        }
        for field, motor in {
            "rotation_rad": "shoulder_pan",
            "pitch_rad": "shoulder_lift",
            "elbow_rad": "elbow_flex",
            "wrist_pitch_rad": "wrist_flex",
            "wrist_roll_rad": "wrist_roll",
        }.items()
    }
    return {
        "schema": "so101.motion.capabilities.v1",
        "robot_id": "so101_pi_follower",
        "hardware_arm": "right",
        "calibration_sha256": CALIBRATION_SHA256,
        "mapping": {
            "id": MAPPING_ID,
            "jaw_travel_m": 0.027,
            "joints": joints,
        },
        "command_auth": {
            "required": True,
            "schema": "so101.motion.authenticated-command.v1",
            "domain": "so101.motion.hmac.v1",
            "key_id": "operator-mac-v1",
        },
        "sim_limits": fields,
        "max_step_deg": {
            "shoulder_pan": 2.0,
            "shoulder_lift": 2.0,
            "elbow_flex": 2.0,
            "wrist_flex": 2.0,
            "wrist_roll": 2.0,
        },
        "max_step_gripper_pct": 4.0,
        "max_following_error_deg": {
            "shoulder_pan": 4.0,
            "shoulder_lift": 4.0,
            "elbow_flex": 5.0,
            "wrist_flex": 4.0,
            "wrist_roll": 5.0,
        },
        "max_following_error_gripper_pct": 8.0,
    }


def canonical_fixture():
    return {
        "rotation_rad": -0.2,
        "pitch_rad": 1.1,
        "elbow_rad": 2.2,
        "wrist_pitch_rad": 0.4,
        "wrist_roll_rad": 1.5,
        "jaw_travel_m": 0.02,
    }


def test_registration_round_trip_matches_all_six_joint_names():
    canonical = canonical_fixture()
    registered = registered_from_canonical(canonical)
    assert tuple(registered) == JOINT_NAMES
    assert canonical_from_registered(registered) == pytest.approx(canonical)


def test_capability_identity_is_pinned():
    value = capabilities()
    validate_capabilities(value)
    value["calibration_sha256"] = "0" * 64
    with pytest.raises(ProtocolError, match="calibration_sha256"):
        validate_capabilities(value)


def test_authenticated_command_matches_bridge_message_contract():
    auth_key = bytes.fromhex("11" * 32)
    command = authenticated_command(
        {"schema": "so101.motion.stop.command.v1", "reason": "test"},
        "moveit-test-1",
        1234,
        500,
        "operator-mac-v1",
        auth_key,
    )
    message = "\n".join(
        [
            AUTH_DOMAIN,
            command["key_id"],
            command["id"],
            str(command["issued_at_ms"]),
            str(command["ttl_ms"]),
            command["body"],
        ]
    ).encode()
    assert command["signature"] == hmac.new(
        auth_key, message, hashlib.sha256
    ).hexdigest()


def test_slew_is_bounded_by_step_and_feedback_envelopes():
    caps = capabilities()
    previous = canonical_fixture()
    measured = canonical_fixture()
    desired = {field: value + 1.0 for field, value in previous.items()}
    result = slew_target_from_feedback(desired, previous, measured, caps)
    assert result["rotation_rad"] - previous["rotation_rad"] == pytest.approx(
        2.0 / 57.29577951308232
    )
    assert result["jaw_travel_m"] - previous["jaw_travel_m"] == pytest.approx(
        0.04 * 0.027
    )


def test_hardware_marker_must_be_bounded_and_unexpired(tmp_path: Path):
    marker = tmp_path / "enabled"
    marker.write_text(
        "schema=so101.moveit.hardware-mode.v1\n"
        "enabled_at=2026-07-24T12:00:00Z\n"
        "expires_at_epoch_seconds=1300\n"
        "duration_seconds=300\n",
        encoding="utf-8",
    )
    assert hardware_marker_is_valid(marker, now_epoch_seconds=1000)
    assert not hardware_marker_is_valid(marker, now_epoch_seconds=1300)
    assert not hardware_marker_is_valid(marker, now_epoch_seconds=900)


def test_hardware_marker_fails_closed_on_unrecognized_content(tmp_path: Path):
    marker = tmp_path / "enabled"
    marker.write_text("enabled=true\n", encoding="utf-8")
    assert not hardware_marker_is_valid(marker, now_epoch_seconds=1000)
