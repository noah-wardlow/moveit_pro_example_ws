import threading
import time

from so101_zenoh_ros.bridge_node import So101ZenohBridge


def bridge_without_ros(tmp_path):
    bridge = So101ZenohBridge.__new__(So101ZenohBridge)
    bridge._commands_enabled_file = tmp_path / "hardware-commands-enabled"
    bridge._control_key_file = tmp_path / "control-auth.key"
    bridge._lock = threading.RLock()
    bridge._latest_safety = None
    bridge._refresh_capabilities = lambda: {"writes_enabled": True}
    bridge._query_json = lambda _key: {
        "joints": {"test": "joints"},
        "safety": {"writes_enabled": True},
    }
    bridge._validate_status = lambda _status: None
    bridge._store_joints = lambda _joints: None
    bridge._motion_preconditions = lambda _capabilities: None
    return bridge


def enable_local_hardware_mode(bridge):
    expires_at = int(time.time()) + 60
    bridge._commands_enabled_file.write_text(
        "schema=so101.moveit.hardware-mode.v1\n"
        "enabled_at=2026-07-25T12:00:00Z\n"
        f"expires_at_epoch_seconds={expires_at}\n"
        "duration_seconds=60\n",
        encoding="utf-8",
    )
    bridge._control_key_file.write_text("11" * 32, encoding="utf-8")


def test_hardware_readiness_explains_how_to_enable_local_mode(tmp_path):
    bridge = bridge_without_ros(tmp_path)

    blocker = bridge._hardware_control_blocker()

    assert blocker is not None
    assert "so101-moveit-mode.bash enable" in blocker


def test_hardware_readiness_requires_a_valid_control_key(tmp_path):
    bridge = bridge_without_ros(tmp_path)
    enable_local_hardware_mode(bridge)
    bridge._control_key_file.write_text("not-a-key", encoding="utf-8")

    blocker = bridge._hardware_control_blocker()

    assert blocker == "MoveIt control key is not 256-bit lowercase hex"


def test_hardware_readiness_reports_live_safety_blocker(tmp_path):
    bridge = bridge_without_ros(tmp_path)
    enable_local_hardware_mode(bridge)
    bridge._motion_preconditions = lambda _capabilities: (
        "the Pi write window is closed"
    )

    blocker = bridge._hardware_control_blocker()

    assert blocker == "the Pi write window is closed"


def test_hardware_readiness_succeeds_when_all_gates_are_ready(tmp_path):
    bridge = bridge_without_ros(tmp_path)
    enable_local_hardware_mode(bridge)

    assert bridge._hardware_control_blocker() is None
