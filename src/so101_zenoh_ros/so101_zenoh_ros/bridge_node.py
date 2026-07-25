"""ROS joint-state/command adapter for the existing SO-101 Zenoh supervisor."""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Mapping

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zenoh

from .protocol import (
    CALIBRATION_SHA256,
    HARDWARE_ARM,
    JOINT_NAMES,
    JOINT_TO_FIELD,
    MAPPING_ID,
    ROBOT_ID,
    ProtocolError,
    authenticated_command,
    canonical_from_registered,
    hardware_marker_is_valid,
    payload_bytes,
    registered_from_canonical,
    slew_target_from_feedback,
    telemetry_is_fresh,
    validate_capabilities,
)


ROOT = f"robots/{ROBOT_ID}"
KEYS = {
    "capabilities": f"{ROOT}/capabilities",
    "status": f"{ROOT}/status",
    "joints": f"{ROOT}/state/joints",
    "safety": f"{ROOT}/state/safety",
    "acks": f"{ROOT}/acks/**",
    "lease": f"{ROOT}/commands/lease",
    "jog": f"{ROOT}/commands/jog",
    "stop": f"{ROOT}/commands/stop",
}
RECOVERABLE_JOG_ERRORS = {
    "following_error",
    "invalid_target",
    "soft_limit",
    "step_limit",
    "start_proximity",
    "rate_limit",
    "superseded",
}


class MotionCommandRejected(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code


class So101ZenohBridge(Node):
    """Expose the Pi's native safety boundary as topic-based ros2_control I/O."""

    def __init__(self) -> None:
        super().__init__("so101_zenoh_bridge")
        self.declare_parameter(
            "endpoint",
            os.environ.get("SO101_ZENOH_ENDPOINT", "tcp/100.79.11.87:7447"),
        )
        default_ws = Path(
            os.environ.get("USER_WS", str(Path.home() / "user_ws"))
        )
        self.declare_parameter(
            "control_key_file",
            str(default_ws / ".runtime" / "control-auth.key"),
        )
        self.declare_parameter(
            "commands_enabled_file",
            str(default_ws / ".runtime" / "hardware-commands-enabled"),
        )
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("telemetry_max_age_seconds", 1.0)

        self._endpoint = str(self.get_parameter("endpoint").value)
        self._control_key_file = Path(
            str(self.get_parameter("control_key_file").value)
        )
        self._commands_enabled_file = Path(
            str(self.get_parameter("commands_enabled_file").value)
        )
        self._telemetry_max_age = float(
            self.get_parameter("telemetry_max_age_seconds").value
        )
        command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        if not 1.0 <= command_rate_hz <= 10.0:
            raise ValueError("command_rate_hz must be from 1 through 10 Hz")

        self._lock = threading.RLock()
        self._pending_acks: dict[str, queue.Queue[dict[str, Any]]] = {}
        self._latest_joints: dict[str, Any] | None = None
        self._latest_safety: dict[str, Any] | None = None
        self._capabilities: dict[str, Any] | None = None
        self._latest_registered: dict[str, float] | None = None
        self._desired_registered: dict[str, float] | None = None
        self._previous_registered: dict[str, float] | None = None
        self._previous_telemetry_at: float | None = None
        self._lease: dict[str, Any] | None = None
        self._last_applied: dict[str, float] | None = None
        self._sequence = -1
        self._commands_were_enabled = False
        self._fatal_latched = False
        self._following_error_count = 0
        self._last_log_at: dict[str, float] = {}
        self._client_id = f"moveit-pro-{uuid.uuid4()}"

        self._joint_publisher = self.create_publisher(
            JointState,
            "/so101/joint_states",
            10,
        )
        self._joint_command_subscription = self.create_subscription(
            JointState,
            "/so101/joint_commands",
            self._on_ros_joint_command,
            10,
        )

        config = zenoh.Config()
        config.insert_json5("mode", '"client"')
        config.insert_json5("connect/endpoints", json.dumps([self._endpoint]))
        self._session = zenoh.open(config)
        self._zenoh_subscribers = [
            self._session.declare_subscriber(KEYS["joints"], self._on_joints),
            self._session.declare_subscriber(KEYS["safety"], self._on_safety),
            self._session.declare_subscriber(KEYS["acks"], self._on_ack),
        ]

        capabilities = self._query_json(KEYS["capabilities"])
        validate_capabilities(capabilities)
        status = self._query_json(KEYS["status"])
        self._validate_status(status)
        with self._lock:
            self._capabilities = capabilities
            self._store_joints(status["joints"])
            self._latest_safety = dict(status["safety"])
        self._publish_joint_state()

        self._command_timer = self.create_timer(
            1.0 / command_rate_hz,
            self._on_command_tick,
        )
        self.get_logger().info(
            "SO-101 live telemetry connected at "
            f"{self._endpoint}; commands are fail-closed until "
            f"{self._commands_enabled_file} exists and the Pi write window is open."
        )

    def _log_throttled(
        self,
        level: str,
        key: str,
        message: str,
        period_seconds: float = 5.0,
    ) -> None:
        now = time.monotonic()
        previous = self._last_log_at.get(key, float("-inf"))
        if now - previous < period_seconds:
            return
        self._last_log_at[key] = now
        getattr(self.get_logger(), level)(message)

    def _query_json(self, key: str) -> dict[str, Any]:
        replies = list(self._session.get(key, timeout=5))
        if not replies:
            raise ProtocolError(f"Zenoh query returned no reply for {key}")
        reply = replies[0]
        if not getattr(reply, "ok", None):
            raise ProtocolError(f"Zenoh query failed for {key}: {reply}")
        value = json.loads(payload_bytes(reply.ok).decode("utf-8"))
        if not isinstance(value, dict):
            raise ProtocolError(f"Zenoh query for {key} did not return an object")
        return value

    def _validate_status(self, status: Mapping[str, Any]) -> None:
        if (
            status.get("schema") != "so101.robot.status.v2"
            or status.get("robot_id") != ROBOT_ID
            or status.get("bridge") != "online"
            or not isinstance(status.get("joints"), Mapping)
            or not isinstance(status.get("safety"), Mapping)
        ):
            raise ProtocolError("SO-101 status query failed identity or schema checks")
        self._validate_joints(status["joints"])
        self._validate_safety(status["safety"])

    @staticmethod
    def _validate_joints(joints: Mapping[str, Any]) -> None:
        if (
            joints.get("schema") != "so101.joints.state.v1"
            or joints.get("robot_id") != ROBOT_ID
            or not isinstance(joints.get("sim"), Mapping)
        ):
            raise ProtocolError("SO-101 joint telemetry failed identity checks")
        for field in JOINT_TO_FIELD.values():
            value = joints["sim"].get(field)
            if not isinstance(value, (int, float)):
                raise ProtocolError(f"joint telemetry is missing finite field {field}")

    @staticmethod
    def _validate_safety(safety: Mapping[str, Any]) -> None:
        if (
            safety.get("schema") != "so101.motion.safety.v1"
            or safety.get("robot_id") != ROBOT_ID
        ):
            raise ProtocolError("SO-101 safety telemetry failed identity checks")

    def _store_joints(self, joints: Mapping[str, Any]) -> None:
        self._validate_joints(joints)
        stored = dict(joints)
        stored["_received_monotonic"] = time.monotonic()
        registered = registered_from_canonical(joints["sim"])
        self._latest_joints = stored
        self._latest_registered = registered
        if self._desired_registered is None:
            self._desired_registered = dict(registered)

    def _on_joints(self, sample: Any) -> None:
        try:
            joints = json.loads(payload_bytes(sample).decode("utf-8"))
            with self._lock:
                self._store_joints(joints)
            self._publish_joint_state()
        except Exception as exc:
            self._log_throttled(
                "error",
                "joints_decode",
                f"Rejected SO-101 joint telemetry: {exc}",
            )

    def _on_safety(self, sample: Any) -> None:
        try:
            safety = json.loads(payload_bytes(sample).decode("utf-8"))
            self._validate_safety(safety)
            with self._lock:
                self._latest_safety = dict(safety)
        except Exception as exc:
            self._log_throttled(
                "error",
                "safety_decode",
                f"Rejected SO-101 safety telemetry: {exc}",
            )

    def _on_ack(self, sample: Any) -> None:
        try:
            ack = json.loads(payload_bytes(sample).decode("utf-8"))
            command_id = ack.get("id") if isinstance(ack, dict) else None
            if not isinstance(command_id, str):
                return
            with self._lock:
                pending = self._pending_acks.get(command_id)
            if pending is not None:
                try:
                    pending.put_nowait(ack)
                except queue.Full:
                    pass
        except Exception as exc:
            self._log_throttled(
                "warning",
                "ack_decode",
                f"Ignored malformed SO-101 acknowledgement: {exc}",
            )

    def _publish_joint_state(self) -> None:
        with self._lock:
            registered = (
                dict(self._latest_registered)
                if self._latest_registered is not None
                else None
            )
            previous = (
                dict(self._previous_registered)
                if self._previous_registered is not None
                else None
            )
            now_monotonic = time.monotonic()
            previous_at = self._previous_telemetry_at
            if registered is not None:
                self._previous_registered = dict(registered)
                self._previous_telemetry_at = now_monotonic
        if registered is None:
            return
        elapsed = (
            now_monotonic - previous_at
            if previous is not None and previous_at is not None
            else 0.0
        )
        velocities = [
            (registered[name] - previous[name]) / elapsed
            if elapsed > 1e-6 and previous is not None
            else 0.0
            for name in JOINT_NAMES
        ]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [registered[name] for name in JOINT_NAMES]
        message.velocity = velocities
        message.effort = [0.0] * len(JOINT_NAMES)
        self._joint_publisher.publish(message)

    def _on_ros_joint_command(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            self._log_throttled(
                "error",
                "bad_ros_command",
                "Ignored /so101/joint_commands with mismatched names and positions.",
            )
            return
        with self._lock:
            if self._latest_registered is None:
                return
            desired = dict(self._desired_registered or self._latest_registered)
            recognized = 0
            for name, position in zip(message.name, message.position, strict=True):
                if name in JOINT_TO_FIELD:
                    desired[name] = float(position)
                    recognized += 1
            if recognized:
                self._desired_registered = desired

    def _load_auth_key(self) -> bytes:
        try:
            key_hex = self._control_key_file.read_text(encoding="utf-8").strip()
        except OSError as exc:
            raise ProtocolError(
                f"MoveIt control key is unavailable at {self._control_key_file}"
            ) from exc
        if not re.fullmatch(r"[0-9a-f]{64}", key_hex):
            raise ProtocolError("MoveIt control key is not 256-bit lowercase hex")
        return bytes.fromhex(key_hex)

    def _send_command(
        self,
        key: str,
        body: Mapping[str, Any],
        ttl_ms: int,
    ) -> dict[str, Any]:
        with self._lock:
            capabilities = self._capabilities
        if capabilities is None:
            raise ProtocolError("capabilities are unavailable")
        command_id = f"moveit-{uuid.uuid4()}"
        pending: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=1)
        with self._lock:
            self._pending_acks[command_id] = pending
        try:
            command = authenticated_command(
                body,
                command_id,
                int(time.time() * 1000),
                ttl_ms,
                str(capabilities["command_auth"]["key_id"]),
                self._load_auth_key(),
            )
            self._session.put(key, json.dumps(command, separators=(",", ":")))
            try:
                ack = pending.get(timeout=max(2.0, ttl_ms / 1000.0 + 1.0))
            except queue.Empty as exc:
                raise TimeoutError(f"SO-101 command ACK timed out for {key}") from exc
        finally:
            with self._lock:
                self._pending_acks.pop(command_id, None)
        if ack.get("ok") is not True:
            error = ack.get("error") if isinstance(ack.get("error"), dict) else {}
            raise MotionCommandRejected(
                str(error.get("code", "command_rejected")),
                str(error.get("message", "SO-101 command was rejected")),
            )
        result = ack.get("result")
        if not isinstance(result, dict):
            raise ProtocolError("SO-101 command ACK did not contain a result")
        return result

    def _refresh_capabilities(self) -> dict[str, Any]:
        capabilities = self._query_json(KEYS["capabilities"])
        validate_capabilities(capabilities)
        with self._lock:
            self._capabilities = capabilities
        return capabilities

    def _acquire(self, capabilities: Mapping[str, Any]) -> None:
        with self._lock:
            joints = self._latest_joints
        if joints is None:
            raise ProtocolError("cannot acquire without joint telemetry")
        allow_current_pose = bool(joints.get("soft_limit_violations"))
        result = self._send_command(
            KEYS["lease"],
            {
                "schema": "so101.motion.lease.command.v1",
                "op": "acquire",
                "client_id": self._client_id,
                "bound_arm": HARDWARE_ARM,
                "allow_current_pose": allow_current_pose,
            },
            min(2_000, int(capabilities["lease_ttl_ms"])),
        )
        lease = result.get("lease")
        state = result.get("state")
        if (
            not isinstance(lease, dict)
            or not isinstance(state, dict)
            or result.get("torque_enabled") is not False
        ):
            raise ProtocolError("lease ACK did not contain a torque-off lease state")
        self._validate_joints(
            {
                **state,
                "schema": "so101.joints.state.v1",
                "robot_id": ROBOT_ID,
            }
        )
        with self._lock:
            self._lease = lease
            self._sequence = int(lease["last_sequence"])
            self._last_applied = dict(state["sim"])
            self._desired_registered = registered_from_canonical(state["sim"])
            self._following_error_count = 0
        self.get_logger().warning(
            "MoveIt Pro acquired the exclusive SO-101 right-arm lease; "
            "the first jog command will seed torque at the measured pose."
        )

    def _best_effort_stop(self, reason: str) -> None:
        try:
            self._send_command(
                KEYS["stop"],
                {
                    "schema": "so101.motion.stop.command.v1",
                    "reason": reason,
                },
                2_000,
            )
        except Exception as exc:
            self._log_throttled(
                "error",
                "stop_failed",
                f"SO-101 stop was not acknowledged: {exc}",
                1.0,
            )
        finally:
            with self._lock:
                self._lease = None
                self._last_applied = None
                self._sequence = -1

    def _motion_preconditions(
        self,
        capabilities: Mapping[str, Any],
    ) -> str | None:
        with self._lock:
            joints = self._latest_joints
            safety = self._latest_safety
            lease = self._lease
        if joints is None or safety is None:
            return "live joint or safety telemetry is unavailable"
        if not telemetry_is_fresh(
            joints,
            now_seconds=time.monotonic(),
            max_age_seconds=self._telemetry_max_age,
        ):
            return "joint telemetry is stale"
        if capabilities.get("writes_enabled") is not True:
            return "the Pi write window is closed"
        if safety.get("writes_enabled") is not True:
            return "safety telemetry reports the Pi write window closed"
        if safety.get("fault"):
            return f"the Pi supervisor is faulted: {safety['fault']}"
        if safety.get("estop_latched"):
            return f"the Pi emergency stop is latched: {safety.get('estop_reason')}"
        public_lease = safety.get("lease")
        if (
            lease is None
            and isinstance(public_lease, dict)
            and public_lease.get("client_id") != self._client_id
        ):
            return f"another client owns motion: {public_lease.get('client_id')}"
        return None

    def _jog_once(self, capabilities: Mapping[str, Any]) -> None:
        with self._lock:
            lease = dict(self._lease) if self._lease is not None else None
            previous = (
                dict(self._last_applied) if self._last_applied is not None else None
            )
            joints = dict(self._latest_joints) if self._latest_joints else None
            desired_registered = (
                dict(self._desired_registered)
                if self._desired_registered is not None
                else None
            )
            sequence = self._sequence + 1
        if (
            lease is None
            or previous is None
            or joints is None
            or desired_registered is None
        ):
            raise ProtocolError("jog state is incomplete")
        desired = canonical_from_registered(desired_registered)
        target = slew_target_from_feedback(
            desired,
            previous,
            joints["sim"],
            capabilities,
        )
        result = self._send_command(
            KEYS["jog"],
            {
                "schema": "so101.motion.jog.command.v1",
                "robot_id": ROBOT_ID,
                "bound_arm": HARDWARE_ARM,
                "mapping_id": MAPPING_ID,
                "calibration_sha256": CALIBRATION_SHA256,
                "lease_id": lease["id"],
                "client_id": self._client_id,
                "sequence": sequence,
                "targets": target,
            },
            int(capabilities["command_ttl_max_ms"]),
        )
        if result.get("sequence") != sequence or not isinstance(
            result.get("sim"),
            dict,
        ):
            raise ProtocolError("jog ACK did not confirm sequence and applied pose")
        with self._lock:
            self._sequence = sequence
            self._last_applied = dict(result["sim"])
            self._following_error_count = 0

    def _on_command_tick(self) -> None:
        enabled = hardware_marker_is_valid(
            self._commands_enabled_file,
            now_epoch_seconds=time.time(),
        )
        with self._lock:
            was_enabled = self._commands_were_enabled
            lease_exists = self._lease is not None
            latest_registered = (
                dict(self._latest_registered)
                if self._latest_registered is not None
                else None
            )

        if not enabled:
            if was_enabled or lease_exists:
                self._best_effort_stop("moveit_mode_disabled")
            with self._lock:
                self._commands_were_enabled = False
                self._fatal_latched = False
                self._following_error_count = 0
            return

        if not was_enabled:
            with self._lock:
                self._commands_were_enabled = True
                self._fatal_latched = False
                self._desired_registered = latest_registered
            self.get_logger().warning(
                "MoveIt hardware command marker detected; waiting for the Pi "
                "write window and then holding the measured pose."
            )

        with self._lock:
            if self._fatal_latched:
                return

        try:
            capabilities = self._refresh_capabilities()
            blocked = self._motion_preconditions(capabilities)
            if blocked is not None:
                self._log_throttled(
                    "warning",
                    "motion_blocked",
                    f"MoveIt hardware commands remain blocked: {blocked}.",
                )
                return
            with self._lock:
                lease_exists = self._lease is not None
            if not lease_exists:
                self._acquire(capabilities)
            self._jog_once(capabilities)
        except MotionCommandRejected as exc:
            if exc.code in RECOVERABLE_JOG_ERRORS:
                with self._lock:
                    self._following_error_count += 1
                    if (
                        exc.code == "following_error"
                        and self._following_error_count >= 2
                        and self._latest_registered is not None
                    ):
                        self._desired_registered = dict(self._latest_registered)
                self._log_throttled(
                    "warning",
                    f"recoverable_{exc.code}",
                    f"SO-101 safely deferred a MoveIt command: {exc}",
                    1.0,
                )
                return
            self.get_logger().error(f"Fatal SO-101 command rejection: {exc}")
            self._best_effort_stop("moveit_command_rejected")
            with self._lock:
                self._fatal_latched = True
        except Exception as exc:
            self.get_logger().error(f"MoveIt hardware adapter fault: {exc}")
            self._best_effort_stop("moveit_adapter_fault")
            with self._lock:
                self._fatal_latched = True

    def destroy_node(self) -> bool:
        if hasattr(self, "_lease") and self._lease is not None:
            self._best_effort_stop("moveit_adapter_shutdown")
        for subscriber in getattr(self, "_zenoh_subscribers", []):
            try:
                subscriber.undeclare()
            except Exception:
                pass
        if hasattr(self, "_session"):
            try:
                self._session.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: So101ZenohBridge | None = None
    try:
        node = So101ZenohBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
