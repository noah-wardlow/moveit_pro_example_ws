"""Serve MoveIt Pro GetActionChunk requests from an HTTP policy server."""

from __future__ import annotations

from typing import Any

import requests

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_pro_ml_msgs.srv import GetActionChunk

from .images import encode_jpeg_base64
from .protocol import ProtocolError, carryover_matrix, validate_response


DEFAULT_JOINT_NAMES = [
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
    "Jaw_R",
]


class GetActionChunkAdapter(Node):
    """Translate the stable MoveIt Pro service into a small JSON protocol."""

    def __init__(self) -> None:
        super().__init__("so101_get_action_chunk_adapter")
        self.declare_parameter("service_name", "/get_action_chunk")
        self.declare_parameter("inference_url", "http://127.0.0.1:8973/infer")
        self.declare_parameter("request_timeout_seconds", 30.0)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("verify_tls", True)
        self.declare_parameter("expected_joint_names", DEFAULT_JOINT_NAMES)

        self._inference_url = (
            self.get_parameter("inference_url").get_parameter_value().string_value
        )
        self._timeout = (
            self.get_parameter("request_timeout_seconds")
            .get_parameter_value()
            .double_value
        )
        self._jpeg_quality = (
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )
        self._verify_tls = (
            self.get_parameter("verify_tls").get_parameter_value().bool_value
        )
        self._expected_joint_names = list(
            self.get_parameter("expected_joint_names")
            .get_parameter_value()
            .string_array_value
        )
        if self._timeout <= 0.0:
            raise ValueError("request_timeout_seconds must be greater than zero")
        if not self._inference_url.startswith(("http://", "https://")):
            raise ValueError("inference_url must use http:// or https://")

        service_name = (
            self.get_parameter("service_name").get_parameter_value().string_value
        )
        self._session = requests.Session()
        self._service = self.create_service(
            GetActionChunk, service_name, self._on_request
        )
        self.get_logger().info(
            f"Serving {service_name} from policy endpoint {self._inference_url}"
        )

    @staticmethod
    def _failure(response: Any, message: str) -> Any:
        response.success = False
        response.message = message
        return response

    def _validate_observation(self, request: Any) -> None:
        names = list(request.robot_state.name)
        positions = list(request.robot_state.position)
        if not names or len(names) != len(positions):
            raise ProtocolError(
                "robot_state must have the same non-zero number of names and positions"
            )
        if len(set(names)) != len(names):
            raise ProtocolError("robot_state joint names contain duplicates")
        if self._expected_joint_names and names != self._expected_joint_names:
            raise ProtocolError(
                "robot_state joint names do not match expected_joint_names: "
                f"received {names}"
            )
        if len(request.images) != len(request.image_names):
            raise ProtocolError("images and image_names lengths do not match")
        if len(set(request.image_names)) != len(request.image_names):
            raise ProtocolError("image_names contains duplicates")

    def _build_payload(self, request: Any) -> dict[str, Any]:
        self._validate_observation(request)
        images: dict[str, str] = {}
        for name, image in zip(request.image_names, request.images, strict=True):
            images[name] = encode_jpeg_base64(
                height=int(image.height),
                width=int(image.width),
                step=int(image.step),
                encoding=image.encoding,
                data=bytes(image.data),
                quality=int(self._jpeg_quality),
            )

        payload: dict[str, Any] = {
            "state": [float(value) for value in request.robot_state.position],
            "state_names": list(request.robot_state.name),
            "prompt": request.prompt,
            "images": images,
            "new_episode": bool(request.new_episode),
        }

        previous = request.previous_action_chunk
        previous_rows = carryover_matrix(
            previous.data,
            [dimension.size for dimension in previous.layout.dim],
            previous.layout.data_offset,
        )
        if previous_rows is not None:
            payload["prev_chunk_left_over"] = [
                list(row) for row in previous_rows
            ]
            payload["inference_delay"] = int(request.frozen_prefix_steps)

        anchor = request.previous_anchor_state
        if anchor.name or anchor.position:
            if len(anchor.name) != len(anchor.position):
                raise ProtocolError(
                    "previous_anchor_state names and positions lengths do not match"
                )
            payload["previous_anchor_state"] = {
                "names": list(anchor.name),
                "positions": [float(value) for value in anchor.position],
            }
        if request.guidance_horizon > 0:
            payload["execution_horizon"] = int(request.guidance_horizon)
        return payload

    def _on_request(self, request: Any, response: Any) -> Any:
        try:
            payload = self._build_payload(request)
        except (ProtocolError, ValueError, RuntimeError) as exc:
            return self._failure(response, f"invalid observation: {exc}")

        try:
            http_response = self._session.post(
                self._inference_url,
                json=payload,
                timeout=self._timeout,
                verify=self._verify_tls,
            )
            http_response.raise_for_status()
            result = http_response.json()
        except (requests.RequestException, ValueError) as exc:
            return self._failure(response, f"policy request failed: {exc}")

        if not isinstance(result, dict):
            return self._failure(response, "policy response is not a JSON object")
        if result.get("error"):
            return self._failure(response, f"policy error: {result['error']}")
        if result.get("done"):
            response.success = False
            response.message = str(result.get("message", "policy completed"))
            return response

        try:
            validated = validate_response(result, request.robot_state.name)
        except ProtocolError as exc:
            return self._failure(response, f"invalid policy response: {exc}")

        trajectory = JointTrajectory()
        trajectory.joint_names = list(request.robot_state.name)
        for action in validated.actions:
            point = JointTrajectoryPoint()
            point.positions = list(action)
            trajectory.points.append(point)

        response.success = True
        response.message = ""
        response.chunk = trajectory
        response.native_control_period = validated.native_control_period

        if validated.raw_actions is not None:
            raw = Float64MultiArray()
            steps = len(validated.raw_actions)
            width = len(validated.raw_actions[0])
            raw.layout.data_offset = 0
            raw.layout.dim = [
                MultiArrayDimension(
                    label="steps", size=steps, stride=steps * width
                ),
                MultiArrayDimension(label="dims", size=width, stride=width),
            ]
            raw.data = [value for row in validated.raw_actions for value in row]
            response.policy_action_chunk = raw
        return response

    def destroy_node(self) -> None:
        self._session.close()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GetActionChunkAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
