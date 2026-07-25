"""Publish the existing SO-101 MediaMTX RTSP streams as ROS images."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .contract import StreamSpec, pinhole_projection, validate_stream_spec


class StreamWorker:
    """Keep one RTSP decoder alive and retain only its newest frame."""

    def __init__(
        self,
        spec: StreamSpec,
        *,
        reconnect_delay: float,
        open_timeout_ms: int,
        read_timeout_ms: int,
        log: Callable[[str], None],
    ) -> None:
        self.spec = spec
        self._reconnect_delay = reconnect_delay
        self._open_timeout_ms = open_timeout_ms
        self._read_timeout_ms = read_timeout_ms
        self._log = log
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest: Any | None = None
        self._sequence = 0
        self._thread = threading.Thread(
            target=self._run,
            name=f"rtsp-{spec.name}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=max(2.0, self._read_timeout_ms / 1000.0 + 1.0))

    def newest_after(self, sequence: int) -> tuple[int, Any] | None:
        with self._lock:
            if self._latest is None or self._sequence <= sequence:
                return None
            return self._sequence, self._latest

    def _open(self) -> Any:
        capture = cv2.VideoCapture()
        if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self._open_timeout_ms)
        if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
            capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, self._read_timeout_ms)
        capture.open(self.spec.url, cv2.CAP_FFMPEG)
        return capture

    def _run(self) -> None:
        announced = False
        while not self._stop.is_set():
            capture = self._open()
            if not capture.isOpened():
                capture.release()
                if not announced:
                    self._log(f"{self.spec.name} RTSP stream is unavailable; retrying")
                    announced = True
                self._stop.wait(self._reconnect_delay)
                continue
            self._log(f"{self.spec.name} RTSP stream connected")
            announced = False
            while not self._stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    self._log(f"{self.spec.name} RTSP stream interrupted; reconnecting")
                    break
                if len(frame.shape) != 3 or frame.shape[2] != 3:
                    self._log(f"{self.spec.name} produced a non-BGR frame; reconnecting")
                    break
                with self._lock:
                    self._latest = frame
                    self._sequence += 1
            capture.release()
            self._stop.wait(self._reconnect_delay)


class RtspCameraBridge(Node):
    """Read two existing RTSP streams without owning camera hardware."""

    def __init__(self) -> None:
        super().__init__("so101_rtsp_camera_bridge")
        self.declare_parameter(
            "head_url", "rtsp://so101-pi.tail337068.ts.net:8554/head"
        )
        self.declare_parameter(
            "gripper_url",
            "rtsp://so101-pi.tail337068.ts.net:8554/gripper",
        )
        self.declare_parameter("publish_fps", 15.0)
        self.declare_parameter("vertical_fov_degrees", 74.5)
        self.declare_parameter("reconnect_delay_seconds", 1.0)
        self.declare_parameter("open_timeout_ms", 5000)
        self.declare_parameter("read_timeout_ms", 3000)

        specs = [
            StreamSpec(
                name="head",
                url=self.get_parameter("head_url").value,
                image_topic="/so101/cameras/overhead/image_raw",
                camera_info_topic="/so101/cameras/overhead/camera_info",
                frame_id="overhead_cam_optical_frame",
            ),
            StreamSpec(
                name="gripper",
                url=self.get_parameter("gripper_url").value,
                image_topic="/so101/cameras/wrist/image_raw",
                camera_info_topic="/so101/cameras/wrist/camera_info",
                frame_id="right_wrist_cam_optical_frame",
            ),
        ]
        for spec in specs:
            validate_stream_spec(spec)

        publish_fps = float(self.get_parameter("publish_fps").value)
        self._vertical_fov = float(
            self.get_parameter("vertical_fov_degrees").value
        )
        reconnect_delay = float(
            self.get_parameter("reconnect_delay_seconds").value
        )
        open_timeout_ms = int(self.get_parameter("open_timeout_ms").value)
        read_timeout_ms = int(self.get_parameter("read_timeout_ms").value)
        if publish_fps <= 0.0:
            raise ValueError("publish_fps must be greater than zero")
        if reconnect_delay <= 0.0:
            raise ValueError("reconnect_delay_seconds must be greater than zero")
        if min(open_timeout_ms, read_timeout_ms) <= 0:
            raise ValueError("RTSP timeouts must be greater than zero")
        pinhole_projection(640, 480, self._vertical_fov)

        self._publishers = {
            spec.name: (
                self.create_publisher(Image, spec.image_topic, 2),
                self.create_publisher(CameraInfo, spec.camera_info_topic, 2),
            )
            for spec in specs
        }
        self._last_sequences = {spec.name: 0 for spec in specs}
        self._workers = [
            StreamWorker(
                spec,
                reconnect_delay=reconnect_delay,
                open_timeout_ms=open_timeout_ms,
                read_timeout_ms=read_timeout_ms,
                log=self.get_logger().info,
            )
            for spec in specs
        ]
        for worker in self._workers:
            worker.start()
        self._timer = self.create_timer(1.0 / publish_fps, self._publish_latest)
        self.get_logger().info(
            "Publishing read-only SO-101 camera taps on stable ROS topics"
        )

    def _publish_latest(self) -> None:
        for worker in self._workers:
            latest = worker.newest_after(self._last_sequences[worker.spec.name])
            if latest is None:
                continue
            sequence, frame = latest
            height, width = frame.shape[:2]
            stamp = self.get_clock().now().to_msg()

            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = worker.spec.frame_id
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = 0
            image.step = width * 3
            image.data = frame.tobytes()

            k, p = pinhole_projection(width, height, self._vertical_fov)
            camera_info = CameraInfo()
            camera_info.header = image.header
            camera_info.height = height
            camera_info.width = width
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [0.0] * 5
            camera_info.k = k
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            camera_info.p = p

            image_publisher, info_publisher = self._publishers[worker.spec.name]
            image_publisher.publish(image)
            info_publisher.publish(camera_info)
            self._last_sequences[worker.spec.name] = sequence

    def destroy_node(self) -> None:
        for worker in self._workers:
            worker.stop()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RtspCameraBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
