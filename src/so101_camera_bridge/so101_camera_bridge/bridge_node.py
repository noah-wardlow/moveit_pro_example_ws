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
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        """Begin decoding. Idempotent, so the demand check can call it freely."""
        if self.running:
            return
        self._stop.clear()
        with self._lock:
            # Drop a frame retained from a previous run; a subscriber that
            # arrives now wants live video, not whatever was on screen when the
            # last one left.
            self._latest = None
        self._thread = threading.Thread(
            target=self._run,
            name=f"rtsp-{self.spec.name}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop decoding and release the upstream connection. Idempotent."""
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=max(2.0, self._read_timeout_ms / 1000.0 + 1.0))
        self._thread = None

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
        # CameraInfo used to be derived from a decoded frame, which meant the
        # decoder had to run just to describe the camera. Declaring the geometry
        # lets the info topic stand on its own; the decoder validates it against
        # the first real frame it sees.
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
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

        # Not `self._publishers`: rclpy's Node keeps its own list of publishers
        # under that name, and shadowing it with a dict makes destroy_node()
        # index a dict by integer and raise KeyError while tearing the node down.
        self._camera_publishers = {
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
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        if min(self._width, self._height) <= 0:
            raise ValueError("image_width and image_height must be positive")
        # Warned about once per stream rather than per frame.
        self._geometry_warned: set[str] = set()

        # Workers are NOT started here. Decoding begins only when something
        # subscribes to the image topic — see _publish_latest. The publishers
        # exist from construction, so the topics are discoverable and the camera
        # panes list them whether or not anything is decoding.
        self._timer = self.create_timer(1.0 / publish_fps, self._publish_latest)
        self.get_logger().info(
            "Publishing read-only SO-101 camera taps on stable ROS topics; "
            "decoding starts on the first image subscriber"
        )

    def _camera_info(self, frame_id: str, stamp, width: int, height: int) -> CameraInfo:
        """Describe the camera. Needs no decoded frame."""
        k, p = pinhole_projection(width, height, self._vertical_fov)
        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        camera_info.height = height
        camera_info.width = width
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0] * 5
        camera_info.k = k
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = p
        return camera_info

    def _publish_latest(self) -> None:
        """Publish CameraInfo always; decode and publish images only on demand.

        The video path no longer runs through this node — an already-encoded
        camera is relayed straight to the browser — so decoding every frame
        unconditionally burned CPU and memory producing images nothing read.
        Subscriber count is the honest signal for whether anyone still wants
        them: a recording session or a perception Behavior subscribing starts
        the decoder, and it stops again when they let go.
        """
        stamp = self.get_clock().now().to_msg()
        for worker in self._workers:
            name = worker.spec.name
            image_publisher, info_publisher = self._camera_publishers[name]

            # Cheap, and it keeps calibration available to anything that needs
            # to describe the camera without wanting its pixels.
            info_publisher.publish(
                self._camera_info(
                    worker.spec.frame_id, stamp, self._width, self._height
                )
            )

            if image_publisher.get_subscription_count() == 0:
                if worker.running:
                    self.get_logger().info(
                        f"{name}: last image subscriber left; stopping decoder"
                    )
                    worker.stop()
                    self._last_sequences[name] = 0
                continue

            if not worker.running:
                self.get_logger().info(
                    f"{name}: image subscriber appeared; starting decoder"
                )
                worker.start()

            latest = worker.newest_after(self._last_sequences[name])
            if latest is None:
                continue
            sequence, frame = latest
            height, width = frame.shape[:2]

            if (width, height) != (self._width, self._height):
                # The declared geometry drives CameraInfo, so a mismatch means
                # consumers are being handed intrinsics for a different image.
                if name not in self._geometry_warned:
                    self._geometry_warned.add(name)
                    self.get_logger().warning(
                        f"{name}: stream is {width}x{height} but image_width/"
                        f"image_height say {self._width}x{self._height}; "
                        "CameraInfo will not match the image"
                    )

            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = worker.spec.frame_id
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = 0
            image.step = width * 3
            image.data = frame.tobytes()
            image_publisher.publish(image)
            self._last_sequences[name] = sequence

    def destroy_node(self) -> None:
        for worker in self._workers:
            worker.stop()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RtspCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Guarded: a signal that interrupts spin can leave the context already
        # shut down, and calling it again raises over the top of whatever
        # actually stopped the node.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
