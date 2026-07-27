"""Node construction and teardown. Needs rclpy, unlike the pure contract tests."""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")

from so101_camera_bridge.bridge_node import RtspCameraBridge  # noqa: E402

# Unroutable, so the decoder fails fast instead of reaching a real camera.
DEAD_URL = "rtsp://127.0.0.1:1/none"


@pytest.fixture
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node() -> RtspCameraBridge:
    from rclpy.parameter import Parameter

    node = RtspCameraBridge()
    node.set_parameters(
        [
            Parameter("head_url", Parameter.Type.STRING, DEAD_URL),
            Parameter("gripper_url", Parameter.Type.STRING, DEAD_URL),
        ]
    )
    return node


def test_destroy_node_does_not_raise(ros) -> None:
    """rclpy's Node keeps its own `_publishers` list and walks it on teardown.

    Shadowing that name with a dict keyed by camera makes destroy_node() index a
    dict by integer, so every shutdown raised KeyError after the node had already
    stopped serving — visible only in the logs, and easy to mistake for noise.
    """
    node = _node()
    node.destroy_node()  # must not raise


def test_camera_publishers_do_not_shadow_the_node_attribute(ros) -> None:
    """Pins the cause, so a rename back is caught at the point of the mistake."""
    node = _node()
    try:
        assert isinstance(node._publishers, list)
        assert set(node._camera_publishers) == {"head", "gripper"}
    finally:
        node.destroy_node()
