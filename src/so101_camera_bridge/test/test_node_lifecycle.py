"""Node construction and teardown. Needs rclpy, unlike the pure contract tests."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

import so101_camera_bridge  # noqa: E402
from so101_camera_bridge.bridge_node import RtspCameraBridge  # noqa: E402

# Whatever made the package importable here has to reach the child process too,
# which is not guaranteed when pytest resolved it through sys.path rather than
# the environment.
PACKAGE_ROOT = str(Path(so101_camera_bridge.__file__).resolve().parent.parent)

# Unroutable, so the decoder fails fast instead of reaching a real camera.
DEAD_URL = "rtsp://127.0.0.1:1/none"

# These have to arrive as context-global overrides. The node reads the URLs in
# __init__ and starts decoding immediately, so set_parameters() after
# construction is too late — the workers are already streaming from whatever the
# defaults point at, which is the real robot.
DEAD_URL_ARGS = [
    "-p",
    f"head_url:={DEAD_URL}",
    "-p",
    f"gripper_url:={DEAD_URL}",
]

# Logged at the end of the constructor, just before main() calls spin().
READY_LOG = "Publishing read-only SO-101 camera taps"

# The marker above is printed before spin() builds the global executor, so
# signalling the instant it appears lands in that gap instead of in spin — a
# separate, much narrower race that this test is not about.
SPIN_SETTLE_SECONDS = 1.0

# rclpy's executor builds its wait set from the context *after* checking
# context.ok(), so it can lose a race with rclpy's own signal handler
# (executors.py:757). Nothing this node does prevents that, and it is not one of
# the regressions under test, so it is tolerated rather than asserted away.
UPSTREAM_WAIT_SET_RACE = "failed to initialize wait set"


@pytest.fixture
def ros():
    """Init rclpy with unroutable camera URLs; always tear the context down."""
    rclpy.init(args=["test_node_lifecycle", "--ros-args", *DEAD_URL_ARGS])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _node() -> RtspCameraBridge:
    """Build the node, refusing to proceed if it is aimed at a real camera."""
    node = RtspCameraBridge()
    urls = [worker.spec.url for worker in node._workers]
    if urls != [DEAD_URL, DEAD_URL]:
        node.destroy_node()
        pytest.fail(f"parameter overrides never reached the decoders: {urls}")
    return node


def _spawn_bridge() -> subprocess.Popen[str]:
    """Run main() in its own process so real signals can be delivered to it."""
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        path for path in (PACKAGE_ROOT, env.get("PYTHONPATH")) if path
    )
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from so101_camera_bridge.bridge_node import main; main()",
            "--ros-args",
            *DEAD_URL_ARGS,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        line = process.stdout.readline()
        if not line:
            break
        if READY_LOG in line:
            time.sleep(SPIN_SETTLE_SECONDS)
            return process
    process.kill()
    pytest.fail(f"bridge never reached {READY_LOG!r}: {process.communicate()[0]}")


def test_regression_destroy_node_does_not_raise_key_error(ros) -> None:
    """Regression: `self._publishers` shadowed the list rclpy's Node owns.

    rclpy.node.Node keeps its publishers in `self._publishers` and destroy_node()
    drains that list by index (`self._publishers[0]`, node.py:1956). A dict keyed
    by camera name made every teardown raise `KeyError: 0` — after the node had
    already stopped serving, so it only ever showed up in the logs.
    """
    node = _node()
    node.destroy_node()  # must not raise


def test_regression_camera_publishers_stay_tracked_by_rclpy(ros) -> None:
    """Regression: the shadowing also orphaned the publishers rclpy should destroy.

    Asserting the publishers are still in rclpy's own list pins the defect rather
    than the name: rebinding the attribute left them unreachable from Node's
    bookkeeping, so destroy_node() could never have cleaned them up.
    """
    node = _node()
    try:
        self_owned = node._publishers
        assert isinstance(
            self_owned, list
        ), "rclpy Node must still own `_publishers` as its own list"
        assert set(node._camera_publishers) == {"head", "gripper"}
        created = [
            publisher
            for pair in node._camera_publishers.values()
            for publisher in pair
        ]
        assert all(publisher in self_owned for publisher in created), (
            "every camera publisher must stay in rclpy's list, or destroy_node() "
            "cannot destroy it"
        )
    finally:
        node.destroy_node()


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_regression_signal_during_spin_exits_zero(stop_signal) -> None:
    """Regression: an ordinary stop signal made main() exit nonzero with a traceback.

    rclpy handles both SIGINT and SIGTERM by shutting the context down, so spin
    raises ExternalShutdownException and the unguarded rclpy.shutdown() in the
    `finally` then raised RCLError on top of it. Ctrl-C and a systemd stop are
    both normal, so the process must exit 0.
    """
    process = _spawn_bridge()
    process.send_signal(stop_signal)
    output = process.communicate(timeout=30)[0]
    assert "KeyError" not in output, f"teardown walked the wrong list:\n{output}"
    assert (
        "rcl_shutdown already called" not in output
    ), f"shutdown ran twice on an already-stopped context:\n{output}"
    if process.returncode != 0:
        assert UPSTREAM_WAIT_SET_RACE in output, (
            f"{stop_signal.name} is a clean stop but exited "
            f"{process.returncode}:\n{output}"
        )
