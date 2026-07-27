import math

import pytest

from so101_camera_bridge.contract import (
    StreamSpec,
    pinhole_projection,
    validate_stream_spec,
)


def test_valid_stream_contract() -> None:
    validate_stream_spec(
        StreamSpec(
            "head",
            "rtsp://robot:8554/head",
            "/camera/image_raw",
            "/camera/camera_info",
            "camera_optical_frame",
        )
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("url", "https://robot/head"),
        ("image_topic", "camera/image_raw"),
        ("camera_info_topic", "/camera/info/"),
        ("frame_id", "/camera_optical_frame"),
    ],
)
def test_invalid_stream_contract(field: str, value: str) -> None:
    values = {
        "name": "head",
        "url": "rtsp://robot:8554/head",
        "image_topic": "/camera/image_raw",
        "camera_info_topic": "/camera/camera_info",
        "frame_id": "camera_optical_frame",
    }
    values[field] = value
    with pytest.raises(ValueError):
        validate_stream_spec(StreamSpec(**values))


def test_pinhole_projection_is_centered_and_finite() -> None:
    k, p = pinhole_projection(640, 480, 74.5)
    assert len(k) == 9
    assert len(p) == 12
    assert k[2] == 319.5
    assert k[5] == 239.5
    assert k[0] == k[4]
    assert all(math.isfinite(value) for value in k + p)


@pytest.mark.parametrize(
    "width,height,fov",
    [(0, 480, 74.5), (640, 0, 74.5), (640, 480, 0.0), (640, 480, 180.0)],
)
def test_invalid_projection(width: int, height: int, fov: float) -> None:
    with pytest.raises(ValueError):
        pinhole_projection(width, height, fov)


def test_stream_worker_start_and_stop_are_idempotent() -> None:
    """The demand check calls these on every publish tick, so repeated calls
    must not spawn a second decoder or fail on an already-stopped worker."""
    from so101_camera_bridge.bridge_node import StreamWorker

    worker = StreamWorker(
        StreamSpec(
            name="probe",
            url="rtsp://127.0.0.1:1/none",
            image_topic="/probe/image_raw",
            camera_info_topic="/probe/camera_info",
            frame_id="probe_frame",
        ),
        reconnect_delay=0.1,
        open_timeout_ms=100,
        read_timeout_ms=100,
        log=lambda _message: None,
    )
    assert not worker.running
    worker.stop()  # stopping a worker that never ran is a no-op
    worker.start()
    assert worker.running
    worker.start()  # must not spawn a second decoder
    assert worker.running
    worker.stop()
    assert not worker.running
