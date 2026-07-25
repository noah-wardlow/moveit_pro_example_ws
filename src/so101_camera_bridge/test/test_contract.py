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
