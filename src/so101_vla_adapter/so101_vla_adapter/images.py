"""sensor_msgs/Image-compatible JPEG encoding without a cv_bridge dependency."""

from __future__ import annotations

import base64


def encode_jpeg_base64(
    *,
    height: int,
    width: int,
    step: int,
    encoding: str,
    data: bytes,
    quality: int = 90,
) -> str:
    """Encode a possibly row-padded ROS image buffer as base64 JPEG."""

    import cv2
    import numpy as np

    normalized_encoding = str(encoding).lower()
    channel_count = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
    }.get(normalized_encoding)
    if channel_count is None:
        raise ValueError(f"unsupported image encoding '{encoding}'")
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    row_bytes = width * channel_count
    if step < row_bytes:
        raise ValueError("image step is smaller than the encoded row width")
    if len(data) < height * step:
        raise ValueError("image data is shorter than height * step")
    if quality < 1 or quality > 100:
        raise ValueError("JPEG quality must be in [1, 100]")

    rows = np.frombuffer(data[: height * step], dtype=np.uint8).reshape(height, step)
    image = rows[:, :row_bytes].reshape(height, width, channel_count)[:, :, :3]
    if normalized_encoding.startswith("rgb"):
        image = image[:, :, ::-1]

    ok, encoded = cv2.imencode(
        ".jpg",
        np.ascontiguousarray(image),
        [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)],
    )
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return base64.b64encode(encoded.tobytes()).decode("ascii")
