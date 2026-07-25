import base64

import cv2
import numpy as np
import pytest

from so101_vla_adapter.images import encode_jpeg_base64


def test_rgb_image_with_row_padding_encodes_to_expected_size():
    height = 2
    width = 3
    step = 12
    row = bytes([255, 0, 0] * width) + bytes([0, 0, 0])
    encoded = encode_jpeg_base64(
        height=height,
        width=width,
        step=step,
        encoding="rgb8",
        data=row * height,
        quality=95,
    )

    decoded = cv2.imdecode(
        np.frombuffer(base64.b64decode(encoded), dtype=np.uint8),
        cv2.IMREAD_COLOR,
    )
    assert decoded.shape == (height, width, 3)
    assert decoded[:, :, 2].mean() > 200


@pytest.mark.parametrize("encoding", ["mono8", "16UC1", ""])
def test_unsupported_encodings_are_rejected(encoding):
    with pytest.raises(ValueError, match="unsupported"):
        encode_jpeg_base64(
            height=1,
            width=1,
            step=3,
            encoding=encoding,
            data=b"\0\0\0",
        )
