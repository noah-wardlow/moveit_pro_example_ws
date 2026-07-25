import math

import pytest

from so101_vla_adapter.protocol import (
    ProtocolError,
    carryover_matrix,
    validate_response,
)


JOINTS = ["a", "b"]


def test_valid_response_preserves_actions_and_raw_chunk():
    result = validate_response(
        {
            "joint_names": JOINTS,
            "action_chunk": [[1, 2], [3.5, 4]],
            "action_chunk_raw": [[0.1, 0.2], [0.3, 0.4]],
            "dt": 0.1,
        },
        JOINTS,
    )

    assert result.actions == ((1.0, 2.0), (3.5, 4.0))
    assert result.raw_actions == ((0.1, 0.2), (0.3, 0.4))
    assert result.native_control_period == 0.1


@pytest.mark.parametrize(
    "payload,match",
    [
        ({"action_chunk": [[1.0]], "dt": 0.1}, "width"),
        ({"action_chunk": [[1.0, math.nan]], "dt": 0.1}, "not finite"),
        ({"action_chunk": [[1.0, 2.0]], "dt": 0.0}, "greater than zero"),
        (
            {
                "joint_names": ["b", "a"],
                "action_chunk": [[1.0, 2.0]],
                "dt": 0.1,
            },
            "joint_names",
        ),
        (
            {
                "action_chunk": [[1.0, 2.0], [2.0, 3.0]],
                "action_chunk_raw": [[0.0, 0.0]],
                "dt": 0.1,
            },
            "same number of steps",
        ),
    ],
)
def test_invalid_responses_fail_closed(payload, match):
    with pytest.raises(ProtocolError, match=match):
        validate_response(payload, JOINTS)


def test_carryover_matrix_honors_data_offset():
    assert carryover_matrix(
        [99.0, 1.0, 2.0, 3.0, 4.0],
        [2, 2],
        data_offset=1,
    ) == ((1.0, 2.0), (3.0, 4.0))


def test_carryover_matrix_rejects_bad_shape():
    with pytest.raises(ProtocolError, match="data length"):
        carryover_matrix([1.0, 2.0, 3.0], [2, 2])
