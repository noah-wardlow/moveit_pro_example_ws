"""Validation helpers for the policy-server JSON protocol.

This module deliberately has no ROS imports so the inference contract can be
tested in a plain Python environment and reused by server-side tooling.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Mapping, Sequence


class ProtocolError(ValueError):
    """The policy server or action-chunk carryover violated the contract."""


@dataclass(frozen=True)
class ValidatedChunk:
    """A policy response normalized into immutable, finite matrices."""

    actions: tuple[tuple[float, ...], ...]
    native_control_period: float
    raw_actions: tuple[tuple[float, ...], ...] | None


def _finite_float(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError(f"{label} is not numeric") from exc
    if not math.isfinite(result):
        raise ProtocolError(f"{label} is not finite")
    return result


def _matrix(
    value: Any,
    label: str,
    *,
    expected_width: int | None = None,
) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ProtocolError(f"{label} must be a matrix")
    if len(value) == 0:
        raise ProtocolError(f"{label} must not be empty")

    rows: list[tuple[float, ...]] = []
    width: int | None = None
    for row_index, row in enumerate(value):
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes, bytearray)):
            raise ProtocolError(f"{label}[{row_index}] must be an array")
        if len(row) == 0:
            raise ProtocolError(f"{label}[{row_index}] must not be empty")
        if width is None:
            width = len(row)
        elif len(row) != width:
            raise ProtocolError(f"{label} rows have inconsistent widths")
        rows.append(
            tuple(
                _finite_float(item, f"{label}[{row_index}][{column_index}]")
                for column_index, item in enumerate(row)
            )
        )

    if expected_width is not None and width != expected_width:
        raise ProtocolError(
            f"{label} width {width} does not match expected joint count {expected_width}"
        )
    return tuple(rows)


def carryover_matrix(
    data: Iterable[Any],
    dimension_sizes: Sequence[int],
    data_offset: int = 0,
) -> tuple[tuple[float, ...], ...] | None:
    """Decode a ROS Float64MultiArray carryover into step-major rows."""

    flat = list(data)
    if not flat:
        return None
    if len(dimension_sizes) != 2:
        raise ProtocolError("previous_action_chunk must have exactly two dimensions")

    steps, width = (int(dimension_sizes[0]), int(dimension_sizes[1]))
    offset = int(data_offset)
    if steps <= 0 or width <= 0:
        raise ProtocolError("previous_action_chunk dimensions must be positive")
    if offset < 0:
        raise ProtocolError("previous_action_chunk data_offset must not be negative")
    if len(flat) - offset != steps * width:
        raise ProtocolError(
            "previous_action_chunk data length does not match its dimensions"
        )

    values = [
        _finite_float(value, f"previous_action_chunk.data[{index}]")
        for index, value in enumerate(flat[offset:], start=offset)
    ]
    return tuple(
        tuple(values[row * width : (row + 1) * width]) for row in range(steps)
    )


def validate_response(
    payload: Mapping[str, Any],
    expected_joint_names: Sequence[str],
) -> ValidatedChunk:
    """Validate one successful policy-server response."""

    if not isinstance(payload, Mapping):
        raise ProtocolError("policy response must be a JSON object")
    expected_names = tuple(str(name) for name in expected_joint_names)
    if not expected_names or any(not name for name in expected_names):
        raise ProtocolError("expected joint names must be non-empty")
    if len(set(expected_names)) != len(expected_names):
        raise ProtocolError("expected joint names contain duplicates")

    response_names = payload.get("joint_names")
    if response_names is not None:
        if not isinstance(response_names, Sequence) or isinstance(
            response_names, (str, bytes, bytearray)
        ):
            raise ProtocolError("joint_names must be an array")
        if tuple(str(name) for name in response_names) != expected_names:
            raise ProtocolError(
                "policy response joint_names do not exactly match the observation"
            )

    actions = _matrix(
        payload.get("action_chunk"),
        "action_chunk",
        expected_width=len(expected_names),
    )
    dt = _finite_float(payload.get("dt"), "dt")
    if dt <= 0.0:
        raise ProtocolError("dt must be greater than zero")

    raw_value = payload.get("action_chunk_raw")
    raw_actions = (
        None
        if raw_value is None or raw_value == []
        else _matrix(raw_value, "action_chunk_raw")
    )
    if raw_actions is not None and len(raw_actions) != len(actions):
        raise ProtocolError(
            "action_chunk_raw must have the same number of steps as action_chunk"
        )

    return ValidatedChunk(
        actions=actions,
        native_control_period=dt,
        raw_actions=raw_actions,
    )
