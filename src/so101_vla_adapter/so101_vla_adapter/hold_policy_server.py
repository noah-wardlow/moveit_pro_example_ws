"""A deterministic no-motion policy endpoint for integration testing."""

from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from typing import Any


MAX_REQUEST_BYTES = 64 * 1024 * 1024


class HoldPolicyHandler(BaseHTTPRequestHandler):
    """Repeat the observed joint state for every action in the returned chunk."""

    server_version = "SO101HoldPolicy/0.1"

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path != "/health":
            self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        self._json_response(
            HTTPStatus.OK,
            {
                "ready": True,
                "policy": "hold-current-state",
                "motion": "none",
            },
        )

    def do_POST(self) -> None:
        if self.path != "/infer":
            self._json_response(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0 or length > MAX_REQUEST_BYTES:
            self._json_response(
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                {"error": "invalid request size"},
            )
            return
        try:
            payload = json.loads(self.rfile.read(length))
            state = [float(value) for value in payload["state"]]
            names = [str(value) for value in payload["state_names"]]
            if not state or len(state) != len(names):
                raise ValueError("state and state_names lengths do not match")
            if any(not math.isfinite(value) for value in state):
                raise ValueError("state contains a non-finite value")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self._json_response(
                HTTPStatus.BAD_REQUEST, {"error": f"invalid observation: {exc}"}
            )
            return

        server = self.server
        actions = [list(state) for _ in range(server.chunk_steps)]
        response: dict[str, Any] = {
            "action_chunk": actions,
            "joint_names": names,
            "dt": server.control_period,
        }
        if server.emit_rtc:
            response["action_chunk_raw"] = actions
        self._json_response(HTTPStatus.OK, response)

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"{self.address_string()} - {format_string % args}")


class HoldPolicyServer(ThreadingHTTPServer):
    """HTTP server carrying immutable policy configuration."""

    def __init__(
        self,
        address: tuple[str, int],
        *,
        chunk_steps: int,
        control_period: float,
        emit_rtc: bool,
    ) -> None:
        super().__init__(address, HoldPolicyHandler)
        self.chunk_steps = chunk_steps
        self.control_period = control_period
        self.emit_rtc = emit_rtc


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Serve a policy that repeats the observed state. This validates the "
            "VLA transport and controller path without requesting robot motion."
        )
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8973)
    parser.add_argument("--chunk-steps", type=int, default=32)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument(
        "--emit-rtc",
        action="store_true",
        help="Echo an RTC action_chunk_raw in addition to absolute actions.",
    )
    parsed = parser.parse_args(args)
    if not 1 <= parsed.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if parsed.chunk_steps <= 1:
        parser.error("--chunk-steps must be greater than one")
    if not math.isfinite(parsed.dt) or parsed.dt <= 0.0:
        parser.error("--dt must be finite and greater than zero")
    return parsed


def main(args: list[str] | None = None) -> None:
    parsed = parse_args(args)
    server = HoldPolicyServer(
        (parsed.bind, parsed.port),
        chunk_steps=parsed.chunk_steps,
        control_period=parsed.dt,
        emit_rtc=parsed.emit_rtc,
    )
    print(
        f"Hold policy ready at http://{parsed.bind}:{parsed.port}; "
        f"{parsed.chunk_steps} steps at dt={parsed.dt:g}. "
        "It only repeats the observed state."
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
