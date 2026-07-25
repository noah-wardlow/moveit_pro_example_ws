import threading

import requests

from so101_vla_adapter.hold_policy_server import HoldPolicyServer


def test_hold_server_health_and_inference():
    server = HoldPolicyServer(
        ("127.0.0.1", 0),
        chunk_steps=3,
        control_period=0.1,
        emit_rtc=True,
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_port}"
    try:
        health = requests.get(f"{base_url}/health", timeout=2.0)
        assert health.status_code == 200
        assert health.json()["motion"] == "none"

        inference = requests.post(
            f"{base_url}/infer",
            json={
                "state": [1.0, 2.0],
                "state_names": ["joint_a", "joint_b"],
                "images": {},
                "prompt": "",
                "new_episode": True,
            },
            timeout=2.0,
        )
        assert inference.status_code == 200
        payload = inference.json()
        assert payload["joint_names"] == ["joint_a", "joint_b"]
        assert payload["action_chunk"] == [[1.0, 2.0]] * 3
        assert payload["action_chunk_raw"] == [[1.0, 2.0]] * 3
        assert payload["dt"] == 0.1
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
