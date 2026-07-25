# SO-101 VLA adapter

This package is the only policy-specific component in the robot command path.
It exposes `moveit_pro_ml_msgs/srv/GetActionChunk`, converts observations to an
HTTP request, and validates every returned action chunk before MoveIt Pro sees
it. It never publishes a controller command itself.

## Policy protocol

`POST /infer` receives:

```json
{
  "state": [0.0, 1.575, 1.46, 0.0, 1.5707963268, 0.027],
  "state_names": [
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
    "Jaw_R"
  ],
  "prompt": "place the cube in the bowl",
  "images": {
    "overhead": "<base64 JPEG>",
    "wrist": "<base64 JPEG>"
  },
  "new_episode": true
}
```

RTC-capable calls additionally carry `prev_chunk_left_over`,
`inference_delay`, `previous_anchor_state`, and optionally
`execution_horizon`.

A successful response is:

```json
{
  "joint_names": [
    "Rotation_R",
    "Pitch_R",
    "Elbow_R",
    "Wrist_Pitch_R",
    "Wrist_Roll_R",
    "Jaw_R"
  ],
  "action_chunk": [[0.0, 1.575, 1.46, 0.0, 1.5707963268, 0.027]],
  "action_chunk_raw": [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
  "dt": 0.1
}
```

`action_chunk` must contain absolute joint positions. `action_chunk_raw` is
optional and is the policy's unmodified normalized action space for real-time
chunking. Rows must be finite and joint names must exactly match the request.

## No-motion integration server

The package includes a deterministic server that repeats the observed state:

```bash
ros2 run so101_vla_adapter hold_policy_server
ros2 launch so101_vla_adapter adapter.launch.py
```

Its `/health` endpoint reports readiness. It is intended to validate cameras,
serialization, `GetActionChunk`, controller switching, and trajectory
stitching without asking the robot to change pose.
