# Recording and LeRobot boundary

The `legalaspro/so101-ros-physical-ai` repository makes two useful boundaries
explicit:

1. `episode_recorder` owns capture and episode lifecycle, but stays
   policy-agnostic.
2. `rosbag_to_lerobot` owns the opinionated dataset schema and runs in the
   fast-changing LeRobot/Pixi environment rather than the ROS runtime.

The same split fits MoveIt Pro, but the capture component does not need to be
duplicated. MoveIt Pro Trainer already provides recording state, demonstration
markers, and ROS recording hooks. The SO-101 integration should therefore use:

| Responsibility | MoveIt Pro workflow | Upstream analogue |
| --- | --- | --- |
| Start/stop/accept/reject an episode | Trainer UI and recording services | `episode_recorder` keyboard controls |
| Durable raw capture | ROS bag/MCAP produced by Trainer | `episode_recorder` MCAP |
| Observation state | `/joint_states` | configured joint-state topic |
| Commanded action | desired/reference fields from `/joint_trajectory_controller/controller_state` | configured command topic |
| Overhead image | `/so101/cameras/overhead/image_raw` | configured head camera |
| Wrist image | `/so101/cameras/wrist/image_raw` | configured gripper camera |
| Task/language metadata | Trainer episode metadata | converter arguments/metadata |
| LeRobot v3 conversion | offline Pixi `lerobot` environment | `rosbag_to_lerobot` |

Keeping state and action distinct matters. `/joint_states` describes what the
arm actually did; the JTC controller-state reference is the supervision target.
Treating observed state as action hides tracking error and makes
physical-policy training less faithful.

## Converter contract

An offline converter should consume accepted MCAP episodes and emit one
LeRobot episode per accepted recording. It should:

- resample joints and images onto an explicit policy clock;
- preserve source timestamps and record dropped/stale frames;
- preserve the ROS joint order and calibration identifier recorded by the Pi;
- encode both cameras under stable feature names;
- store the task prompt and robot/config identifiers;
- fail on missing action samples, non-finite values, or an unknown calibration
  hash;
- write locally first, then optionally push the validated dataset to the Hub.

That converter belongs in the Pixi `lerobot` feature, not in a ROS package.
ROS Jazzy and MoveIt Pro can then stay pinned while LeRobot, codecs, Hub
clients, and model dependencies evolve independently.

## Why MCAP remains the source of truth

Direct-to-LeRobot recording is tempting, but MCAP is the safer operational
boundary: it is replayable, inspectable with standard ROS tools, preserves
unmodeled topics for debugging, and allows the dataset schema to be rebuilt
after calibration or feature changes. LeRobot output should be treated as a
derived artifact.
