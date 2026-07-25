# SO-101 MoveIt Pro workspace

This workspace adds the physical follower arm to MoveIt Pro without replacing
the Raspberry Pi's existing camera, Zenoh, recording, or safety services.

## Packages

- `so101_moveit_config`: five-axis follower plus Norma parallel gripper,
  MoveIt/SRDF configuration, and a single-arm MuJoCo scene derived from
  `/Users/noah/mujoco/so101-robot-ops/camera-dashboard/public/models/so101_overhead_bimanual`.
- `so101_moveit_hardware_config`: the same planning model using
  `topic_based_ros2_control`.
- `so101_zenoh_ros`: ROS joint-state/command adapter for the Pi's existing
  authenticated native Zenoh protocol.
- `so101_camera_bridge`: read-only RTSP-to-ROS adapter for the Pi's existing
  head and gripper streams.
- `so101_vla_adapter`: a strict `GetActionChunk` adapter between MoveIt Pro
  and an HTTP policy server. It validates names, shapes, finite values, control
  rate, images, and real-time-chunking carryover, and never publishes directly
  to a controller.

The physical robot remains owned by the existing Pi motion supervisor. ROS 2
and DDS stay in the MoveIt Pro container; no ROS packages are required on the
Pi. This is the smallest live installation and avoids a second serial-bus
owner.

## Build

Inside the MoveIt Pro container:

```bash
cd ~/user_ws
./scripts/bootstrap.bash
colcon build --symlink-install
```

Use `MOVEIT_CONFIG_PACKAGE=so101_moveit_config` for simulation or
`MOVEIT_CONFIG_PACKAGE=so101_moveit_hardware_config` for live telemetry.

### Apple Container

Build the MoveIt Pro Apple-arm64 image once from the MoveIt Pro repository:

```bash
BUILD_TYPE=release \
IMAGE_TAG=moveit-pro-arm64:so101 \
BUILDER_CPUS=10 \
BUILDER_MEM=16G \
./apple_container/build.sh
```

Set `MOVEIT_CONFIG_PACKAGE` in the repository `.env`, then prepare and launch
the selected configuration:

```bash
IMAGE_TAG=moveit-pro-arm64:so101 \
CONTAINER_NAME=moveit-pro-so101 \
MOVEIT_APPLE_SKIP_SUBMODULES=1 \
MOVEIT_APPLE_WORKSPACE_BOOTSTRAP=scripts/bootstrap.bash \
./apple_container/run.sh

CONTAINER_NAME=moveit-pro-so101 ./apple_container/launch.sh
```

The workspace bootstrap is required for the physical configuration because it
installs the pinned Zenoh Python binding and imports
`topic_based_ros2_control`. It is safe to use for simulation as well.

For a source frontend, run this from `moveit_pro/src/web/frontend`:

```bash
set -a
source ../../../.env
set +a
pnpm start:e2e
```

Open <http://127.0.0.1:5173>. Switching between simulation and live telemetry
is reversible: stop the existing container, change only
`MOVEIT_CONFIG_PACKAGE`, and repeat the two setup/launch commands.

## VLA tooling

Pixi owns only the fast-changing Python/ML layer; ROS remains in the MoveIt
Pro Linux container. The lock covers both `linux-64` and `linux-aarch64`, so
use these commands inside Docker or Apple Container rather than on the macOS
host:

```bash
pixi run contract-tests
pixi run hold-policy
pixi run -e lerobot lerobot-info
pixi run -e lerobot train -- --help
pixi run -e visualization rerun-viewer
```

The hold policy repeats the observed state and is the first integration test:
it exercises image encoding, the HTTP boundary, `GetActionChunk`, and
trajectory stitching without requesting a change in robot pose.

Simulation publishes stable image topics:

- `/so101/cameras/overhead/image_raw`
- `/so101/cameras/wrist/image_raw`

The physical config publishes those exact same topics from the existing Pi
streams. This lets Objectives, Trainer layouts, and inference requests stay
mode-independent.

Trainer should record `/joint_states` as state and
`/so101/joint_commands` as the distinct commanded-action stream.
See [`RECORDING_AND_LEROBOT.md`](RECORDING_AND_LEROBOT.md) for the
episode-recorder comparison and proposed offline LeRobot v3 conversion
contract.

## Hardware authority

The hardware config starts read-only. It publishes the live encoders into ROS,
but ignores ROS commands unless both gates are open:

1. `.runtime/hardware-commands-enabled` exists in this workspace.
2. The Pi's existing in-memory motion write window is open.

Open both for a bounded operator session:

```bash
./scripts/so101-moveit-mode.bash enable \
  --duration-seconds 180 \
  --confirm ENABLE-SO101-MOVEIT-PRO
```

Return immediately to the safe baseline:

```bash
./scripts/so101-moveit-mode.bash disable
```

The adapter uses a unique `moveit-pro-*` client ID and the Pi's existing
exclusive lease, step/rate/following-error limits, command TTL, watchdog,
calibration pin, and torque-off cleanup. The dashboard remains available while
MoveIt Pro is read-only. Do not open a dashboard motion session at the same time
as a MoveIt Pro hardware session.

## Coordinate contract

MoveIt uses the registered MJCF joint coordinates. The adapter applies the
same calibration-keyed mechanical registration used by the camera dashboard
before translating to the Pi's canonical `xlerobot-overhead-v2` coordinates.
The exact calibration and model source hashes are recorded in
`src/so101_moveit_config/config/so101_mapping.yaml`.
