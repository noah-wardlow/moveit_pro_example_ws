# SO-101 with MoveIt Pro

This workspace supports one attached SO-101 follower in MuJoCo and on physical
hardware. The hardware path follows the same boundary as the working
`legalaspro/so101-ros-physical-ai` stack:

- the Raspberry Pi owns the Feetech serial bus;
- `controller_manager`, the Feetech ros2_control hardware interface,
  `joint_trajectory_controller`, `gripper_controller`, and
  `joint_state_broadcaster` run on the Pi;
- MoveIt Pro uses the standard ROS actions, services, and topics over
  `rmw_zenoh_cpp`;
- the camera bridge reads the Pi's existing RTSP streams without opening the
  camera devices;
- there is no browser lease, command watchdog, native JSON motion bridge, or
  second controller manager in the normal hardware path.

See [ROS 2 and Zenoh architecture](ROS2_ZENOH_ARCHITECTURE.md) for the exact
runtime boundary.

## Packages

| Package | Responsibility |
| --- | --- |
| `so101_moveit_config` | URDF/Xacro, MuJoCo scene, SRDF, planning limits, waypoints, simulation, and VLA Objectives |
| `so101_moveit_hardware_config` | Physical config that consumes the controllers already running on the Pi |
| `so101_camera_bridge` | Read-only RTSP to ROS `Image`/`CameraInfo` bridge |
| `so101_vla_adapter` | Validated `GetActionChunk` adapter for an HTTP policy server |

Pixi owns only the fast-moving Python/ML tools. ROS Jazzy and MoveIt Pro remain
in the MoveIt Pro Linux environment.

## Robot prerequisite

The Pi must expose its ROS graph through an `rmw_zenohd` router on port 7447 and
run `so101-ros2-control.service`. The expected active controllers are:

```text
joint_state_broadcaster
joint_trajectory_controller
gripper_controller
```

The default hostname is the Pi's Tailscale MagicDNS name,
`so101-pi.tail337068.ts.net`, so the same checkout works from macOS and Linux
on the tailnet. Set `SO101_ROBOT_HOST` or the individual endpoint variables
when using another network.

## Linux

Install MoveIt Pro and Git LFS, then clone this fork and check out the SO-101
branch:

```bash
git clone --recurse-submodules \
  git@github.com:noah-wardlow/moveit_pro_example_ws.git
cd moveit_pro_example_ws
git checkout feat/so101-vla-workflows
git lfs pull
```

From the workspace root, launch the physical config:

```bash
./scripts/run-linux-hardware.bash
```

The wrapper exports a Zenoh client configuration and then delegates to the
normal command:

```text
moveit_pro run -c so101_moveit_hardware_config
```

If MagicDNS is unavailable inside Docker, use the Pi's tailnet address:

```bash
SO101_ROBOT_HOST=100.x.y.z ./scripts/run-linux-hardware.bash
```

Advanced endpoint overrides are also supported:

```bash
SO101_ZENOH_REMOTE_ENDPOINT=tcp/100.x.y.z:7447 \
SO101_HEAD_RTSP_URL=rtsp://100.x.y.z:8554/head \
SO101_GRIPPER_RTSP_URL=rtsp://100.x.y.z:8554/gripper \
./scripts/run-linux-hardware.bash
```

## macOS with Apple Container

Prepare the workspace with the MoveIt Pro Apple Container workflow, selecting
`so101_moveit_hardware_config`. Then launch:

```bash
CONTAINER_NAME=moveit-pro-so101 \
./scripts/launch-apple-rmw-zenoh.bash
```

The Apple launcher builds the four SO-101 packages, starts a local
`rmw_zenohd`, connects it to the Pi router, and starts MoveIt Pro. It does not
patch the installed MoveIt Pro image or stage a control key.

The same launcher can start an isolated simulation without a Pi connection:

```bash
MOVEIT_CONFIG_PACKAGE=so101_moveit_config \
ROS_DOMAIN_ID=1 \
./scripts/launch-apple-rmw-zenoh.bash
```

## Simulation

Simulation does not require the Pi or Zenoh wrapper:

```bash
moveit_pro run -c so101_moveit_config
```

The MuJoCo and physical configurations publish the same camera topics:

- `/so101/cameras/overhead/image_raw`
- `/so101/cameras/wrist/image_raw`

The planning group, joint names, waypoints, controller action names, and VLA
Objective are also shared.

## First hardware check

1. Open **Objectives** and run **Teleoperate**.
2. Confirm both cameras and live joint state are visible.
3. Select the waypoint panel and click **Ready**.
4. Confirm the Objective reports success and the Pi continues holding the
   endpoint through its local controller.

The attached arm has a known lower-authority elbow. Its shared planning limit is
intentionally lower than the other joints; do not raise gains or limits merely
to make the visualization move faster.

## VLA and dataset tooling

Install Pixi and run:

```bash
pixi run contract-tests
pixi run hold-policy
pixi run -e lerobot lerobot-info
pixi run -e lerobot train -- --help
pixi run -e visualization rerun-viewer
```

Start with `hold-policy`; it returns the observed state and requests no change
in pose. See [Recording and LeRobot](RECORDING_AND_LEROBOT.md) for the MCAP and
conversion boundary.

## Compatibility override

`so101_moveit_hardware_config/objectives/request_teleoperation.xml` shadows the
released core Objective so waypoint mode always initializes
`velocity_scale_factor` and uses the standard JTC action. It can be removed
after the corresponding generic MoveIt Pro fix is included in the base release.
