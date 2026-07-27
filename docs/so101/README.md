# SO-101 with MoveIt Pro

This workspace supports one attached SO-101 follower in MuJoCo and on physical
hardware. The hardware path follows the same boundary as the working
`legalaspro/so101-ros-physical-ai` stack:

- the Raspberry Pi owns the Feetech serial bus;
- `controller_manager`, the Feetech ros2_control hardware interface,
  MoveIt Pro's Cartesian/joint jog controllers, trajectory/gripper
  controllers, and joint-state broadcaster run on the Pi;
- MoveIt Pro uses standard ROS 2 actions, services, and topics through a
  directionally filtered `zenoh-bridge-ros2dds` link;
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
| `so101_camera_bridge` | Read-only RTSP to ROS `Image`/`CameraInfo` bridge; MoveIt Pro's generic video service presents those topics to the UI over WebRTC |
| `so101_vla_adapter` | Validated `GetActionChunk` adapter for an HTTP policy server |

Pixi owns only the fast-moving Python/ML tools. ROS Jazzy and MoveIt Pro remain
in the MoveIt Pro Linux environment.

## Robot prerequisite

The Pi must run `so101-ros2-control.service` and
`so101-ros2dds-bridge.service`. The latter exposes the standard ROS 2 contract
at `tcp/<robot>:7448`; port 7447 remains separate legacy dashboard transport.
The expected controller states before Teleoperate starts are:

```text
joint_state_broadcaster       active
joint_trajectory_controller   active
gripper_controller            active
velocity_force_controller     inactive
joint_velocity_controller     inactive
```

The default hostname is the Pi's Tailscale MagicDNS name,
`so101-pi.tail337068.ts.net`, so the same checkout works from macOS and Linux
on the tailnet. Set `SO101_ROBOT_HOST` or `SO101_ROS2DDS_ENDPOINT` when using
another network.

## Ubuntu/Linux

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

The wrapper downloads the pinned standalone ROS2DDS bridge for `x86_64` or
`aarch64`, verifies its SHA-256, binds its DDS side to loopback, and delegates
to the normal command:

```text
moveit_pro run -c so101_moveit_hardware_config
```

If MagicDNS is unavailable, use the Pi's tailnet address:

```bash
SO101_ROBOT_HOST=100.x.y.z ./scripts/run-linux-hardware.bash
```

Endpoint and camera overrides are also supported:

```bash
SO101_ROS2DDS_ENDPOINT=tcp/100.x.y.z:7448 \
SO101_HEAD_RTSP_URL=rtsp://100.x.y.z:8554/head \
SO101_GRIPPER_RTSP_URL=rtsp://100.x.y.z:8554/gripper \
./scripts/run-linux-hardware.bash
```

The bridge is stopped when `moveit_pro run` exits. Its log is retained under
`${XDG_STATE_HOME:-$HOME/.local/state}/so101-moveit-pro/`.

## Runtime portability

The robot-facing contract is independent of the operator container runtime:
local CycloneDDS, one ROS2DDS client, and the stable Pi endpoint. Host-specific
wrappers must preserve that contract and gate hardware readiness on a typed
`/controller_manager/list_controllers` response. No workstation address or
control credential belongs in the workspace.

## Simulation

Simulation does not require the Pi or ROS2DDS wrapper:

```bash
moveit_pro run -c so101_moveit_config
```

The MuJoCo and physical configurations publish the same camera topics:

- `/so101/cameras/overhead/image_raw`
- `/so101/cameras/wrist/image_raw`

The planning group, joint names, waypoints, controller action names, and VLA
Objective are also shared.

With a MoveIt Pro build containing the generic WebRTC camera transport, both
topics appear as low-latency video panes. The SO-101 package remains a normal
ROS image publisher and contains no UI-specific streaming code.

## First hardware check

MoveIt Pro's UI **Stop** requests a cooperative software stop. It is not a
safety-rated emergency stop. Keep physical power isolation or the robot's
hardware E-stop accessible and maintain a clear workspace during live checks.

1. Open **Objectives** and run **Teleoperate**.
2. Confirm both cameras and live joint state are visible.
3. Select the waypoint panel and click **Ready**.
4. Confirm the Objective reports success and the Pi continues holding the
   endpoint through its local controller.
5. Select **Pose**, use `Base (Base_2)` as the control frame, and briefly jog
   each translation direction.

`Ready` is a collision-checked bent-arm pose with room for Cartesian jogging.
Position-only Pose Jog is intentional because this arm has five actuated arm
joints. A boundary warning from a genuinely long jog is valid; a warning on a
short base-frame jog indicates that the Pose Jog safety predictor and
controller are not evaluating the same control frame.

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

The current MoveIt Pro source also carries generic support for position-only
Pose Jog and command-frame-aware safety prediction. Neither change contains
SO-101 names or dimensions. See
[MoveIt Pro onboarding gaps](MOVEIT_PRO_ONBOARDING_GAPS.md) for the reusable
lessons from this integration.
