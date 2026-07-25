# Hiros and rmw_zenoh architecture for SO-101

## Decision

MoveIt Pro should remain a standard ROS 2 application. The target transport is
`rmw_zenoh_cpp`, connected to the robot's existing Zenoh 1.9 router. A Hiros
endpoint on the Pi should translate the existing safety-supervised robot
protocol into typed ROS 2 entities.

Hiros does not automatically turn native Zenoh JSON keys into ROS topics. ROS
interoperability requires both ends to use the same ROS type name, RIHS01 type
hash, CDR representation, QoS, domain ID, and Zenoh router.

The current `so101_zenoh_ros` package remains the transitional implementation.
It is useful until the typed endpoint has passed the same safety and controller
tests, but it is not the final transport boundary.

## Runtime boundary

```text
MoveIt Pro container                         Raspberry Pi

MoveIt / Objectives / Trainer
           |
joint_trajectory_controller
           |
topic_based_ros2_control
           |
standard ROS 2 topics
           |
rmw_zenoh_cpp  <---- Zenoh 1.9 router ----> Hiros typed endpoint
                                               |
                                      existing native protocol
                                               |
                                  motion supervisor / serial owner
```

The first Pi endpoint should be a sidecar over the existing native protocol,
not a second serial-bus owner. That preserves the dashboard, exclusive lease,
HMAC command authentication, bounded write window, watchdog, step/rate limits,
following-error checks, calibration pin, and torque-off cleanup. A later
single-process implementation may absorb both APIs, but only after it has
equivalent safety tests.

## ROS contract

| Direction | ROS entity | Type | Notes |
| --- | --- | --- | --- |
| Pi to MoveIt | `/so101/joint_states` | `sensor_msgs/msg/JointState` | Registered MoveIt joint names and positions; sensor-data QoS |
| MoveIt to Pi | `/so101/joint_commands` | `sensor_msgs/msg/JointState` | Output of `topic_based_ros2_control`; never bypasses the supervisor |
| MoveIt to Pi | `/so101/hardware_control_ready` | `std_srvs/srv/Trigger` | Fail-closed preflight for both command gates and live safety state |
| Pi/camera bridge to MoveIt | `/so101/cameras/overhead/image_raw` | `sensor_msgs/msg/Image` | Stable Trainer/Objective topic |
| Pi/camera bridge to MoveIt | `/so101/cameras/wrist/image_raw` | `sensor_msgs/msg/Image` | Stable Trainer/Objective topic |
| MoveIt internal | `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Remains the controller action; it need not cross the Pi boundary initially |

Keeping `joint_trajectory_controller` and `topic_based_ros2_control` in the
MoveIt container is the lowest-risk migration. Hiros already includes the
message families needed by the state/command boundary. Moving the complete
`FollowJointTrajectory` action to the Pi can wait until `control_msgs` code
generation and action cancellation have dedicated integration tests.

The camera bridge may continue to translate RTSP into ROS images inside the
MoveIt container. That is already a standard ROS interface and avoids sending
uncompressed camera frames through the robot control router. A later
`CompressedImage` transport is an independent bandwidth decision.

## Compatibility proof

On 2026-07-25, an isolated Jazzy test on arm64 validated both directions:

1. a Hiros Python publisher sent a typed `std_msgs/msg/String` to a ROS 2
   subscriber using `rmw_zenoh_cpp`;
2. an `rmw_zenoh_cpp` publisher sent the same type to a Hiros subscriber;
3. the publish path also passed through the Pi's existing Zenoh 1.9 router;
4. a CDR-serialized `sensor_msgs/msg/JointState` published through Hiros was
   decoded correctly by a ROS 2 subscriber through that router.

All tests used unique topics and isolated ROS domain IDs. They did not publish
to a controller or issue a motor command.

The current `v0.1.0-rc13` Python release still has packaging defects:

- the arm64 Jazzy wheel filename has an extra platform suffix that pip rejects;
- the companion message wheel lacks its generated `types` package;
- hand-written nested Python message classes did not reproduce the generated
  `JointState` serializer, although raw typed CDR and simple generated-shape
  messages interoperated.

Therefore the first supported Pi endpoint should use the pinned Rust crates,
or wait for corrected Hiros Python wheels. Do not add the current wheel URLs to
the Pixi lock.

## Migration gates

### 1. Exercise MoveIt Pro over rmw_zenoh

- Install the Jazzy `rmw_zenoh_cpp` package in the MoveIt image.
- Run simulation with every MoveIt process using the same
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and isolated `ROS_DOMAIN_ID`.
- Connect through an explicit router endpoint with multicast scouting off.
- Run planning, controller switching, waypoint execution, Trainer recording,
  and both camera views.
- Keep Cyclone DDS as a launch-time rollback until this suite passes.

### 2. Add a read-only Hiros Pi endpoint

- Publish typed `/so101/joint_states` from the existing native state stream.
- Expose typed readiness without accepting commands.
- Validate name order, calibration hash, timestamps, stale-state behavior,
  restart behavior, router loss, and MCAP recording.
- Run this endpoint beside the native bridge because it does not own the serial
  bus.

### 3. Add guarded commands

- Subscribe to typed `/so101/joint_commands`.
- Translate commands into the existing authenticated, leased native request.
- Require the same two bounded command gates used today.
- Prove that malformed types, stale timestamps, expired leases, competing
  owners, router loss, process exit, and watchdog expiry all fail closed.
- Execute the `Ready` waypoint only with explicit operator authority.

### 4. Retire the container JSON adapter

- Switch the hardware config to the typed Pi endpoint.
- Preserve the current adapter as a rollback for one release.
- Remove it only after repeated live sessions have matching telemetry,
  command, recording, and failure behavior.

## Pixi boundary

Pixi should continue to own LeRobot, dataset conversion, visualization, and
other fast-moving Python/ML dependencies. ROS Jazzy and `rmw_zenoh_cpp` remain
in the MoveIt Linux image. A future `hiroz` Pixi feature may own code generation
and no-motion interoperability checks once upstream publishes valid wheels for
both `linux-64` and `linux-aarch64`; the deployed Pi endpoint should remain a
pinned systemd artifact with an explicit rollback.
