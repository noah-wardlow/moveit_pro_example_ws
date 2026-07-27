# ROS 2 and Zenoh architecture

## Decision

MoveIt Pro and the robot communicate as standard ROS 2 systems using
CycloneDDS on each host with a directionally filtered `zenoh-bridge-ros2dds`
link between them. The Pi runs the Feetech hardware interface and all
`ros2_control` controllers locally. This matches the controller boundary used
by `legalaspro/so101-ros-physical-ai` while avoiding a custom dashboard motion
protocol.

```text
MoveIt Pro workstation                         Raspberry Pi

Objectives / planning / Trainer
              |
FollowJointTrajectory + GripperCommand
controller services + VFC/JVC commands
              |
       local CycloneDDS                         local CycloneDDS
              |                                      |
       ros2dds client <---- Zenoh/TCP 7448 ----> ros2dds router
                                                     |
                                             controller_manager
                                             ├─ joint_state_broadcaster
                                             ├─ joint_trajectory_controller
                                             ├─ gripper_controller
                                             ├─ velocity_force_controller
                                             └─ joint_velocity_controller
                                                     |
                                             FeetechHardwareInterface
                                                     |
                                               /dev/ttyACM0
```

The Pi is the only serial-bus owner and the only controller manager for
physical hardware. The MoveIt Pro hardware config sets
`launch_control_node: false` and lists the Pi controllers as
`controllers_not_managed`. VFC and JVC still run on the Pi because their
high-rate update loop must be beside the hardware command interfaces.

## ROS contract

| Entity | Type | Owner |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Pi joint-state broadcaster |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Pi JTC |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Pi gripper controller |
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Pi controller manager |
| `/controller_manager/switch_controller` | `controller_manager_msgs/srv/SwitchController` | Pi controller manager |
| `/velocity_force_controller/command` | `moveit_pro_controllers_msgs/msg/VelocityForceCommand` | MoveIt Pose Jog |
| `/joint_velocity_controller/command` | `moveit_pro_controllers_msgs/msg/JointVelocityCommand` | MoveIt Joint Jog |
| `/so101/cameras/overhead/image_raw` | `sensor_msgs/msg/Image` | MoveIt camera bridge |
| `/so101/cameras/wrist/image_raw` | `sensor_msgs/msg/Image` | MoveIt camera bridge |
| `/get_action_chunk` | MoveIt Pro policy service | MoveIt VLA adapter |

RTSP camera traffic remains outside the ROS control router. The read-only
camera bridge decodes the two Pi streams in the MoveIt environment and
publishes stable ROS image topics for the UI, Trainer, and policies. MoveIt
Pro's generic video server subscribes to those image topics and serves WHEP/
WebRTC to the browser; the robot package does not implement browser transport.

## Why Hiroz is not in the control path

Hiroz proved that typed Zenoh endpoints can interoperate with
ROS 2, but it is not required here: the Pi can run the standard Feetech driver,
MoveIt controllers, and controller manager directly. ROS2DDS preserves their
native ROS graph. Removing the JSON command adapter also removes its browser
lease, command TTL, and watchdog failure modes from normal MoveIt operation.

## Transport configuration

All MoveIt processes in a hardware session use one loopback-only CycloneDDS
domain. The operator launcher starts an architecture-matched ROS2DDS client
before MoveIt Pro and connects it to the Pi router with multicast scouting
disabled. The Pi router exports only Pi-owned publishers and servers; the
workstation imports those endpoints and exports only command publishers and
action/service clients.

The directional allowlists are important. Importing and exporting the same
action endpoints on one side can reflect bridge-created DDS proxies and deliver
an action goal more than once.

The default endpoint is the Tailscale MagicDNS host on TCP 7448. The robot
hostname and endpoint remain environment variables, so no workstation address
is committed and a new Linux machine does not require changing the Pi's DDS
peer list.

## Readiness contract

Seeing a service name is not sufficient proof that a remote service proxy can
answer. Hardware startup must complete a typed
`controller_manager_msgs/srv/ListControllers` request before it reports ready
or permits the Agent to start. This prevents the UI from opening successfully
and then failing its first controller switch.

The corresponding pre-motion acceptance check is:

1. `/joint_states` advances continuously.
2. `list_controllers` answers and shows JTC/gripper/state broadcaster active,
   VFC/JVC inactive.
3. trajectory and gripper action servers are discoverable.
4. Teleoperate can switch JTC to VFC and back without losing joint state.
5. `Ready` completes, then short position-only base-frame Pose Jog commands
   move in all requested translation directions.
6. The manipulator tip, end-effector group, and IMarker frame resolve to the
   physical tool assembly; VFC and Pose Jog prediction interpret the selected
   robot-model control frame by the same orientation.

Deactivation is only a release of command interfaces, not a hold command.
Each incoming position controller must preserve the outgoing finite position
reference when available, use measured position as the fallback, and zero its
velocity and acceleration references before commanding hardware.
