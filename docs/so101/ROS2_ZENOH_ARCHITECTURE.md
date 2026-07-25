# ROS 2 and Zenoh architecture

## Decision

MoveIt Pro and the robot communicate as standard ROS 2 systems using
`rmw_zenoh_cpp`. The Pi runs the Feetech hardware interface and ros2_control
controllers locally. This matches the controller boundary used by
`legalaspro/so101-ros-physical-ai` and avoids translating controller traffic
through a custom dashboard protocol.

```text
MoveIt Pro workstation                         Raspberry Pi

Objectives / planning / Trainer
              |
FollowJointTrajectory + GripperCommand
              |
        rmw_zenoh_cpp  <---- router ---->  rmw_zenoh_cpp
                                                |
                                      controller_manager
                                      ├─ joint_state_broadcaster
                                      ├─ joint_trajectory_controller
                                      └─ gripper_controller
                                                |
                                      FeetechHardwareInterface
                                                |
                                          /dev/ttyACM0
```

The Pi is the only serial-bus owner and the only controller manager for
physical hardware. The MoveIt Pro hardware config sets
`launch_control_node: false` and lists the Pi controllers as
`controllers_not_managed`.

## ROS contract

| Entity | Type | Owner |
| --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | Pi joint-state broadcaster |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Pi JTC |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Pi gripper controller |
| `/controller_manager/list_controllers` | `controller_manager_msgs/srv/ListControllers` | Pi controller manager |
| `/so101/cameras/overhead/image_raw` | `sensor_msgs/msg/Image` | MoveIt camera bridge |
| `/so101/cameras/wrist/image_raw` | `sensor_msgs/msg/Image` | MoveIt camera bridge |
| `/get_action_chunk` | MoveIt Pro policy service | MoveIt VLA adapter |

RTSP camera traffic remains outside the ROS control router. The read-only
camera bridge decodes the two Pi streams in the MoveIt environment and
publishes stable ROS image topics for the UI, Trainer, and policies.

## Why Hiroz is not required

Hiros proved that typed Zenoh endpoints can interoperate with
`rmw_zenoh_cpp`, but it is not needed in the deployed boundary: the Pi can run
the standard ROS 2 Feetech driver and controller manager directly. Removing the
native JSON command adapter also removes its browser lease, command TTL, and
watchdog failure modes from normal MoveIt operation.

## Transport configuration

All ROS processes in a hardware session use the same domain ID and connect to
the Pi router with multicast scouting disabled. Linux injects this through the
workspace Compose override; Apple Container starts a local router that connects
to the Pi router. The robot hostname and endpoints remain environment
variables, so no machine-specific address is committed.
