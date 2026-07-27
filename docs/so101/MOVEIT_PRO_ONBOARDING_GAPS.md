# MoveIt Pro Robot Onboarding: Field Gaps

This is a reusable field note from onboarding a physical robot whose
`ros2_control` controller manager runs on a Raspberry Pi while MoveIt Pro runs
on an operator workstation. It is intentionally not an SO-101 setup guide.

## What the current documentation covers

- [Create Robot Mock Configuration](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_mock_config/create_mock_robot_config/)
  and [Create Robot Physical Configuration](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_physical_config/create_robot_physical_config/)
  establish the mock/simulation/hardware package workflow.
- [hardware Values](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_mock_config/hardware_configuration/)
  covers URDF/SRDF, driver launch files, base/tip inspection, and
  `launch_control_node: false` for an externally owned controller manager.
- [ros2_control Values](https://docs.picknik.ai/how_to/configuration_tutorials/create_robot_mock_config/ros2_control_configuration/)
  lists the trajectory, Pose Jog, Joint Jog, and state controllers required by
  Teleoperate.
- [UI-Based Teleoperation](https://docs.picknik.ai/how_to/custom_view_panes/ui_teleoperation/)
  and [ros2_control in MoveIt Pro](https://docs.picknik.ai/concepts/ros2_control/)
  explain the modes, controller switching, and runtime safety behavior.

## Gaps encountered in a complete physical bring-up

| Challenge | Existing guidance | Reusable addition needed |
| --- | --- | --- |
| Placing the high-rate control loop | External controller-manager ownership is documented | Add a reference topology where controller manager, VFC/JVC, and hardware interfaces run on the robot computer while MoveIt Pro plans remotely. |
| Proving a remote controller manager is ready | Required service names are documented | Require a typed `ListControllers` response. Discovery of a service name alone can precede a usable proxy, causing the first `SwitchController` to fail. |
| Bridging the graph without reflected goals | ROS endpoints are documented individually | Show directional ROS2DDS/Zenoh allowlists. A bridge that imports and exports the same action endpoints can reflect its own proxies and deliver a goal twice. |
| Initializing a gravity-loaded position controller | Controller switching is documented | Require the new controller to inherit a finite held position reference, fall back to measured position only when needed, and zero velocity/acceleration. Otherwise activation can cause a drop, following-error abort, or error-ratcheting sag. |
| Distinguishing UI Stop from a safety function | Runtime stop behavior is documented | State explicitly that UI Stop is cooperative software control, not a safety-rated E-stop; physical isolation and risk controls remain the integrator's responsibility. |
| Supporting an underactuated Cartesian arm | Pose Jog configuration is documented | Document `position_only` Pose Jog as the normal choice for arms that cannot control all six Cartesian dimensions. |
| Keeping Pose Jog prediction consistent with execution | Control-frame selection and boundary checking are documented separately | State that the safety predictor must integrate linear and angular commands in the same control frame used by VFC. A tip-frame predictor can reject a valid base-frame jog. |
| Aligning robot limits across layers | URDF, MoveIt, and controller limits are documented separately | Add one table/check that compares URDF limits, MoveIt limits, controller limits, driver limits, and actual measured coordinates. Mock hardware can hide a mismatch. |
| Distinguishing tuning from power trouble | Execution scaling and tolerances are documented | Add measured-state diagnostics for speed/acceleration, reference lag, voltage/current, and tolerance aborts. Repeated errors just above a threshold are usually configuration or handoff evidence, not a reason to keep widening tolerance. |
| Defining a usable first waypoint | Waypoint authoring and collision checking are documented | Recommend a collision-checked bent pose with clearance in both directions for Cartesian jogging, rather than a singular, folded, or workspace-edge pose. |
| Aligning end-effector frames | Planning-group base/tip inspection is documented | Include a complete SRDF example relating manipulator tip, end-effector parent link, end-effector group, and computed tip. The common symptom is an IMarker one link above the gripper. |
| Bringing cameras through the product path | ROS image panes are documented | Separate source transport from UI transport: a robot may publish RTSP, bridge it to ROS `Image`, and let MoveIt Pro's generic WebRTC server deliver WHEP to the browser. Verify decoded frames, not merely topic discovery. |
| Knowing onboarding is complete | Individual pages contain local checks | Provide one acceptance checklist covering controller states, continuous joint states, controller switching, a real waypoint, Pose/Joint Jog, gripper, camera frames, and restart/reconnect behavior. |

## Suggested documentation shape

The smallest useful addition is an **External Hardware Controller Host** page
linked from both `hardware.launch_control_node` and the teleoperation
controller table. It should contain one topology, a ROS endpoint and
controller-state matrix, a typed readiness probe, directional bridge examples,
and one end-to-end acceptance checklist.
