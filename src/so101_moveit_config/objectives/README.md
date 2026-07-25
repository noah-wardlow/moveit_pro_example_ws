# SO-101 objectives

Start with MoveIt Pro's core joint-space objectives. The SO-101 has five arm
degrees of freedom, so arbitrary six-dimensional Cartesian poses are not
generally reachable; position-only or orientation-sampled objectives should
be used when adding Cartesian workflows.

`Execute SO101 VLA Policy` uses MoveIt Pro's native `ExecutePolicy` Behavior
with the stable overhead and wrist camera topics. It sends trajectories through
the same standard `joint_trajectory_controller` used by waypoint execution;
the Pi remains the sole controller manager and serial-bus owner.

Bring up a policy in this order:

1. Start `pixi run hold-policy`; it returns the observed state and requests no
   motion.
2. Confirm `/get_action_chunk` and both image topics are live.
3. Run the VLA Objective in simulation.
4. Only after the trained policy's joint order, units, rate, and calibration
   are verified should its endpoint replace the hold server.

For physical named-waypoint motion, either use the hardware-only `Move Live
SO101 to Waypoint` Objective or start `Teleoperate` and select a waypoint. Both
use `/joint_trajectory_controller/follow_joint_trajectory`. The hardware config
ships a compatibility override for `Request Teleoperation` so released MoveIt
Pro builds seed the velocity scale and select the standard JTC pipeline.

For no-motion validation, the simulation config alone loads `Move Simulated
SO101 to Waypoint`. It also defaults to `Ready` and uses the same standard
JTC/JTC execution path, but omits the hardware readiness service because no
physical command boundary exists in MuJoCo.
