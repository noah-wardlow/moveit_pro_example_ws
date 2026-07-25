# SO-101 objectives

Start with MoveIt Pro's core joint-space objectives. The SO-101 has five arm
degrees of freedom, so arbitrary six-dimensional Cartesian poses are not
generally reachable; position-only or orientation-sampled objectives should
be used when adding Cartesian workflows.

`Execute SO101 VLA Policy` uses MoveIt Pro's native `ExecutePolicy` Behavior
with the stable overhead and wrist camera topics. It switches only the five arm
joints to JTAC, leaves the single `Jaw_R` gripper controller active, and restores
the normal joint trajectory controller after either success or ordinary
failure. A Behavior Tree cancellation halts immediately, so run `Restore SO101
Planning Controller` after any cancelled or externally interrupted policy.

Bring up a policy in this order:

1. Start `pixi run hold-policy`; it returns the observed state and requests no
   motion.
2. Confirm `/get_action_chunk` and both image topics are live.
3. Run the VLA Objective in simulation.
4. Only after the trained policy's joint order, units, rate, and calibration
   are verified should its endpoint replace the hold server.

The live hardware configuration remains read-only unless the existing local
authorization marker, Keychain-staged control key, Pi write window, and lease
are all present. Starting MoveIt Pro or viewing cameras does not satisfy those
gates.
