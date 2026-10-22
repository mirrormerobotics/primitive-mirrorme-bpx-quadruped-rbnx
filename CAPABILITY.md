---
description: MirrorMe BPX quadruped odometry and guarded stand/sit posture.
---

# MirrorMe BPX Quadruped

The published package identity exposes
`robonix/primitive/quadruped/odom` and
`robonix/primitive/quadruped/posture`. Its SDK runtime owns one
`MotionLevelControl` instance but exposes only exact `stand` and `sit` RPCs;
velocity mode remains disabled and it does not create or declare `twist_in`.
The explicit SDK read-only and normalized-replay profiles expose only odom.

An explicitly non-release `motion_dev` package identity keeps the guarded fake
Twist path available for offline safety tests. It is not a supported hardware
capability and must not be submitted to the Robonix Package Catalog as a
release variant.

An isolated, explicitly confirmed hardware probe now owns one
`MotionLevelControl` object and supports only a bounded
`setStandUp()` -> three-second hold -> `setSitDown()` cycle. A distinct,
hardware-test-only package identity also exposes the same two commands as the
Robonix `posture` RPC with state feedback, serialization, timeouts, and
bounded failure cleanup. The validated stand/sit seam is also the production
posture implementation. A second
manually confirmed probe uses `setVelocity(0, 0, yaw)` only for one bounded,
LegOdom-closed-loop yaw circle between stand and sit. Only feedback in the
commanded direction counts toward completion; bounded opposite-direction,
translation, timeout, and interruption failures zero yaw and attempt to sit.
It is likewise absent from every release manifest. The non-release
`motion_test` and `motion_test_fake` identities now implement general
`twist_in` and standard bounded `move`, with local-only arm leases, feedback
guards, cancellation, deadlines and zero cleanup. Their hardware acceptance
is pending; see the Robot repository's `MOTION_TEST_GUIDE.md`.
`JointLevelControl` will not be integrated.

Bounded `motion_test` distance/angle moves use configurable minimum targets
of 0.25 m/s and 0.25 rad/s by default to avoid the documented Walk deadzone.
They stop on tolerance or fail on overshoot without automatic reversal.
This does not boost continuous Twist or timed velocity requests. Acceleration,
velocity limits, cancellation and local leases still apply; physical stopping
error and hardware response require acceptance. See `M5_VALIDATION_2026-10-14.md`
in the Robot repository before testing the changed bounded-move behavior.

All runtimes publish the internal ROS 2 `/joint_states` topic with the 12 names
defined by the pinned BPX SDK and official URDF. This topic is not advertised
as a Robonix capability because no quadruped JointState contract exists in the
fixed baseline.

Current restrictions:

- Mapping, localization, autonomous navigation, obstacle avoidance, lidar and
  camera perception are outside the BPX product scope; odometry is body-state
  feedback and does not imply navigation support.
- Activation never moves the robot. Production use of `backend=sdk` loads a
  separately mounted, checksum-verified v1.0.8 wheel and enables only the
  stand/sit posture service with `allow_motion=true` and
  `enable_posture_service=true`.
- The explicit SDK read-only profile constructs `RequestRobotState` only with
  `allow_motion=false`; the posture-test identity additionally requires local
  acknowledgement.
- `backend=replay` requires an absolute versioned JSONL path, remains read-only,
  and preserves recorded freshness and disconnect events.
- Unknown, non-finite, stale or out-of-state commands are rejected or reduced
  to a strict zero command.
- Flip, running, jumping, handstand and low-level joint control are not exposed.
- Bounded move and continuous twist contracts are not declared by a release
  manifest. Posture supports only `stand` and `sit`.
- JointState effort is omitted until the SDK torque unit is confirmed.
