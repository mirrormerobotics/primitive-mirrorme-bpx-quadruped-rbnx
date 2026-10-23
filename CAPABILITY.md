---
description: MirrorMe BPX unified odometry, stand/sit, bounded move and velocity control.
---

# MirrorMe BPX Quadruped — dev.3

Default/x86 manifests expose `robonix/primitive/quadruped/{odom,posture,move,twist_in}`
through one SDK MotionLevelControl owner. Posture accepts exact `stand`/`sit` only.
Startup stays idle; fresh motion commands require standing/healthy feedback but no
manual arm. move and posture serialize; posture stops a Twist stream before transition.
Command/state watchdogs, speed/acceleration limits, fault latching and cancellation
remain. Recover faults through DEACTIVATE -> ACTIVATE after fixing their cause.
Local clear-fault and arm remain optional diagnostic compatibility tools.

Bounded moves use initial-heading distance or accumulated yaw, limits 1 m / 360 deg
and 30 s; min targets 0.25 m/s and 0.25 rad/s address SDK deadzone. Timed/Twist
commands are not boosted. Rotation has no translation-distance guard.

Read-only SDK/replay remain odom-only. Legacy posture-only and test manifests remain
available. JointState, optional IMU and diagnostics are internal ROS publications.
No mapping, navigation, gait tricks, damping or joint-level control is provided.
SDK v1.0.8 remains a separately supplied, checksum-pinned artifact. Physical accuracy,
extrinsics and covariance must not be inferred from simulated tests.

See Robot `UNIFIED_TEST_GUIDE.md` for offline evidence and manual hardware checks.
The operator confirmed unified stand -> 0.1m move -> 15deg rotation -> sit on
2026-10-23. This is not a measured physical-accuracy claim. See RELEASE_NOTES.md.
