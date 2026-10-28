# MirrorMe BPX Quadruped Robonix Primitive

Version **0.1.0-dev.4** unifies `odom`, exact `stand`/`sit` posture,
`move` and `twist_in` in one SDK session. Activation stays idle. Motion requires
standing and healthy feedback, but no manual arm or local lease. Fault recovery
uses standard DEACTIVATE -> ACTIVATE; no private clear-fault call is required.
The SDK is supplied externally and is not redistributed.

See the Robot repository's `UNIFIED_TEST_GUIDE.md` for the current launch and
standard-RPC tests. Release scope and validation are recorded in [RELEASE_NOTES.md](RELEASE_NOTES.md).

The SDK baseline is v1.0.9, pinned to public repository commit
`f8d73125c04bfca81ae7b7d7d4b3071285ed4303`. Download the CPython 3.10 Linux
x86_64 wheel from the fixed
[`v1.0.9` release](https://github.com/mirrormerobotics/bpx_sdk_open/releases/tag/v1.0.9),
then pass its local path using `--sdk-wheel` to the launcher. The adapter does not follow
the moving `master` HEAD implicitly.

See [SDK_COMPATIBILITY.md](SDK_COMPATIBILITY.md) for the supported stable artifacts and validation.
dev.4 supports SDK 1.0.9 and retains 1.0.8 fallback. Prereleases 1.0.10/1.0.11 are not enabled.

## Current capability surface

- Default and x86 Docker manifests: `odom`, `posture`, `move`, `twist_in`;
  `start-sdk-unified.sh` uses one `MotionRuntime` and one `SdkMotionBackend`.
- `package_manifest.unified-fake.yaml`: the same four contracts, simulated state.
- SDK read-only/replay: odom only; no movement path.
- `package_manifest.posture-only.yaml` and posture-test: legacy stand/sit-only runtime.
- motion-test and motion-dev: legacy development profiles, not the unified release.

Posture and move are mutually exclusive. Posture stops an active velocity stream;
new Twist input is rejected while a posture/move operation owns the channel.
Posture accepts only stand/sit and honors RPC cancellation and configured timeouts.
Each move stops on completion, failure or cancellation. Command/state watchdogs,
limits and fault latching remain active; shutdown never automatically sits.

## Safety defaults

- The release start entry requires the pinned SDK wheel, forces `backend=sdk`,
  and enables the four unified contracts.
- The generic runtime config still defaults to `fake`, but only the explicitly
  selected fake manifests may use that backend.
- `allow_motion` defaults to `false`; production posture configuration must
  explicitly set `allow_motion=true`, `enable_posture_service=true` and `enable_motion_service=true`.
- Activation validates fresh feedback but sends no
  posture command.
- The controlled fake path emits zero for invalid, stale, disconnected,
  deactivated and shutdown states. Unified SDK motion requires standing feedback.
  DISARMED standby feedback gaps wait for 1 second of stable samples; control-time
  feedback faults remain latched until explicit lifecycle recovery.
- No code in this package calls `setZeroPositionsFlag()`.

## Offline tests

```bash
bash scripts/test.sh
```

## Docker build

The image uses Ubuntu 22.04 through `ros:humble-ros-base`:

```bash
bash scripts/build-container.sh
```

This builds a test target first and only tags the runtime image after the tests
pass. The default image name is `primitive-mirrorme-bpx-quadruped-rbnx:dev`.
The Robonix wire dependencies are pinned in the image to the versions used by
the fixed Robonix source baseline; `grpcio-tools` remains confined to the
code-generation tool image.

## Verify the fixed SDK wheel

The wheel stays outside this repository. Verification mounts it read-only into
an isolated container and checks the recorded SHA-256 before importing it:

```bash
bash scripts/verify-sdk-wheel.sh /path/to/bpx_sdk_open-1.0.9-cp310-...x86_64.whl
```

For a bounded, state-only hardware capture:

```bash
bash scripts/run-read-only-probe.sh /path/to/the-wheel \
  --robot-ip 10.21.20.1 --duration-s 10
```

The probe constructs `RequestRobotState` only. It never constructs a motion or
joint controller and never calls `setZeroPositionsFlag()`. It reports ready
only after both the TCP session and the first odometry frame are available.
In this document, read-only means there is no actuator-command interface. The
vendor `connect()` sequence still configures state upload and performs the
SDK's built-in robot-time synchronization handshake.

Native Linux Docker should keep the default host networking. Docker Desktop
does not expose the robot's UDP state upload to a host-network container in the
same way; use an explicit bridge-mode UDP mapping for the bounded probe and
recorder:

```bash
bash scripts/run-read-only-probe.sh /path/to/the-wheel --network-mode bridge \
  --robot-ip 192.168.1.237 --robot-state-port 9873 --duration-s 10
```

The selected UDP port must be free on the Docker host. The integrated runtime
also accepts bridge mode. In that topology Atlas must listen on an address
reachable from the Docker bridge, and `--docker-host-address` must identify
the address the Primitive container uses for Atlas (for the tested nested
Docker Desktop topology, the local Docker bridge gateway). The container
automatically advertises its own bridge IPv4 lifecycle endpoint. Native Linux
Docker should retain the default host-network mode.

The reusable `SdkStateSource` follows the same boundary. Its readiness wait is
bounded, it maps leg odometry/IMU/joints/temperatures/motion/battery telemetry,
and its host freshness timestamp advances only when the BPX odometry timestamp
changes. It intentionally has no command method.

## Explicit stand/hold/sit hardware probe

After the physical E-stop, clear test area, operator presence, robot condition,
and control-source arbitration have been checked, one stand/hold/sit cycle can
be run with the narrow MotionLevelControl probe:

```bash
bash scripts/run-posture-cycle-probe.sh /path/to/the-wheel --network-mode bridge \
    --robot-ip 192.168.1.237 \
    --robot-state-port 9873 \
    --hold-duration-s 3 \
    --confirm-physical-motion
```

Use the default host network instead on native Linux. The wrapper first checks
the pinned wheel checksum and required MotionLevelControl API, then requires a
fresh `LyingDown(0)` state before sending anything. It repeats `setStandUp()`
until `Motion(6)`, verifies that state throughout the three-second hold, and
repeats `setSitDown()` until `LyingDown(0)`. Failure, SIGINT, or SIGTERM after
the stand phase starts triggers a bounded sit-down cleanup before disconnect.

This probe never calls `setVelocity()`, `setZeroPositionsFlag()`, gait or
damping commands, and never constructs `JointLevelControl`. It remains a
manual diagnostic; production posture uses the same narrow SDK seam.

## Robonix posture hardware-test profile

`package_manifest.posture-test.yaml` connects the same narrow stand/sit seam to
`robonix/primitive/quadruped/posture` under a separate diagnostic identity.
The unified release candidate publishes the same RPC without the test-only startup
acknowledgement. The diagnostic profile requires all of the following:

- the pinned external SDK wheel;
- `backend=sdk`, `allow_motion=true`, and `enable_posture_service=true`;
- `--confirm-physical-posture-service` when using the operator launcher;
- a local operator with the physical E-stop and a clear test area.

The RPC accepts only exact lowercase `stand` and `sit`. Calls are serialized,
each response waits for fresh `MotionState` feedback, and duplicate requests
are idempotent. Activation accepts a fresh `LyingDown(0)` or `Motion(6)` state
without moving. Command failure during a posture transition performs a bounded
sit-down attempt. Deactivate and shutdown disable velocity mode and disconnect
without implicitly changing posture. The profile
does not register `twist_in` or `move`, never enables velocity control, and
does not construct `JointLevelControl`.

## Explicit stand/yaw-circle/stop/sit hardware probe

The separate `scripts/run-yaw-circle-probe.sh` wrapper is for a manually
initiated test only. It stands from `LyingDown(0)`, enables velocity control,
holds linear x/y at exactly zero, and sends a bounded yaw rate until the
unwrapped LegOdom yaw reaches `2*pi` in the commanded direction. It then sends
zero yaw, disables velocity control, flushes zero for two seconds, and sits to
`LyingDown(0)`.

Defaults are `0.5 rad/s`, a `2*pi` target, a 30-second turn timeout, and a
0.15-metre maximum horizontal translation from the standing origin. Feedback
more than 0.2 rad in the direction opposite to the command aborts the cycle.
Negative yaw rate selects the other direction. The adapter hard-caps absolute
yaw at `1.0 rad/s` even though the SDK documents a higher Walk limit.

The vendor C++ example sends `1.0 rad/s` for three seconds, which is about
3 radians rather than a full circle. This probe follows its velocity-enable,
periodic command, zero-flush and sit ordering, but uses odometry closure to
implement the requested full circle. It omits the example's zero-position and
unrelated gait commands. A fake-clock regression also exercises the C++
example's nominal `1.0 rad/s`/`3 rad` phase and locks the cross-command order,
while retaining state-confirmed stand/sit and bounded cleanup.

See the Robot repository's `YAW_CIRCLE_TEST_GUIDE.md` before manual use. The
wrapper requires `--confirm-physical-yaw-circle`, and no package manifest
launches it. This direct SDK test still does not create a Robonix `twist_in`
subscriber.

All state sources pass through one telemetry boundary that rejects malformed
dimensions, non-finite values, invalid timestamps and zero-norm quaternions;
valid non-unit odometry/IMU quaternions are normalized before publication.

To create a bounded normalized recording once hardware is available:

```bash
bash scripts/record-sdk-replay.sh /path/to/the-wheel /path/to/capture.jsonl \
  --robot-ip 10.21.20.1 --duration-s 30
```

The recorder is read-only, refuses to overwrite an existing file, disconnects
in `finally`, and prints an audit report covering rate, event gaps, timestamp
duplicates/regressions, disconnects and estimated missing odometry frames.

## State replay profile

`package_manifest.state-replay.yaml` forces `backend=replay` and declares only
`odom`. The bundled synthetic fixture can exercise the production Robonix/ROS
publication path without the wheel or robot:

```bash
cd ../robot-mirrorme-bpx
bash scripts/test-offline-integration.sh /path/to/fixed/robonix replay
```

This test performs Driver DEACTIVATE/ACTIVATE, receives `/odom` and
`/joint_states` plus `odom -> base_link`, checks the single-publisher topology,
and verifies shutdown cleanup. The fake variant also publishes a non-planar
Twist and requires the provider to reject it.

For the integrated SDK runtime, supply the same pinned wheel as an external
host artifact. The default and x86 Docker release manifests select the unified SDK
wrapper. The --release-posture launcher option retains the legacy posture-only variant. It verifies the recorded SHA-256, mounts that one file
read-only, and installs it without contacting a package index:

```bash
bash ../robot-mirrorme-bpx/scripts/start-sdk-read-only-docker.sh \
  --sdk-wheel /absolute/path/to/bpx_sdk_open-1.0.9-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl \
  --robot-ip 192.168.1.237 --release-posture \
  --confirm-physical-posture-service
```

For normal deployment, use the Robot package's
`robonix_manifest.yaml` profile. Use `robonix_manifest.sdk-read-only.yaml` when
posture must be unavailable. Hardware activation remains bounded by
`sdk_connect_timeout_s` and fails if no fresh odometry frame arrives.

## Robonix build and start

Once `rbnx` is installed and configured against a fixed Robonix source tree:

```bash
rbnx validate .
rbnx build -p .
bash ../robot-mirrorme-bpx/scripts/start-sdk-read-only-docker.sh \
  --sdk-wheel /absolute/path/to/the-pinned-wheel \
  --robot-ip 192.168.1.237 --release-posture \
  --confirm-physical-posture-service
```

For a fake, motion-disabled development smoke test without hardware, explicitly
select the non-release identity:

```bash
rbnx start -p . --manifest package_manifest.motion-dev.yaml \
  --set backend=fake --set allow_motion=false
```

For standalone `rbnx start`, the wrapper defaults `RBNX_INSTANCE_NAME` to the
manifest package name, `robonix.primitive.mirrorme.bpx.quadruped`. An instance
name supplied by a deployment remains authoritative.

To validate and generate against the fixed Robonix source entirely in Docker:

```bash
bash scripts/verify-rbnx.sh /path/to/robonix
```

The script checks the expected Robonix commit and recorded interface submodule
commits, builds `rbnx` in a pinned Rust image, and uses the `grpcio-tools`
versions from that Robonix lockfile. All tool caches stay under `.artifacts/`.

The start wrapper bind-mounts the package and `robonix-api` into the container.
When `--sdk-wheel` is supplied (internally `BPX_SDK_WHEEL`), it additionally verifies the pinned artifact and
bind-mounts only that wheel under `/vendor-sdk` with `:ro`. The artifact remains
outside the repository and runtime image so the release can preserve its own
Apache-2.0 LICENSE, NOTICE, third-party notices, checksum, and SBOM boundary.

The fixed-baseline test suite currently passes all 102 package tests. The
previous Atlas lifecycle smoke test used the distinct fake motion-development
identity: it reached `ACTIVE` with Driver, `twist_in`, and `odom`; its internal
`/joint_states` stream was consumed by ROS 2 `robot_state_publisher` to resolve
all four toe TF chains. Driver `CMD_SHUTDOWN` transitioned it to `TERMINATED`
without leaving a runtime container behind.

## Development status

See the workspace-level `TODO.md`. The published package can read state and
perform only explicit `stand`/`sit`; it cannot accept a velocity command. Do
not promote the motion-development identity or use its Twist path with a real
robot until the controlled-motion milestone and hardware checklist are complete.

## License

This Primitive is licensed under the Apache License 2.0. See [LICENSE](LICENSE)
and [NOTICE](NOTICE). The separately distributed BPX SDK is also Apache-2.0,
but retains its own LICENSE, NOTICE, third-party notices, and release metadata.

## Operator command-line options

Operator scripts accept command-line options.
See [SCRIPT_CLI.md](SCRIPT_CLI.md) for image, network, container, cache and SDK
options. `bash scripts/<name>.sh --help` is side-effect free.
## Unreleased motion development

`package_manifest.motion-test-fake.yaml` and `package_manifest.motion-test.yaml`
provide supervised `twist_in` and standard bounded `move` without a local arm prerequisite,
feedback guards, cancellation and stop cleanup. Neither is a release manifest.
See the sibling Robot repository's `MOTION_TEST_GUIDE.md` for operator commands
and pending physical acceptance. Hardware behavior has not been verified.
