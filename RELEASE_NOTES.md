# v0.1.0-dev.4 — 2026-10-28

Support the pinned BPX SDK 1.0.9 stable wheel, retaining 1.0.8 fallback.
Reject 1.0.10/1.0.11 prereleases in this release. Capability scope remains
odom, stand/sit, move and twist_in; no new motion or gait interfaces.
Robot pins Primitive v0.1.0-dev.4. Build/start use the versioned dev.4 runtime
image to avoid reusing an entrypoint that accepts only SDK 1.0.8.
Validation: 196 Primitive + 46 Robot unit tests, model generation, isolated fake
Atlas/ROS/gRPC integration, SDK 1.0.9 native loopback and actual image entrypoint,
and SDK 1.0.8 unified API fallback checks passed.
Native SDK checks and loopback tests are offline; SDK 1.0.9 hardware testing
remains pending. No physical robot commands are issued during release validation.

# v0.1.0-dev.3 — 2026-10-23

Unified BPX control: one SDK owner provides odom, exact stand/sit posture,
bounded move and continuous twist_in through standard Robonix interfaces.
Startup stays idle. Fresh movement requests no longer require a private arm call
or a 30-second local lease. Posture and bounded moves share operation ownership.

DISARMED standby stale/disconnected feedback holds zero output, refuses requests,
and returns to ready only after 1 second of continuous healthy observations.
The 0.5-second feedback limit is unchanged. Control-time faults, invalid data,
SDK preparation/stop failures remain latched. Standard DEACTIVATE -> ACTIVATE
recovers after fixing the cause; old commands are never replayed. Read-only and
legacy posture-only profiles remain available.

Validation: 194 Primitive + 46 Robot unit tests, model/Soma consistency,
rbnx manifest validation and fresh protobuf/ROS code generation. A network-isolated
fake Atlas/provider integration passed standard posture/move calls, ROS twist/odom,
command watchdog and Driver fault recovery without arm/clear-fault.
The operator confirmed unified status -> stand -> 0.1m forward -> 15deg rotate ->
sit on 2026-10-23. This does not claim externally measured accuracy or a separate
hardware fault-injection campaign for this version.

Supported SDK artifact: checksum-pinned BPX 1.0.8 CPython 3.10 Linux x86_64 wheel,
supplied externally (not included in source or runtime image). Robonix validation
baseline: f6cacfc4b402eb57c75b553d375c6cfaba11dce7. Other architectures/firmware and
current upstream compatibility are not inferred from this baseline.
No navigation, mapping, gait tricks or joint-level control is added.

Both repositories use v0.1.0-dev.3. Robot robonix_manifest.yaml pins the Primitive
tag and selects package_manifest.x86-docker.yaml. Configure the actual robot IP
and BPX_SDK_WHEEL before deployment. The existing Catalog entries index master;
no archive upload to the core Robonix repository is needed.
