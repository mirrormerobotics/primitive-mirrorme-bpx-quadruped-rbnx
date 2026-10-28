# SDK compatibility — v0.1.0-dev.4

Baseline: BPX SDK **1.0.9**. SDK **1.0.8** remains supported for rollback.
Only the checksum-pinned CPython 3.10 Linux x86_64 wheels in `sdk/SHA256SUMS`
are supported. Prereleases 1.0.10/1.0.11 are rejected by this release.
Their experimental implementation is preserved on the local sdk-1.0.9-validation branch.

| SDK | Source commit | Verification |
|---|---|---|
| 1.0.8 | cab478a5fd3dfce087e0774534bb87b51a3d69e1 | Existing baseline; retained fallback |
| 1.0.9 | f8d73125c04bfca81ae7b7d7d4b3071285ed4303 | Official v1.0.9 wheel, SHA256 verified against release listing |

The unified API check covers state getters, read-only separation, posture,
setWalk and setVelocity. The native SDK loopback test runs in a disposable
Docker container with `--network none`: simulated TCP handshake, UDP feedback,
velocity/zero/stand/sit packets and disconnect. It does not prove physical
movement or firmware compatibility. Its minimal peer rejects clock synchronization;
teardown connection messages are expected. No robot is contacted.

```bash
bash scripts/build-container.sh
bash scripts/verify-sdk-wheel.sh ../.deps/sdk/bpx_sdk_open-1.0.9-cp310-cp310-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl --unified --loopback
```

Build the new `primitive-mirrorme-bpx-quadruped-rbnx:0.1.0-dev.4` image before
starting: old images embed the old 1.0.8-only entrypoint. Startup/build scripts
now default to the versioned image so an old `:dev` image is not reused silently.
The SDK remains externally supplied, not bundled in the image.

Unchanged capability scope: odom, stand/sit, move, twist_in. No jump or gait
selection is added. SDK 1.0.9 hardware verification remains pending; prior SDK
1.0.8 hardware acceptance does not imply that the new SDK has been field-tested.
