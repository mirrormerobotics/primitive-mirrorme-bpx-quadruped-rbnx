from __future__ import annotations

import os
from pathlib import Path
import unittest


PACKAGE_ROOT = Path(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REPOSITORY_NAME = "primitive-mirrorme-bpx-quadruped-rbnx"
PACKAGE_NAME = "robonix.primitive.mirrorme.bpx.quadruped"
DISTRIBUTION_NAME = "mirrorme-bpx-quadruped-rbnx"
IMAGE_NAME = "primitive-mirrorme-bpx-quadruped-rbnx"


class PackagingNamesTest(unittest.TestCase):
    def test_release_metadata_uses_apache_2_0(self) -> None:
        license_text = (PACKAGE_ROOT / "LICENSE").read_text(encoding="utf-8")
        notice = (PACKAGE_ROOT / "NOTICE").read_text(encoding="utf-8")
        setup = (PACKAGE_ROOT / "setup.cfg").read_text(encoding="utf-8")

        self.assertIn("Apache License", license_text)
        self.assertIn("Version 2.0, January 2004", license_text)
        self.assertIn("Hangzhou MirrorMe Technology Co., Ltd.", notice)
        self.assertIn("license = Apache-2.0", setup)
        self.assertIn("license_files =", setup)

        for filename in (
            "package_manifest.yaml",
            "package_manifest.x86-docker.yaml",
            "package_manifest.sdk-read-only.yaml",
            "package_manifest.state-replay.yaml",
            "package_manifest.motion-dev.yaml",
            "package_manifest.posture-test.yaml",
        ):
            manifest = (PACKAGE_ROOT / filename).read_text(encoding="utf-8")
            self.assertIn("license: Apache-2.0", manifest)

    def test_repository_and_manifest_names_include_vendor(self) -> None:
        self.assertEqual(REPOSITORY_NAME, PACKAGE_ROOT.name)
        for filename in (
            "package_manifest.yaml",
            "package_manifest.x86-docker.yaml",
            "package_manifest.sdk-read-only.yaml",
            "package_manifest.state-replay.yaml",
        ):
            manifest = (PACKAGE_ROOT / filename).read_text(encoding="utf-8")
            self.assertIn("name: {}".format(PACKAGE_NAME), manifest)
            self.assertIn("mirrorme", manifest)

    def test_release_manifests_publish_all_unified_contracts(self) -> None:
        for filename in (
            "package_manifest.yaml",
            "package_manifest.x86-docker.yaml",
        ):
            manifest = (PACKAGE_ROOT / filename).read_text(encoding="utf-8")
            self.assertIn("name: {}".format(PACKAGE_NAME), manifest)
            self.assertEqual(
                4,
                manifest.count("  - name: robonix/primitive/quadruped/"),
            )
            self.assertIn("  - name: robonix/primitive/quadruped/odom", manifest)
            self.assertIn("  - name: robonix/primitive/quadruped/posture", manifest)
            self.assertIn("robonix/primitive/quadruped/twist_in", manifest)
            self.assertIn("robonix/primitive/quadruped/move", manifest)
            self.assertIn("start: bash scripts/start-sdk-unified.sh", manifest)

        for filename in (
            "package_manifest.sdk-read-only.yaml",
            "package_manifest.state-replay.yaml",
        ):
            manifest = (PACKAGE_ROOT / filename).read_text(encoding="utf-8")
            self.assertIn("read-only", manifest)
            self.assertEqual(
                1,
                manifest.count("  - name: robonix/primitive/quadruped/"),
            )
            self.assertIn("  - name: robonix/primitive/quadruped/odom", manifest)
            self.assertNotIn("robonix/primitive/quadruped/posture", manifest)

    def test_motion_development_manifest_has_a_distinct_fake_only_identity(self) -> None:
        path = PACKAGE_ROOT / "package_manifest.motion-dev.yaml"
        text = path.read_text(encoding="utf-8")

        self.assertTrue(text.startswith("# DEVELOPMENT ONLY:"))
        self.assertIn(
            "name: robonix.primitive.mirrorme.bpx.quadruped.motion_dev", text
        )
        self.assertNotIn("name: {}\n".format(PACKAGE_NAME), text)
        self.assertIn("development", text)
        self.assertIn("start: bash scripts/start-motion-dev.sh", text)
        self.assertIn("robonix/primitive/quadruped/twist_in", text)

        wrapper = (PACKAGE_ROOT / "scripts/start-motion-dev.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("export BPX_REQUIRED_BACKEND=fake", wrapper)
        self.assertIn("quadruped.motion_dev", wrapper)

    def test_posture_test_manifest_is_distinct_and_requires_local_confirmation(self) -> None:
        manifest = (PACKAGE_ROOT / "package_manifest.posture-test.yaml").read_text(
            encoding="utf-8"
        )
        wrapper = (PACKAGE_ROOT / "scripts/start-posture-test.sh").read_text(
            encoding="utf-8"
        )
        provider = (PACKAGE_ROOT / "bpx_quadruped/main.py").read_text(
            encoding="utf-8"
        )

        self.assertTrue(manifest.startswith("# HARDWARE TEST ONLY:"))
        self.assertIn("quadruped.posture_test", manifest)
        self.assertIn("robonix/primitive/quadruped/posture", manifest)
        self.assertNotIn("robonix/primitive/quadruped/twist_in", manifest)
        self.assertIn("start-posture-test.sh", manifest)
        self.assertIn("BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE", wrapper)
        self.assertIn("I_UNDERSTAND_POSTURE_RPC_CAN_MOVE_BPX", wrapper)
        self.assertIn("BPX_POSTURE_CAPABILITY=true", wrapper)
        self.assertIn('primitive.grpc("robonix/primitive/quadruped/posture")', provider)

    def test_distribution_and_default_image_include_vendor(self) -> None:
        setup = (PACKAGE_ROOT / "setup.cfg").read_text(encoding="utf-8")
        build_script = (PACKAGE_ROOT / "scripts/build-container.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("name = {}".format(DISTRIBUTION_NAME), setup)
        self.assertIn("{}:dev".format(IMAGE_NAME), build_script)

    def test_runtime_pins_robonix_api_wire_dependencies(self) -> None:
        dockerfile = (PACKAGE_ROOT / "docker/Dockerfile").read_text(encoding="utf-8")
        self.assertIn("GRPCIO_VERSION=1.80.0", dockerfile)
        self.assertIn("PROTOBUF_VERSION=6.33.6", dockerfile)
        self.assertIn('"grpcio==${GRPCIO_VERSION}"', dockerfile)
        self.assertIn('"protobuf==${PROTOBUF_VERSION}"', dockerfile)
        self.assertNotIn("grpcio-tools==", dockerfile)

    def test_standalone_start_defaults_instance_name_to_package_name(self) -> None:
        start_script = (PACKAGE_ROOT / "scripts/start.sh").read_text(encoding="utf-8")
        self.assertIn(
            'RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-%s}"' % PACKAGE_NAME,
            start_script,
        )
        self.assertIn("export RBNX_INSTANCE_NAME", start_script)

        production_wrapper = PACKAGE_ROOT / "scripts/start-sdk-posture.sh"
        wrapper = production_wrapper.read_text(encoding="utf-8")
        self.assertTrue(os.access(production_wrapper, os.X_OK))
        self.assertIn("export BPX_REQUIRED_BACKEND=sdk", wrapper)
        self.assertIn("export BPX_POSTURE_CAPABILITY=true", wrapper)
        self.assertIn("export BPX_REQUIRED_POSTURE_CAPABILITY=true", wrapper)
        self.assertNotIn("BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE", wrapper)

    def test_external_sdk_wheel_is_verified_and_mounted_read_only(self) -> None:
        start_script = (PACKAGE_ROOT / "scripts/start.sh").read_text(encoding="utf-8")
        entrypoint = (PACKAGE_ROOT / "docker/entrypoint.sh").read_text(encoding="utf-8")
        self.assertIn('SDK_WHEEL="${BPX_SDK_WHEEL:-}"', start_script)
        self.assertIn("sdk/SHA256SUMS", start_script)
        self.assertIn("sha256sum", start_script)
        self.assertIn(":/vendor-sdk/$wheel_name:ro", start_script)
        self.assertIn('BPX_SDK_WHEEL="/vendor-sdk/$wheel_name"', start_script)
        self.assertIn('python3 -m pip install --no-index --no-deps --force-reinstall "$BPX_SDK_WHEEL"', entrypoint)

    def test_sdk_runtime_has_explicit_docker_desktop_udp_mode(self) -> None:
        start_script = (PACKAGE_ROOT / "scripts/start.sh").read_text(encoding="utf-8")
        probe_script = (PACKAGE_ROOT / "scripts/run-read-only-probe.sh").read_text(
            encoding="utf-8"
        )
        record_script = (PACKAGE_ROOT / "scripts/record-sdk-replay.sh").read_text(
            encoding="utf-8"
        )
        for script in (start_script, probe_script, record_script):
            self.assertIn("--network-mode", script)
            self.assertIn("--publish", script)
        self.assertIn("BPX_DOCKER_STATE_UDP_PORT", start_script)
        self.assertIn("BPX_DOCKER_HOST_ADDRESS", start_script)
        self.assertIn("host.docker.internal:host-gateway", start_script)
        self.assertIn("-e BPX_DOCKER_NETWORK_MODE", start_script)
        entrypoint = (PACKAGE_ROOT / "docker/entrypoint.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('BPX_DOCKER_NETWORK_MODE:-host', entrypoint)
        self.assertIn('ROBONIX_ADVERTISE_HOST="$(hostname -i', entrypoint)

    def test_posture_probe_is_explicit_and_excludes_velocity_and_joint_control(self) -> None:
        wrapper = (PACKAGE_ROOT / "scripts/run-posture-cycle-probe.sh").read_text(
            encoding="utf-8"
        )
        adapter = (PACKAGE_ROOT / "bpx_quadruped/sdk_posture.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("--confirm-physical-motion", wrapper)
        self.assertIn("MotionLevelControl", adapter)
        self.assertIn("setStandUp", adapter)
        self.assertIn("setSitDown", adapter)
        self.assertIn("supports_twist", adapter)
        self.assertNotIn(".setVelocity(", adapter)
        self.assertNotIn(".setZeroPositionsFlag(", adapter)
        self.assertNotIn("JointLevelControl()", adapter)

    def test_yaw_circle_probe_is_manual_only_and_excludes_joint_control(self) -> None:
        wrapper = (PACKAGE_ROOT / "scripts/run-yaw-circle-probe.sh").read_text(
            encoding="utf-8"
        )
        adapter = (PACKAGE_ROOT / "bpx_quadruped/sdk_yaw_circle.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("--confirm-physical-yaw-circle", wrapper)
        self.assertIn("--yaw-circle", wrapper)
        self.assertIn("setVelocity", adapter)
        self.assertIn("0.0, 0.0, float(yaw_rate_rps)", adapter)
        self.assertNotIn("JointLevelControl()", adapter)
        self.assertNotIn("setZeroPositionsFlag", adapter)
        for path in PACKAGE_ROOT.glob("package_manifest*.yaml"):
            manifest = path.read_text(encoding="utf-8")
            self.assertNotIn("run-yaw-circle-probe.sh", manifest)

    def test_sdk_read_only_manifest_does_not_declare_a_command_capability(self) -> None:
        manifest = (PACKAGE_ROOT / "package_manifest.sdk-read-only.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("robonix/primitive/quadruped/odom", manifest)
        self.assertNotIn("robonix/primitive/quadruped/twist_in", manifest)
        self.assertIn("bash scripts/start-sdk-read-only.sh", manifest)

        wrapper = (PACKAGE_ROOT / "scripts/start-sdk-read-only.sh").read_text(
            encoding="utf-8"
        )
        self.assertTrue(os.access(PACKAGE_ROOT / "scripts/start-sdk-read-only.sh", os.X_OK))
        self.assertIn('start.sh" "$@"', wrapper)
        self.assertIn('--sdk-wheel is required for the SDK backend',
                      (PACKAGE_ROOT / "scripts/start.sh").read_text())
        self.assertIn("export BPX_REQUIRED_BACKEND=sdk", wrapper)

        start_script = (PACKAGE_ROOT / "scripts/start.sh").read_text(encoding="utf-8")
        self.assertIn("-e BPX_REQUIRED_BACKEND", start_script)

    def test_provider_shutdown_stops_ros_before_native_extension_unload(self) -> None:
        provider = (PACKAGE_ROOT / "bpx_quadruped/main.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("_runtime.shutdown()", provider)
        self.assertIn("RosBackend.get().shutdown()", provider)

        stop_script = (PACKAGE_ROOT / "scripts/stop.sh").read_text(encoding="utf-8")
        self.assertIn('STOP_TIMEOUT_S=30', stop_script)
        self.assertIn('--stop-timeout-s', stop_script)

    def test_replay_manifest_is_read_only_and_forces_replay_backend(self) -> None:
        manifest = (PACKAGE_ROOT / "package_manifest.state-replay.yaml").read_text(
            encoding="utf-8"
        )
        self.assertIn("robonix/primitive/quadruped/odom", manifest)
        self.assertNotIn("robonix/primitive/quadruped/twist_in", manifest)
        self.assertIn("bash scripts/start-state-replay.sh", manifest)

        wrapper = (PACKAGE_ROOT / "scripts/start-state-replay.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("export BPX_REQUIRED_BACKEND=replay", wrapper)


if __name__ == "__main__":
    unittest.main()
