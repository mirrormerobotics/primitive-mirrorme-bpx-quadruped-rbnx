from __future__ import annotations

import hashlib
import json
import os
import shlex
from pathlib import Path
import subprocess
import tempfile
import unittest
import venv
import zipfile

ROOT = Path(__file__).resolve().parents[1]


class CommandLineTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="bpx cli test ")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.log = self.work / "commands.jsonl"
        self.bin = self.work / "bin"
        self.bin.mkdir()
        # Only these fake executables are reachable as Docker/rbnx in tests.
        docker = self.bin / "docker"
        docker.write_text("""#!/usr/bin/env python3
import json, os, sys
with open(os.environ["CLI_TEST_LOG"], "a") as stream:
    stream.write(json.dumps({"argv": sys.argv[1:], "env": {
        key: os.environ.get(key) for key in ("BPX_REQUIRED_BACKEND", "BPX_POSTURE_CAPABILITY")
    }}) + "\\n")
if len(sys.argv) > 1 and sys.argv[1] == "inspect":
    if os.environ.get("CLI_TEST_EXISTING") == "1":
        if "--format" in sys.argv:
            print(os.environ.get("CLI_TEST_LABEL", "robonix.primitive.mirrorme.bpx.quadruped"))
        sys.exit(0)
    sys.exit(1)
""")
        docker.chmod(0o755)
        rbnx = self.bin / "rbnx"
        rbnx.write_text("#!/bin/bash\nprintf '%s\\n' /tmp/fake-robonix-api\n")
        rbnx.chmod(0o755)
        self.env = {key: value for key, value in os.environ.items()
                    if not key.startswith(("BPX_", "RBNX_", "ROBONIX_", "CLI_"))}
        self.env.update(PATH=str(self.bin) + os.pathsep + os.environ["PATH"],
                        CLI_TEST_LOG=str(self.log))

    def run_script(self, name, *args, env=None):
        return subprocess.run(["bash", str(ROOT / "scripts" / name), *map(str, args)],
                              env={**self.env, **(env or {})}, cwd=self.work,
                              text=True, capture_output=True, timeout=10)

    def calls(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def fake_robonix_source(self):
        source = self.work / "robonix source"
        (source / ".git").mkdir(parents=True)
        expected = "a" * 40
        git = self.bin / "git"
        git.write_text("""#!/usr/bin/env python3
import sys
if sys.argv[-2:] == ["rev-parse", "HEAD"]:
    print("a" * 40)
elif "submodule" not in sys.argv:
    sys.exit(1)
""")
        git.chmod(0o755)
        return source, expected

    def assert_cache_mounts(self, directories):
        runs = [c["argv"] for c in self.calls() if c["argv"][0] == "run"]
        self.assertEqual(len(runs), 5)
        for command in runs:
            mounts = [command[i + 1] for i, argument in enumerate(command) if argument == "-v"]
            for directory, target in zip(directories, ("/target", "/cargo", "/root/.robonix")):
                self.assertTrue(directory.is_dir())
                self.assertTrue(directory.is_absolute())
                self.assertIn(str(directory) + ":" + target, mounts)

    def test_verify_rbnx_resolves_relative_tool_cache_from_invocation_directory(self):
        source, expected = self.fake_robonix_source()
        result = self.run_script("verify-rbnx.sh", source, "--expected-commit", expected,
                                 "--tool-cache", "cache base with spaces")
        self.assertEqual(result.returncode, 0, result.stderr)
        base = self.work / "cache base with spaces"
        self.assert_cache_mounts([base / name for name in ("target", "cargo", "config")])

    def test_verify_rbnx_resolves_individual_relative_caches_and_existing_symlinks(self):
        source, expected = self.fake_robonix_source()
        target = self.work / "existing target directory"
        config = self.work / "existing config directory"
        target.mkdir()
        config.mkdir()
        (self.work / "target link").symlink_to(target, target_is_directory=True)
        (self.work / "config link").symlink_to(config, target_is_directory=True)
        result = self.run_script("verify-rbnx.sh", source, "--expected-commit", expected,
                                 "--tool-cache", "unused cache base",
                                 "--target-cache", "target link",
                                 "--cargo-cache", "new cargo directory",
                                 "--config-cache", "config link")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_cache_mounts([target, self.work / "new cargo directory", config])
        self.assertFalse((self.work / "unused cache base").exists())

    def fake_installable_wheel(self, directory, marker):
        directory.mkdir()
        wheel = directory / "bpx_cli_wheel_fixture-1.0.8-py3-none-any.whl"
        metadata = "bpx_cli_wheel_fixture-1.0.8.dist-info/"
        contents = {
            "bpx_cli_wheel_fixture/__init__.py": "marker = " + repr(marker) + "\n",
            metadata + "METADATA": "Metadata-Version: 2.1\nName: bpx-cli-wheel-fixture\nVersion: 1.0.8\n",
            metadata + "WHEEL": "Wheel-Version: 1.0\nRoot-Is-Purelib: true\nTag: py3-none-any\n",
        }
        contents[metadata + "RECORD"] = "".join(name + ",,\n" for name in contents) + metadata + "RECORD,,\n"
        with zipfile.ZipFile(wheel, "w") as archive:
            for name, content in contents.items():
                archive.writestr(name, content)
        return wheel

    def test_capture_install_command_replaces_different_contents_at_same_wheel_version(self):
        package, _ = self.fake_package()
        baseline = self.fake_installable_wheel(self.work / "baseline wheel", "baseline")
        selected = self.fake_installable_wheel(self.work / "selected wheel", "selected")
        digest = hashlib.sha256(selected.read_bytes()).hexdigest()
        (package / "sdk/SHA256SUMS").write_text(digest + "  " + selected.name + "\n")
        result = self.run_script("record-odom-stage.sh", selected, self.work, "install_check",
                                 "--package-root", package)
        self.assertEqual(result.returncode, 0, result.stderr)
        launch = [c["argv"] for c in self.calls() if c["argv"][0] == "run"][-1]
        install = launch[launch.index("-lc") + 1].split("&&", 1)[0]
        install = install.replace('"/sdk/$BPX_SDK_WHEEL_NAME"', shlex.quote(str(selected)))

        # Use the existing offline pip through a disposable venv, without
        # ensurepip downloads or any writes to the host Python installation.
        environment = self.work / "isolated install"
        venv.create(environment, with_pip=False, system_site_packages=True)
        python = environment / "bin/python"
        pip_env = {**self.env, "PATH": str(environment / "bin") + os.pathsep + self.env["PATH"],
                   "PYTHONDONTWRITEBYTECODE": "1", "PIP_CONFIG_FILE": os.devnull,
                   "PIP_NO_INDEX": "1", "PIP_DISABLE_PIP_VERSION_CHECK": "1",
                   "PIP_CACHE_DIR": str(self.work / "pip cache")}
        baseline_install = subprocess.run(
            [str(python), "-m", "pip", "install", "--no-index", "--no-deps", str(baseline)],
            env=pip_env, cwd=self.work, capture_output=True, text=True, timeout=30)
        self.assertEqual(baseline_install.returncode, 0, baseline_install.stderr)

        def installed_marker():
            probe = subprocess.run(
                [str(python), "-c", "import bpx_cli_wheel_fixture as package; print(package.marker); print(package.__file__)"],
                env=pip_env, cwd=self.work, capture_output=True, text=True, timeout=10)
            self.assertEqual(probe.returncode, 0, probe.stderr)
            marker, location = probe.stdout.strip().splitlines()
            self.assertIn(environment, Path(location).parents)
            return marker

        self.assertEqual(installed_marker(), "baseline")
        replacement = subprocess.run(["bash", "-c", install], env=pip_env, cwd=self.work,
                                     capture_output=True, text=True, timeout=30)
        self.assertEqual(replacement.returncode, 0, replacement.stderr)
        self.assertEqual(installed_marker(), "selected")

    def test_every_operator_script_has_side_effect_free_help(self):
        for script in sorted((ROOT / "scripts").glob("*.sh")):
            with self.subTest(script=script.name):
                result = self.run_script(script.name, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("Usage:", result.stdout)
        self.assertEqual(self.calls(), [])

    def test_shell_syntax(self):
        for script in (ROOT / "scripts").rglob("*.sh"):
            result = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_values_fail_before_docker(self):
        candidates = [("start-sdk-read-only-docker.sh", "--runtime-image"),
                      ("record-odom-stage.sh", "--image")]
        for script, option in candidates:
            if not (ROOT / "scripts" / script).exists():
                continue
            for suffix in ([], ["--robot-ip", "192.168.1.237"]):
                result = self.run_script(script, option, *suffix)
                self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.calls(), [])

    def test_unknown_option_fails_before_docker(self):
        script = "start.sh" if (ROOT / "scripts/start.sh").exists() else "verify-tf-smoke.sh"
        result = self.run_script(script, "--does-not-exist")
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.calls(), [])

    def fake_package(self):
        package = self.work / "package with spaces"
        package.mkdir()
        (package / "scripts").symlink_to(ROOT / "scripts", target_is_directory=True)
        (package / "sdk").mkdir()
        wheel = self.work / "sdk.whl"
        wheel.write_bytes(b"offline test wheel, never installed")
        digest = hashlib.sha256(wheel.read_bytes()).hexdigest()
        (package / "sdk/SHA256SUMS").write_text(digest + "  sdk.whl\n")
        return package, wheel

    def test_capture_passes_image_network_paths_and_python_options(self):
        package, wheel = self.fake_package()
        result = self.run_script(
            "record-odom-stage.sh", wheel, self.work, "before",
            "--package-root", package, "--image=custom:test", "--network-mode", "bridge",
            "--robot-state-port=9988", "--robot-ip", "192.168.1.237",
            "--note", "P1 H1; no motion",
            env={"BPX_RBNX_IMAGE": "ignored:image", "BPX_DOCKER_NETWORK_MODE": "invalid"})
        self.assertEqual(result.returncode, 0, result.stderr)
        runs = [call["argv"] for call in self.calls() if call["argv"][0] == "run"]
        self.assertEqual(len(runs), 2)
        self.assertIn("custom:test", runs[0])
        self.assertIn("none", runs[0])
        self.assertIn("custom:test", runs[1])
        self.assertIn("9988:9988/udp", runs[1])
        self.assertIn("P1 H1; no motion", runs[1])
        self.assertIn(str(self.work) + ":/capture", runs[1])
        self.assertNotIn("--image=custom:test", runs[1])
        self.assertNotIn("--network-mode", runs[1])

    def test_motion_probes_require_confirmation_before_docker(self):
        _, wheel = self.fake_package()
        for script in ("run-posture-cycle-probe.sh", "run-yaw-circle-probe.sh"):
            result = self.run_script(script, wheel, "--robot-ip", "192.168.1.237")
            self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(self.calls(), [])

    def test_capture_imports_mounted_module_when_image_workdir_contains_old_package(self):
        package, wheel = self.fake_package()
        result = self.run_script("record-odom-stage.sh", wheel, self.work, "boot_p0",
                                 "--package-root", package)
        self.assertEqual(result.returncode, 0, result.stderr)
        command = [c["argv"] for c in self.calls() if c["argv"][0] == "run"][-1]
        launch = command[command.index("-lc") + 1]
        # Exercise the emitted Python launch after the wheel install, with no SDK
        # or robot required. Map the container's /pkg mount to the real checkout.
        launch = launch.split("&&", 1)[1].replace("/pkg", shlex.quote(str(ROOT)))
        stale_workdir = self.work / "image-workdir"
        (stale_workdir / "bpx_quadruped").mkdir(parents=True)
        (stale_workdir / "bpx_quadruped/__init__.py").write_text("# Old image package\n")
        probe = subprocess.run(["bash", "-c", launch, "capture", "--help"],
                               cwd=stale_workdir,
                               env={**self.env, "PYTHONPATH": str(ROOT),
                                    "BPX_SDK_WHEEL_NAME": wheel.name,
                                    "PYTHONDONTWRITEBYTECODE": "1"},
                               text=True, capture_output=True, timeout=10)
        self.assertEqual(probe.returncode, 0, probe.stderr)
        self.assertIn("--stage", probe.stdout)

    def test_local_sdk_profile_requires_explicit_choice_and_remains_read_only(self):
        package, wheel = self.fake_package()
        local_pin = (package / "sdk/SHA256SUMS").read_text()
        (package / "sdk/SHA256SUMS.local-v1.0.8").write_text(local_pin)
        (package / "sdk/SHA256SUMS").write_text("0" * 64 + "  sdk.whl\n")
        result = self.run_script("verify-sdk-wheel.sh", wheel, "--package-root", package)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("checksum mismatch", result.stderr)
        self.assertEqual(self.calls(), [])
        for mode in ("--posture", "--yaw-circle"):
            result = self.run_script("verify-sdk-wheel.sh", wheel, "--package-root", package,
                                     "--sdk-profile", "local-v1.0.8", mode)
            self.assertEqual(result.returncode, 2)
            self.assertIn("restricted to read-only", result.stderr)
        self.assertEqual(self.calls(), [])
        result = self.run_script("record-odom-stage.sh", wheel, self.work, "local_before",
                                 "--package-root", package, "--sdk-profile", "local-v1.0.8")
        self.assertEqual(result.returncode, 0, result.stderr)
        runs = [c["argv"] for c in self.calls() if c["argv"][0] == "run"]
        self.assertEqual(len(runs), 2)
        self.assertIn("not the release baseline", result.stderr)
        self.assertNotIn("--sdk-profile", runs[-1])

    def test_unknown_sdk_profile_is_rejected(self):
        package, wheel = self.fake_package()
        result = self.run_script("verify-sdk-wheel.sh", wheel, "--package-root", package,
                                 "--sdk-profile", "anything")
        self.assertEqual(result.returncode, 2)
        self.assertEqual(self.calls(), [])

    def test_local_profile_does_not_admit_arbitrary_wheel(self):
        package, wheel = self.fake_package()
        (package / "sdk/SHA256SUMS.local-v1.0.8").write_text("0" * 64 + "  sdk.whl\n")
        result = self.run_script("verify-sdk-wheel.sh", wheel, "--package-root", package,
                                 "--sdk-profile", "local-v1.0.8")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("checksum mismatch", result.stderr)
        self.assertEqual(self.calls(), [])

    def test_posture_test_start_gate_and_cli_wheel(self):
        package, wheel = self.fake_package()
        result = self.run_script("start-posture-test.sh", "--sdk-wheel", wheel)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("--confirm-physical-posture-service", result.stderr)
        self.assertEqual(self.calls(), [])
        result = self.run_script(
            "start-posture-test.sh", "--sdk-wheel", wheel, "--package-root", package,
            "--confirm-physical-posture-service", "--container-name", "custom-posture",
            "--image", "custom:test", "--network-mode", "bridge",
            "--atlas-address", "127.0.0.1:50051", "--docker-host-address", "172.17.0.1",
            "--robot-state-port", "9988")
        self.assertEqual(result.returncode, 0, result.stderr)
        runs = [call for call in self.calls() if call["argv"][0] == "run"]
        self.assertEqual(len(runs), 1)
        self.assertIn("custom-posture", runs[0]["argv"])
        self.assertIn("custom:test", runs[0]["argv"])
        self.assertEqual(runs[0]["env"]["BPX_POSTURE_CAPABILITY"], "true")
        inspections = [c["argv"][-1] for c in self.calls() if c["argv"][0] == "inspect"]
        self.assertTrue(all(name == "custom-posture" for name in inspections))

    def test_managed_read_only_defaults_survive_and_cli_wins(self):
        package, wheel = self.fake_package()
        result = self.run_script(
            "start-sdk-read-only.sh", "--image", "cli:image", "--container-name", "cli-container",
            "--package-root", package,
            env={"BPX_SDK_WHEEL": str(wheel), "BPX_RBNX_IMAGE": "internal:image",
                 "BPX_RBNX_CONTAINER": "internal-container", "RBNX_DEPLOY_MANAGED": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        run = next(c for c in self.calls() if c["argv"][0] == "run")
        self.assertIn("cli:image", run["argv"])
        self.assertIn("cli-container", run["argv"])
        self.assertEqual(run["env"]["BPX_REQUIRED_BACKEND"], "sdk")
        self.assertEqual(run["env"]["BPX_POSTURE_CAPABILITY"], "false")

    def test_stop_targets_cli_container_and_respects_label(self):
        result = self.run_script("stop.sh", "--container-name", "my-container",
                                 "--stop-timeout-s", "7", env={"CLI_TEST_EXISTING": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(["stop", "--time", "7", "my-container"], [c["argv"] for c in self.calls()])
        self.log.unlink()
        result = self.run_script("stop.sh", "--container-name", "someone-elses",
                                 env={"CLI_TEST_EXISTING": "1", "CLI_TEST_LABEL": "unrelated"})
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(any(c["argv"][0] in ("stop", "rm") for c in self.calls()))

    def test_build_passes_clean_and_images_to_child(self):
        result = self.run_script("build.sh", "--clean", "--image", "runtime:test",
                                 "--test-image", "tests:test", "--use-tuna-mirror", "0")
        self.assertEqual(result.returncode, 0, result.stderr)
        builds = [c["argv"] for c in self.calls() if c["argv"][0] == "build"]
        self.assertEqual(len(builds), 2)
        self.assertIn("tests:test", builds[0])
        self.assertIn("runtime:test", builds[1])
        for command in builds:
            self.assertIn("--no-cache", command)
            self.assertIn("USE_TUNA_MIRROR=0", command)
