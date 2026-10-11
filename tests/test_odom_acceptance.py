from __future__ import annotations

import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bpx_quadruped import odom_acceptance as odom
from bpx_quadruped import read_only_probe


def sample(index, x=1.0, yaw=1.0, timestamp_base=1000, host_base=0):
    return {"host_monotonic_ns": host_base + index * 100_000_000,
            "connected": True, "odometry_timestamp": timestamp_base + index,
            "leg_odometry": {"position": [x, 0.0, 0.0],
                             "orientation": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
                             "velocity_body": [0.0] * 3, "angular_velocity": [0.0] * 3}}


class AcceptanceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def recording(self, name, *, x=1.0, yaw=1.0, timestamp_base=1000, host_base=0,
                  transform=lambda s: s, status="complete"):
        events = [{"event": "capture_start", "schema": odom.SCHEMA, "stage": name,
                   "wheel_sha256": "test", "options": {"robot_ip": "192.168.1.237"}},
                  {"event": "probe_start", "sdk_version": "test"},
                  {"event": "probe_ready", "robot_version": [1, 0, 15]}]
        events += [transform(sample(i, x, yaw, timestamp_base, host_base)) for i in range(101)]
        events += [{"event": "capture_end", "status": status}]
        path = self.root / (name + ".jsonl")
        path.write_text("".join(json.dumps(e) + "\n" for e in events))
        return str(path)

    def compare(self, before, after):
        args = odom.build_parser().parse_args(["compare", before, after])
        return odom.compare(args)

    def test_continuous_and_different_host_clock(self):
        result = self.compare(self.recording("before", host_base=9_000_000_000_000),
                              self.recording("after", timestamp_base=2000))
        self.assertEqual(result["xy_behavior"], "continuous_candidate")
        self.assertEqual(result["yaw_behavior"], "continuous_candidate")
        self.assertFalse(result["delta"]["sdk_timestamp_decreased"])

    def test_reset_and_timestamp_regression_are_separate(self):
        result = self.compare(self.recording("before"),
                              self.recording("after", x=0, yaw=0, timestamp_base=0))
        self.assertEqual(result["xy_behavior"], "reset_to_zero_candidate")
        self.assertEqual(result["yaw_behavior"], "reset_to_zero_candidate")
        self.assertTrue(result["delta"]["sdk_timestamp_decreased"])
        result = self.compare(self.recording("kept1"), self.recording("kept2", timestamp_base=0))
        self.assertEqual(result["xy_behavior"], "continuous_candidate")

    def test_other_jump(self):
        result = self.compare(self.recording("a"), self.recording("b", x=3, yaw=2))
        self.assertEqual(result["xy_behavior"], "other_jump")

    def test_near_zero_is_inconclusive(self):
        result = self.compare(self.recording("a", x=0, yaw=0), self.recording("b", x=0, yaw=0))
        self.assertEqual(result["xy_behavior"], "near_zero_inconclusive")

    def test_yaw_wrap(self):
        result = self.compare(self.recording("a", yaw=math.pi - 0.01),
                              self.recording("b", yaw=-math.pi + 0.01))
        self.assertAlmostEqual(result["delta"]["yaw_rad"], 0.02)
        self.assertEqual(result["yaw_behavior"], "continuous_candidate")

    def test_stale_tail_is_not_a_stationary_window(self):
        def frozen(s):
            if s["odometry_timestamp"] > 1040:
                s["odometry_timestamp"] = 1040
            return s
        result = self.compare(self.recording("a", transform=frozen), self.recording("b"))
        self.assertIn("stale_capture_tail", result["warnings"])
        self.assertEqual(result["assessment"], "insufficient_evidence")

    def test_missing_invalid_disconnected_and_frozen_data(self):
        for transform in (lambda s: {**s, "leg_odometry": None},
                          lambda s: {**s, "connected": False},
                          lambda s: {**s, "odometry_timestamp": 123},
                          lambda s: {**s, "odometry_timestamp": float("nan")}):
            result = self.compare(self.recording("a", transform=transform), self.recording("b"))
            self.assertEqual(result["assessment"], "insufficient_evidence")

    def test_moving_window(self):
        def moving(s):
            s["leg_odometry"]["velocity_body"][0] = 0.2
            return s
        result = self.compare(self.recording("a", transform=moving), self.recording("b"))
        self.assertIn("window_not_stationary", result["warnings"])

    def test_partial_capture_preserves_readable_samples(self):
        path = self.recording("a", status="interrupted")
        with open(path, "a") as stream:
            stream.write('{"partial":')
        result = self.compare(path, self.recording("b"))
        self.assertEqual(result["before"]["samples"], 101)
        self.assertEqual(result["assessment"], "insufficient_evidence")

    def test_timestamp_regression_inside_capture_is_flagged(self):
        def rewind(s):
            if s["odometry_timestamp"] > 1050:
                s["odometry_timestamp"] -= 1000
            return s
        result = self.compare(self.recording("a", transform=rewind), self.recording("b"))
        self.assertEqual(result["before"]["timestamp_regressions"], 1)
        self.assertEqual(result["assessment"], "insufficient_evidence")

    def test_fresh_window_gaps_are_flagged(self):
        def gap(s):
            if 1030 < s["odometry_timestamp"] < 1040:
                s["odometry_timestamp"] = 1030
            return s
        result = self.compare(self.recording("a"), self.recording("b", transform=gap))
        self.assertIn("stale_or_missing_window_data", result["warnings"])

    def test_same_capture_cannot_prove_reconnect(self):
        path = self.recording("a")
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.compare(path, path)

    def test_report_cli_refuses_overwrite_and_writes_insufficient_result(self):
        before = self.recording("a", status="interrupted")
        after = self.recording("b")
        output = str(self.root / "report.json")
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(odom.main(["compare", before, after, "--output", output]), 2)
            original = Path(output).read_text()
            self.assertEqual(odom.main(["compare", before, after, "--output", output]), 2)
            self.assertEqual(Path(output).read_text(), original)
        self.assertEqual(json.loads(original)["assessment"], "insufficient_evidence")

    def capture_args(self):
        wheel = self.root / "sdk.whl"
        wheel.write_bytes(b"fake-wheel")
        return odom.build_parser().parse_args([
            "capture", "--wheel", str(wheel), "--output-dir", str(self.root), "--stage", "capture"])

    def test_capture_flushes_and_retains_interruption(self):
        args = self.capture_args()

        def probe(_args, *, emit, stop_event):
            emit(sample(1))
            self.assertIn("leg_odometry", (self.root / "capture.jsonl").read_text())
            stop_event.set()
            return 0

        with patch.object(read_only_probe, "run", side_effect=probe), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(odom.capture(args), 0)
        records, warnings = odom.load_capture(str(self.root / "capture.jsonl"))
        self.assertEqual(records[-1]["status"], "interrupted")
        self.assertIn("capture_incomplete", warnings)
        self.assertEqual(len(records[0]["wheel_sha256"]), 64)

    def test_no_overwrite_before_sdk_access(self):
        args = self.capture_args()
        (self.root / "capture.jsonl").write_text("keep")
        with patch.object(read_only_probe, "run") as probe:
            with self.assertRaises(FileExistsError):
                odom.capture(args)
            probe.assert_not_called()

    def test_failure_preserves_samples(self):
        args = self.capture_args()

        def probe(_args, *, emit, stop_event):
            emit(sample(1))
            raise RuntimeError("simulated disconnect")

        with patch.object(read_only_probe, "run", side_effect=probe), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                odom.capture(args)
        records, _ = odom.load_capture(str(self.root / "capture.jsonl"))
        self.assertEqual(records[-1]["status"], "error")
        self.assertIn("leg_odometry", records[1])

    def test_probe_constructs_only_read_only_sdk_and_disconnects_on_error(self):
        class ReadOnly:
            disconnected = False

            def setRobotIp(self, value): pass
            def setRobotStateUploadPort(self, value): pass
            def setTcpLocalPort(self, value): pass
            def setRobotStateUploadRate(self, value): pass
            def connect(self): return True
            def isConnected(self): return True
            def getLegOdom(self): return sample(1)["leg_odometry"]
            def getOdometryTimestamp(self): return 1
            def disconnect(self): self.disconnected = True

        robot = ReadOnly()
        sdk = SimpleNamespace(RequestRobotState=lambda: robot)

        def fail_on_sample(event):
            if "event" not in event:
                raise RuntimeError("disk error")

        with patch.dict(sys.modules, {"bpx_sdk": sdk}):
            with self.assertRaisesRegex(RuntimeError, "disk error"):
                read_only_probe.run(self.capture_args(), emit=fail_on_sample)
        self.assertTrue(robot.disconnected)


if __name__ == "__main__":
    unittest.main()
