"""Read-only staged origin capture and offline, observation-only comparisons."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
from typing import Any, Dict

from . import read_only_probe

SCHEMA = "mirrorme-bpx-odom-acceptance-v1"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def finite(value: Any) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("expected a finite number")
    if not math.isfinite(value):
        raise ValueError("expected a finite number")
    return float(value)


def vector(data: Dict[str, Any], key: str, size: int) -> list:
    values = data[key]
    if not isinstance(values, list) or len(values) != size:
        raise ValueError("invalid " + key)
    return [finite(value) for value in values]


def decode(sample: Dict[str, Any]) -> Dict[str, Any]:
    odom = sample["leg_odometry"]
    position = vector(odom, "position", 3)
    q = vector(odom, "orientation", 4)
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion norm")
    x, y, z, w = [value / norm for value in q]
    return {
        "position": position,
        "yaw": math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
        "velocity_body": vector(odom, "velocity_body", 3),
        "angular_velocity": vector(odom, "angular_velocity", 3),
        "timestamp": finite(sample["odometry_timestamp"]),
        "time": finite(sample["host_monotonic_ns"]) / 1e9,
    }


class CaptureWriter:
    """Flush each original event before rendering; never rebase odometry."""

    def __init__(self, stream: Any):
        self.stream = stream
        self.last_timestamp = None
        self.changed_at = None

    def emit(self, event: Dict[str, Any]) -> None:
        self.stream.write(json.dumps(event, sort_keys=True, allow_nan=False) + "\n")
        self.stream.flush()
        if "event" in event:
            print(json.dumps(event, sort_keys=True), flush=True)
            return
        try:
            data = decode(event)
            if data["timestamp"] != self.last_timestamp:
                self.last_timestamp = data["timestamp"]
                self.changed_at = data["time"]
            age = data["time"] - self.changed_at
            print(
                "connected={} xyz={} yaw={:+.4f} rad v_body={} omega_body={} "
                "sdk_ts={} unchanged={:.2f}s".format(
                    event.get("connected"),
                    [round(v, 4) for v in data["position"]], data["yaw"],
                    [round(v, 4) for v in data["velocity_body"]],
                    [round(v, 4) for v in data["angular_velocity"]],
                    data["timestamp"], age,
                ), flush=True,
            )
        except (KeyError, TypeError, ValueError):
            print("connected={} odom=missing/invalid".format(event.get("connected")), flush=True)


def capture(args: argparse.Namespace) -> int:
    read_only_probe.validate_args(args)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.stage):
        raise ValueError("stage must be 1..64 letters, digits, underscores or hyphens")
    wheel = Path(args.wheel)
    digest = hashlib.sha256()
    with wheel.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    output = Path(args.output_dir) / (args.stage + ".jsonl")
    # Open exclusively before touching hardware. Parent must already exist.
    with output.open("x", encoding="utf-8") as stream:
        writer = CaptureWriter(stream)
        writer.emit({
            "event": "capture_start", "schema": SCHEMA, "stage": args.stage,
            "host_wall_time": utc_now(), "wheel_name": wheel.name,
            "wheel_sha256": digest.hexdigest(), "operator_note": args.note,
            "options": {key: getattr(args, key) for key in (
                "robot_ip", "robot_state_port", "tcp_local_port", "state_rate_hz",
                "connect_timeout_s", "duration_s", "sample_period_s",
            )},
        })
        stop = threading.Event()
        previous = {}

        def interrupted(_signum: int, _frame: Any) -> None:
            stop.set()

        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                previous[sig] = signal.signal(sig, interrupted)
            result = read_only_probe.run(args, stop_event=stop, emit=writer.emit)
        except Exception as exc:
            writer.emit({"event": "capture_end", "status": "error",
                         "host_wall_time": utc_now(), "error": str(exc)})
            raise
        else:
            writer.emit({"event": "capture_end", "host_wall_time": utc_now(),
                         "status": "interrupted" if stop.is_set() else "complete"})
            return result
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def load_capture(path: str) -> tuple:
    records = []
    warnings = []
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    for index, line in enumerate(lines):
        try:
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValueError("expected an object")
            records.append(record)
        except ValueError:
            # A killed process may leave an incomplete final write.
            if index != len(lines) - 1:
                raise ValueError("invalid JSON at line {}".format(index + 1))
            warnings.append("truncated_or_invalid_final_line")
    if (not records or records[0].get("schema") != SCHEMA
            or records[0].get("event") != "capture_start"
            or not isinstance(records[0].get("options"), dict)
            or not records[0]["options"].get("robot_ip")
            or not records[0].get("wheel_sha256")):
        raise ValueError("not an odom acceptance capture: " + path)
    if records[-1].get("event") != "capture_end" or records[-1].get("status") != "complete":
        warnings.append("capture_incomplete")
    return records, warnings


def summarize(path: str, edge: str, args: argparse.Namespace) -> Dict[str, Any]:
    records, warnings = load_capture(path)
    samples = [r for r in records if "event" not in r]
    valid = []
    invalid = duplicates = disconnected = regressions = 0
    previous_timestamp = previous_time = None
    for sample in samples:
        try:
            data = decode(sample)
            if sample.get("connected") is not True:
                disconnected += 1
                continue
            if previous_time is not None and data["time"] <= previous_time:
                raise ValueError("non-increasing host sample time")
            previous_time = data["time"]
            if data["timestamp"] == previous_timestamp:
                duplicates += 1
                continue
            if previous_timestamp is not None:
                regressions += int(data["timestamp"] < previous_timestamp)
                valid.append(data)
            previous_timestamp = data["timestamp"]
        except (KeyError, ValueError, TypeError):
            invalid += 1
    result = {"file": path, "metadata": records[0], "warnings": warnings,
              "robot_version": next((r.get("robot_version") for r in records
                                     if r.get("event") == "probe_ready"), None),
              "sdk_version": next((r.get("sdk_version") for r in records
                                   if r.get("event") == "probe_start"), None),
              "samples": len(samples), "fresh_samples": len(valid),
              "invalid_samples": invalid, "duplicate_timestamps": duplicates,
              "disconnected_samples": disconnected, "timestamp_regressions": regressions}
    if invalid or disconnected or regressions:
        warnings.append("invalid_disconnected_or_timestamp_regression")
    if not valid:
        warnings.append("no_fresh_odometry")
        return result
    if previous_time is not None and previous_time - valid[-1]["time"] > args.max_gap_s:
        warnings.append("stale_capture_tail")
    lower = valid[0]["time"] + args.settle_s if edge == "first" else valid[-1]["time"] - args.window_s
    upper = lower + args.window_s
    window = [s for s in valid if lower <= s["time"] <= upper]
    if len(window) < 3 or window[-1]["time"] - window[0]["time"] < args.window_s * 0.8:
        warnings.append("insufficient_window_coverage")
        return result
    if any(b["time"] - a["time"] > args.max_gap_s for a, b in zip(window, window[1:])):
        warnings.append("stale_or_missing_window_data")
    position = [sum(s["position"][i] for s in window) / len(window) for i in range(3)]
    yaw = math.atan2(sum(math.sin(s["yaw"]) for s in window),
                     sum(math.cos(s["yaw"]) for s in window))
    xy_spread = max(math.hypot(s["position"][0] - position[0], s["position"][1] - position[1]) for s in window)
    yaw_spread = max(abs(wrap(s["yaw"] - yaw)) for s in window)
    if (xy_spread > args.position_tolerance_m or yaw_spread > args.yaw_tolerance_rad
            or any(math.hypot(*s["velocity_body"]) > args.stationary_speed_mps
                   or math.hypot(*s["angular_velocity"]) > args.stationary_rate_rps for s in window)):
        warnings.append("window_not_stationary")
    result["window"] = {"edge": edge, "samples": len(window), "position_m": position,
                        "yaw_rad": yaw, "xy_spread_m": xy_spread, "yaw_spread_rad": yaw_spread,
                        "sdk_timestamp_first": window[0]["timestamp"],
                        "sdk_timestamp_last": window[-1]["timestamp"]}
    return result


def classify(before_norm: float, after_norm: float, delta: float, tolerance: float) -> str:
    if before_norm <= tolerance and after_norm <= tolerance:
        return "near_zero_inconclusive"
    if delta <= tolerance:
        return "continuous_candidate"
    if after_norm <= tolerance:
        return "reset_to_zero_candidate"
    return "other_jump"


def compare(args: argparse.Namespace) -> Dict[str, Any]:
    if Path(args.before).samefile(args.after):
        raise ValueError("before and after must be distinct capture files")
    for name in ("window_s", "max_gap_s", "position_tolerance_m", "yaw_tolerance_rad",
                 "stationary_speed_mps", "stationary_rate_rps"):
        if finite(getattr(args, name)) <= 0:
            raise ValueError(name + " must be > 0")
    if finite(args.settle_s) < 0:
        raise ValueError("settle_s must be >= 0")
    before = summarize(args.before, "last", args)
    after = summarize(args.after, "first", args)
    warnings = before["warnings"] + after["warnings"]
    for key in ("wheel_sha256",):
        if before["metadata"].get(key) != after["metadata"].get(key):
            warnings.append("different_" + key)
    if before["metadata"]["options"]["robot_ip"] != after["metadata"]["options"]["robot_ip"]:
        warnings.append("different_robot_ip")
    for key in ("robot_version", "sdk_version"):
        if before[key] != after[key]:
            warnings.append("different_" + key)
    report = {"before": before, "after": after, "warnings": warnings,
              "thresholds": {key: getattr(args, key) for key in (
                  "window_s", "settle_s", "max_gap_s", "position_tolerance_m",
                  "yaw_tolerance_rad", "stationary_speed_mps", "stationary_rate_rps")},
              "assessment": "insufficient_evidence",
              "note": "Observation only; confirm same physical pose/posture and operation manually. "
                      "SDK timestamp units are not assumed. This is not a hardware acceptance verdict."}
    if "window" not in before or "window" not in after:
        return report
    a, b = before["window"], after["window"]
    delta = [right - left for left, right in zip(a["position_m"], b["position_m"])]
    yaw_delta = wrap(b["yaw_rad"] - a["yaw_rad"])
    report["delta"] = {"position_m": delta, "yaw_rad": yaw_delta,
                       "sdk_timestamp_decreased": b["sdk_timestamp_first"] < a["sdk_timestamp_last"]}
    if not warnings:
        report["assessment"] = "requires_operator_confirmation"
        report["xy_behavior"] = classify(math.hypot(*a["position_m"][:2]),
                                         math.hypot(*b["position_m"][:2]),
                                         math.hypot(*delta[:2]), args.position_tolerance_m)
        report["yaw_behavior"] = classify(abs(a["yaw_rad"]), abs(b["yaw_rad"]),
                                          abs(yaw_delta), args.yaw_tolerance_rad)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    capture_parser = sub.add_parser("capture", parents=[read_only_probe.build_parser()], add_help=False)
    capture_parser.set_defaults(robot_ip="192.168.1.237")
    capture_parser.add_argument("--wheel", required=True)
    capture_parser.add_argument("--output-dir", required=True)
    capture_parser.add_argument("--stage", required=True)
    capture_parser.add_argument("--note", default="")
    comparison = sub.add_parser("compare")
    comparison.add_argument("before")
    comparison.add_argument("after")
    comparison.add_argument("--output", help="Optional report path; refuses overwrite")
    comparison.add_argument("--window-s", type=float, default=3.0)
    comparison.add_argument("--settle-s", type=float, default=2.0)
    comparison.add_argument("--max-gap-s", type=float, default=0.5)
    comparison.add_argument("--position-tolerance-m", type=float, default=0.05)
    comparison.add_argument("--yaw-tolerance-rad", type=float, default=0.05)
    comparison.add_argument("--stationary-speed-mps", type=float, default=0.05)
    comparison.add_argument("--stationary-rate-rps", type=float, default=0.05)
    return parser


def main(argv: Any = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "capture":
            return capture(args)
        report = compare(args)
        rendered = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
        if args.output:
            with open(args.output, "x", encoding="utf-8") as stream:
                stream.write(rendered + "\n")
        print(rendered)
        return 0 if report["assessment"] != "insufficient_evidence" else 2
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print("odom acceptance failed: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
