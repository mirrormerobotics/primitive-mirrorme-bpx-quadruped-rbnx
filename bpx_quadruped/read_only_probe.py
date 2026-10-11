"""Bounded BPX state capture using RequestRobotState only.

This module intentionally does not import or construct MotionLevelControl or
JointLevelControl. It never sends posture, velocity, zero-position or joint
commands.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import signal
import sys
import threading
import time
from typing import Any, Callable, Dict, Iterable, Optional


STATE_FIELDS = (
    ("joint_position", "getJointPosition"),
    ("joint_velocity", "getJointVelocity"),
    ("joint_torque", "getJointTorque"),
    ("imu_rpy", "getImuRpy"),
    ("imu_quaternion", "getImuQuat"),
    ("imu_acceleration", "getImuAcc"),
    ("imu_angular_velocity", "getImuOmega"),
    ("leg_odometry", "getLegOdom"),
    ("motor_temperature", "getMotorTemperature"),
    ("driver_temperature", "getDriverTemperature"),
    ("motion_state", "getCurrentMotionState"),
    ("motion_gait", "getCurrentGait"),
    ("sub_gait", "getSubGait"),
    ("max_velocity", "getMaxVelocity"),
    ("battery_level", "getBatteryLevel"),
    ("battery_current", "getBatteryCurrent"),
    ("joint_timestamp", "getJointStateTimestamp"),
    ("imu_timestamp", "getImuTimestamp"),
    ("odometry_timestamp", "getOdometryTimestamp"),
    ("motion_state_timestamp", "getMotionStateTimestamp"),
    ("battery_timestamp", "getBatteryTimestamp"),
)


def collect_sample(robot_state: Any) -> Dict[str, Any]:
    sample: Dict[str, Any] = {
        "host_wall_time": datetime.now(timezone.utc).isoformat(),
        "host_monotonic_ns": time.monotonic_ns(),
        "connected": bool(robot_state.isConnected()),
    }
    errors: Dict[str, str] = {}
    for output_name, method_name in STATE_FIELDS:
        method = getattr(robot_state, method_name, None)
        if method is None:
            errors[output_name] = "method unavailable"
            continue
        try:
            sample[output_name] = _json_safe(method())
        except Exception as exc:  # SDK calls must not terminate cleanup
            errors[output_name] = "{}: {}".format(type(exc).__name__, exc)
    if errors:
        sample["errors"] = errors
    return sample


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture BPX state with RequestRobotState; sends no motion commands."
    )
    parser.add_argument("--robot-ip", default="10.21.20.1")
    parser.add_argument("--robot-state-port", type=int, default=9873)
    parser.add_argument("--tcp-local-port", type=int, default=0)
    parser.add_argument("--state-rate-hz", type=int, default=50)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--sample-period-s", type=float, default=0.1)
    return parser


def validate_args(args: argparse.Namespace) -> None:
    if not 1024 <= args.robot_state_port <= 65535:
        raise ValueError("robot-state-port must be in 1024..65535")
    if not 0 <= args.tcp_local_port <= 65535:
        raise ValueError("tcp-local-port must be in 0..65535")
    if not 1 <= args.state_rate_hz <= 200:
        raise ValueError("state-rate-hz must be in 1..200")
    for name in ("connect_timeout_s", "duration_s", "sample_period_s"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("{} must be finite and > 0".format(name.replace("_", "-")))


def print_event(event: Dict[str, Any]) -> None:
    print(json.dumps(event, sort_keys=True), flush=True)


def run(
    args: argparse.Namespace,
    *,
    stop_event: Optional[threading.Event] = None,
    emit: Callable[[Dict[str, Any]], None] = print_event,
) -> int:
    validate_args(args)
    import bpx_sdk

    if stop_event is None:
        stop_event = threading.Event()

    robot_state = bpx_sdk.RequestRobotState()
    robot_state.setRobotIp(args.robot_ip)
    robot_state.setRobotStateUploadPort(args.robot_state_port)
    robot_state.setTcpLocalPort(args.tcp_local_port)
    robot_state.setRobotStateUploadRate(args.state_rate_hz)

    emit(
        {
            "event": "probe_start",
            "sdk_version": getattr(bpx_sdk, "__version__", "unknown"),
            "robot_ip": args.robot_ip,
            "robot_state_port": args.robot_state_port,
            "state_rate_hz": args.state_rate_hz,
        }
    )

    connect_started = time.monotonic()
    try:
        if not robot_state.connect():
            raise RuntimeError("RequestRobotState.connect() returned false")
        connect_deadline = connect_started + args.connect_timeout_s
        while not stop_event.is_set():
            connected = robot_state.isConnected()
            odometry = robot_state.getLegOdom() if connected else None
            odometry_timestamp = (
                robot_state.getOdometryTimestamp() if connected else None
            )
            if connected and odometry is not None and odometry_timestamp is not None:
                break
            if time.monotonic() >= connect_deadline:
                raise TimeoutError(
                    "BPX state connection did not produce an odometry frame"
                )
            time.sleep(0.05)

        if stop_event.is_set():
            return 0

        get_robot_version = getattr(robot_state, "getRobotVersion", None)
        robot_version = get_robot_version() if get_robot_version is not None else None
        emit(
            {
                "event": "probe_ready",
                "first_odometry_latency_s": time.monotonic() - connect_started,
                "robot_version": _json_safe(robot_version),
            }
        )

        end_at = time.monotonic() + args.duration_s
        while not stop_event.is_set() and time.monotonic() < end_at:
            emit(collect_sample(robot_state))
            stop_event.wait(args.sample_period_s)
    finally:
        robot_state.disconnect()
        emit({"event": "probe_stop"})
    return 0


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    stop_event = threading.Event()
    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        stop_event.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    try:
        return run(args, stop_event=stop_event)
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print("read-only probe failed: {}".format(exc), file=sys.stderr)
        return 2
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
