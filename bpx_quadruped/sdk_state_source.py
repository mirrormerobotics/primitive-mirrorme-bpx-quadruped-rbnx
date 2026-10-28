"""Read-only BPX SDK state adapter.

This module deliberately knows only ``RequestRobotState``. It has no command
method and never imports or constructs ``MotionLevelControl`` or
``JointLevelControl``. The posture runtime injects its single
``MotionLevelControl`` owner through this state-only seam; the explicit SDK
read-only runtime constructs ``RequestRobotState`` only.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import importlib
import ipaddress
import math
import threading
import time
from typing import Any, Callable, Optional, Sequence, Tuple

from .model import RobotState, VendorTimestamps
from .telemetry import normalize_robot_state


@dataclass(frozen=True)
class SdkStateConfig:
    robot_ip: str = "10.21.20.1"
    robot_state_port: int = 9873
    tcp_local_port: int = 0
    state_rate_hz: int = 50
    connect_timeout_s: float = 10.0
    poll_period_s: float = 0.05

    def validate(self) -> None:
        try:
            address = ipaddress.ip_address(self.robot_ip)
        except ValueError as exc:
            raise ValueError("robot_ip must be an IPv4 address") from exc
        if not isinstance(address, ipaddress.IPv4Address):
            raise ValueError("robot_ip must be an IPv4 address")
        _bounded_int(self.robot_state_port, "robot_state_port", 1024, 65535)
        _bounded_int(self.tcp_local_port, "tcp_local_port", 0, 65535)
        _bounded_int(self.state_rate_hz, "state_rate_hz", 1, 200)
        _positive_finite(self.connect_timeout_s, "connect_timeout_s")
        _positive_finite(self.poll_period_s, "poll_period_s")


class SdkStateSource:
    """Own exactly one ``RequestRobotState`` session.

    ``received_at_s`` advances only when the vendor odometry timestamp changes.
    Re-reading the SDK cache therefore cannot hide a stale robot-state stream
    from the controller watchdog.
    """

    def __init__(
        self,
        config: SdkStateConfig,
        request_factory: Optional[Callable[[], Any]] = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        config.validate()
        self._config = config
        self._request_factory = request_factory
        self._clock = clock
        self._sleep = sleep
        self._lock = threading.RLock()
        self._request: Optional[Any] = None
        self._last_state: Optional[RobotState] = None
        self._last_odometry_timestamp_ms: Optional[int] = None
        self._last_received_at_s: Optional[float] = None

    @property
    def name(self) -> str:
        return "bpx-sdk-state-read-only"

    def start(self) -> None:
        with self._lock:
            if self._request is not None:
                return
            request = self._new_request()
            request.setRobotIp(self._config.robot_ip)
            request.setRobotStateUploadPort(self._config.robot_state_port)
            request.setTcpLocalPort(self._config.tcp_local_port)
            request.setRobotStateUploadRate(self._config.state_rate_hz)
            try:
                if not request.connect():
                    raise RuntimeError("RequestRobotState.connect() returned false")
                deadline = self._clock() + self._config.connect_timeout_s
                while True:
                    if request.isConnected():
                        state = self._try_read(request)
                        if state is not None:
                            self._request = request
                            self._last_state = state
                            return
                    if self._clock() >= deadline:
                        raise TimeoutError(
                            "BPX state connection did not produce an odometry frame before timeout"
                        )
                    self._sleep(self._config.poll_period_s)
            except BaseException:
                try:
                    request.disconnect()
                finally:
                    self._clear_cache()
                raise

    def stop(self) -> None:
        with self._lock:
            request, self._request = self._request, None
            try:
                if request is not None:
                    request.disconnect()
            finally:
                self._clear_cache()

    def read_state(self) -> RobotState:
        with self._lock:
            request = self._request
            if request is None:
                raise RuntimeError("BPX read-only state source is not started")
            if not request.isConnected():
                received_at = self._last_received_at_s
                if received_at is None:
                    received_at = self._clock()
                return RobotState(connected=False, received_at_s=received_at)
            state = self._try_read(request)
            if state is not None:
                self._last_state = state
                return state
            if self._last_state is None:
                raise RuntimeError("BPX state is connected but has no valid odometry frame")
            return replace(self._last_state, connected=True)

    def _new_request(self) -> Any:
        if self._request_factory is not None:
            return self._request_factory()
        sdk = importlib.import_module("bpx_sdk")
        from .sdk_compat import validate_sdk_version
        validate_sdk_version(sdk)
        return sdk.RequestRobotState()

    def _clear_cache(self) -> None:
        self._last_state = None
        self._last_odometry_timestamp_ms = None
        self._last_received_at_s = None

    def _try_read(self, request: Any) -> Optional[RobotState]:
        odometry = request.getLegOdom()
        odometry_timestamp = request.getOdometryTimestamp()
        if odometry is None or odometry_timestamp is None:
            return None
        if not isinstance(odometry, dict):
            raise RuntimeError("getLegOdom() returned a non-dict value")

        timestamp_ms = _optional_int(odometry_timestamp, "odometry timestamp")
        now = self._clock()
        if timestamp_ms != self._last_odometry_timestamp_ms:
            self._last_odometry_timestamp_ms = timestamp_ms
            self._last_received_at_s = now
        assert self._last_received_at_s is not None

        return normalize_robot_state(RobotState(
            connected=True,
            received_at_s=self._last_received_at_s,
            position_m=_required_vector(odometry, "position", 3),
            orientation_xyzw=_required_vector(odometry, "orientation", 4),
            linear_velocity_body_mps=_required_vector(odometry, "velocity_body", 3),
            angular_velocity_body_rps=_required_vector(
                odometry, "angular_velocity", 3
            ),
            imu_orientation_xyzw=_optional_vector(request.getImuQuat(), 4, "IMU quaternion"),
            imu_linear_acceleration_mps2=_optional_vector(
                request.getImuAcc(), 3, "IMU acceleration"
            ),
            imu_angular_velocity_rps=_optional_vector(
                request.getImuOmega(), 3, "IMU angular velocity"
            ),
            joint_position_rad=_optional_vector(
                request.getJointPosition(), 12, "joint position"
            ),
            joint_velocity_rad_s=_optional_vector(
                request.getJointVelocity(), 12, "joint velocity"
            ),
            joint_torque_nm=_optional_vector(request.getJointTorque(), 12, "joint torque"),
            motor_temperature_c=_optional_vector(
                request.getMotorTemperature(), 12, "motor temperature"
            ),
            driver_temperature_c=_optional_vector(
                request.getDriverTemperature(), 12, "driver temperature"
            ),
            motion_state=_optional_int(request.getCurrentMotionState(), "motion state"),
            motion_gait=_optional_int(request.getCurrentGait(), "motion gait"),
            sub_gait=_optional_int(request.getSubGait(), "sub gait"),
            battery_level_percent=_optional_int(
                request.getBatteryLevel(), "battery level"
            ),
            battery_current_a=_optional_float(
                request.getBatteryCurrent(), "battery current"
            ),
            vendor_timestamps=VendorTimestamps(
                joint_ms=_optional_int(request.getJointStateTimestamp(), "joint timestamp"),
                imu_ms=_optional_int(request.getImuTimestamp(), "IMU timestamp"),
                odometry_ms=timestamp_ms,
                motion_state_ms=_optional_int(
                    request.getMotionStateTimestamp(), "motion-state timestamp"
                ),
                battery_ms=_optional_int(request.getBatteryTimestamp(), "battery timestamp"),
            ),
        ))


def _required_vector(value: Any, field: str, length: int) -> Tuple[float, ...]:
    if field not in value:
        raise RuntimeError("leg odometry is missing {!r}".format(field))
    return _optional_vector(value[field], length, "leg odometry {}".format(field), required=True)


def _optional_vector(
    value: Optional[Sequence[float]],
    length: int,
    name: str,
    required: bool = False,
) -> Tuple[float, ...]:
    if value is None:
        if required:
            raise RuntimeError("{} is unavailable".format(name))
        return ()
    if isinstance(value, (str, bytes)) or len(value) != length:
        raise RuntimeError("{} must contain {} values".format(name, length))
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise RuntimeError("{} contains a non-finite value".format(name))
    return result


def _optional_int(value: Any, name: str) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise RuntimeError("{} must be an integer".format(name))
    return value


def _optional_float(value: Any, name: str) -> Optional[float]:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError("{} must be finite".format(name))
    return result


def _bounded_int(value: Any, name: str, lower: int, upper: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
        raise ValueError("{} must be an integer in {}..{}".format(name, lower, upper))


def _positive_finite(value: Any, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number > 0".format(name))
    if not math.isfinite(float(value)) or float(value) <= 0.0:
        raise ValueError("{} must be a finite number > 0".format(name))
