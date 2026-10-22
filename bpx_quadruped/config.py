"""Strict parsing for deployment-facing provider configuration."""

from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import math
import os
from typing import Any, Mapping, Optional

from .model import ControllerConfig


@dataclass(frozen=True)
class ProviderConfig:
    backend: str
    controller: ControllerConfig
    robot_ip: str
    robot_state_port: int
    state_rate_hz: int
    command_rate_hz: int
    sdk_tcp_local_port: int
    sdk_connect_timeout_s: float
    sdk_poll_period_s: float
    replay_path: Optional[str]
    cmd_vel_topic: str
    odom_topic: str
    joint_states_topic: str
    odom_frame: str
    base_frame: str
    odom_rate_hz: float
    publish_odom_tf: bool
    auto_arm_fake: bool
    enable_posture_service: bool
    posture_stand_timeout_s: float
    posture_sit_timeout_s: float
    posture_poll_period_s: float
    posture_cleanup_sit_flush_s: float
    enable_motion_service: bool = False
    motion_lease_s: float = 30.0
    move_min_linear_mps: float = 0.25
    move_min_angular_rps: float = 0.25
    publish_imu: bool = False
    imu_topic: str = "/imu/data"
    imu_frame: str = ""
    odom_pose_variance: tuple = (1.0,) * 6
    odom_twist_variance: tuple = (1.0,) * 6

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "ProviderConfig":
        allowed = {
            "backend",
            "allow_motion",
            "robot_ip",
            "robot_state_port",
            "state_rate_hz",
            "command_rate_hz",
            "sdk_tcp_local_port",
            "sdk_connect_timeout_s",
            "sdk_poll_period_s",
            "replay_path",
            "command_timeout_s",
            "state_timeout_s",
            "max_linear_x_mps",
            "max_linear_y_mps",
            "max_angular_z_rps",
            "max_linear_accel_mps2",
            "max_angular_accel_rps2",
            "zero_preamble_count",
            "cmd_vel_topic",
            "odom_topic",
            "joint_states_topic",
            "odom_frame",
            "base_frame",
            "odom_rate_hz",
            "publish_odom_tf",
            "auto_arm_fake",
            "enable_posture_service",
            "posture_stand_timeout_s",
            "posture_sit_timeout_s",
            "posture_poll_period_s",
            "posture_cleanup_sit_flush_s",
            "enable_motion_service",
            "motion_lease_s",
            "move_min_linear_mps", "move_min_angular_rps",
            "publish_imu", "imu_topic", "imu_frame",
            "odom_pose_variance", "odom_twist_variance",
        }
        unknown = sorted(set(values).difference(allowed))
        if unknown:
            raise ValueError("unknown provider config fields: {}".format(", ".join(unknown)))

        backend = _string(values.get("backend", "fake"), "backend")
        if backend not in {"fake", "sdk", "replay"}:
            raise ValueError("backend must be 'fake', 'sdk', or 'replay'")

        controller_names = {
            "allow_motion",
            "command_timeout_s",
            "state_timeout_s",
            "max_linear_x_mps",
            "max_linear_y_mps",
            "max_angular_z_rps",
            "max_linear_accel_mps2",
            "max_angular_accel_rps2",
            "zero_preamble_count",
        }
        controller = ControllerConfig.from_mapping(
            {key: values[key] for key in controller_names if key in values}
        )

        robot_ip = _string(values.get("robot_ip", "10.21.20.1"), "robot_ip")
        try:
            address = ipaddress.ip_address(robot_ip)
        except ValueError as exc:
            raise ValueError("robot_ip must be an IPv4 address") from exc
        if not isinstance(address, ipaddress.IPv4Address):
            raise ValueError("robot_ip must be an IPv4 address")

        robot_state_port = _bounded_int(
            values.get("robot_state_port", 9873), "robot_state_port", 1024, 65535
        )
        state_rate_hz = _bounded_int(
            values.get("state_rate_hz", 50), "state_rate_hz", 1, 200
        )
        command_rate_hz = _bounded_int(
            values.get("command_rate_hz", 50), "command_rate_hz", 1, 200
        )
        odom_rate_hz = _bounded_float(
            values.get("odom_rate_hz", 50.0), "odom_rate_hz", 1.0, 200.0
        )
        auto_arm_fake = values.get("auto_arm_fake", False)
        if not isinstance(auto_arm_fake, bool):
            raise ValueError("auto_arm_fake must be a boolean")
        if auto_arm_fake and backend != "fake":
            raise ValueError("auto_arm_fake is only valid with backend=fake")
        enable_posture_service = values.get("enable_posture_service", False)
        enable_motion_service = values.get("enable_motion_service", False)
        if not isinstance(enable_motion_service, bool):
            raise ValueError("enable_motion_service must be a boolean")
        if enable_motion_service:
            if backend not in {"sdk", "fake"} or not controller.allow_motion:
                raise ValueError("motion-test requires sdk/fake and allow_motion=true")
            if enable_posture_service or auto_arm_fake:
                raise ValueError("motion-test requires local arm and no separate posture service")
            if not 10 <= command_rate_hz <= 100:
                raise ValueError("motion-test command_rate_hz must be 10..100")
        if not isinstance(enable_posture_service, bool):
            raise ValueError("enable_posture_service must be a boolean")
        if enable_posture_service and backend != "sdk":
            raise ValueError("enable_posture_service is only valid with backend=sdk")
        if enable_posture_service and not controller.allow_motion:
            raise ValueError("enable_posture_service requires allow_motion=true")
        if backend == "sdk" and controller.allow_motion and not (enable_posture_service or enable_motion_service):
            raise ValueError(
                "backend=sdk requires allow_motion=false unless the guarded "
                "posture service is enabled"
            )
        if backend == "replay" and controller.allow_motion:
            raise ValueError("backend=replay requires allow_motion=false")

        replay_path_value = values.get("replay_path")
        replay_path: Optional[str] = None
        if replay_path_value is not None:
            replay_path = _string(replay_path_value, "replay_path")
            if not os.path.isabs(replay_path):
                raise ValueError("replay_path must be an absolute path")
        if backend == "replay" and replay_path is None:
            raise ValueError("backend=replay requires replay_path")
        if backend != "replay" and replay_path is not None:
            raise ValueError("replay_path is only valid with backend=replay")

        publish_odom_tf = values.get("publish_odom_tf", False)
        if not isinstance(publish_odom_tf, bool):
            raise ValueError("publish_odom_tf must be a boolean")
        publish_imu = values.get("publish_imu", False)
        if not isinstance(publish_imu, bool):
            raise ValueError("publish_imu must be a boolean")
        imu_frame = values.get("imu_frame", "")
        if publish_imu or imu_frame:
            imu_frame = _frame(imu_frame, "imu_frame")

        sdk_tcp_local_port = _bounded_int(
            values.get("sdk_tcp_local_port", 0), "sdk_tcp_local_port", 0, 65535
        )
        sdk_connect_timeout_s = _bounded_float(
            values.get("sdk_connect_timeout_s", 10.0),
            "sdk_connect_timeout_s",
            0.1,
            300.0,
        )
        sdk_poll_period_s = _bounded_float(
            values.get("sdk_poll_period_s", 0.05),
            "sdk_poll_period_s",
            0.001,
            1.0,
        )

        return cls(
            backend=backend,
            controller=controller,
            robot_ip=str(address),
            robot_state_port=robot_state_port,
            state_rate_hz=state_rate_hz,
            command_rate_hz=command_rate_hz,
            sdk_tcp_local_port=sdk_tcp_local_port,
            sdk_connect_timeout_s=sdk_connect_timeout_s,
            sdk_poll_period_s=sdk_poll_period_s,
            replay_path=replay_path,
            cmd_vel_topic=_topic(values.get("cmd_vel_topic", "/cmd_vel"), "cmd_vel_topic"),
            odom_topic=_topic(values.get("odom_topic", "/odom"), "odom_topic"),
            joint_states_topic=_topic(
                values.get("joint_states_topic", "/joint_states"),
                "joint_states_topic",
            ),
            odom_frame=_frame(values.get("odom_frame", "odom"), "odom_frame"),
            base_frame=_frame(values.get("base_frame", "base_link"), "base_frame"),
            odom_rate_hz=odom_rate_hz,
            publish_odom_tf=publish_odom_tf,
            publish_imu=publish_imu,
            imu_topic=_topic(values.get("imu_topic", "/imu/data"), "imu_topic"),
            imu_frame=imu_frame,
            odom_pose_variance=_variance(values.get("odom_pose_variance", [1.0]*6), "odom_pose_variance"),
            odom_twist_variance=_variance(values.get("odom_twist_variance", [1.0]*6), "odom_twist_variance"),
            auto_arm_fake=auto_arm_fake,
            enable_motion_service=enable_motion_service,
            motion_lease_s=_bounded_float(values.get("motion_lease_s", 30.0), "motion_lease_s", 1.0, 120.0),
            move_min_linear_mps=_bounded_float(values.get("move_min_linear_mps", .25), "move_min_linear_mps", .2, .3),
            move_min_angular_rps=_bounded_float(values.get("move_min_angular_rps", .25), "move_min_angular_rps", .2, .4),
            enable_posture_service=enable_posture_service,
            posture_stand_timeout_s=_bounded_float(
                values.get("posture_stand_timeout_s", 15.0),
                "posture_stand_timeout_s",
                0.1,
                60.0,
            ),
            posture_sit_timeout_s=_bounded_float(
                values.get("posture_sit_timeout_s", 15.0),
                "posture_sit_timeout_s",
                0.1,
                60.0,
            ),
            posture_poll_period_s=_bounded_float(
                values.get("posture_poll_period_s", 0.2),
                "posture_poll_period_s",
                0.01,
                1.0,
            ),
            posture_cleanup_sit_flush_s=_bounded_float(
                values.get("posture_cleanup_sit_flush_s", 1.0),
                "posture_cleanup_sit_flush_s",
                0.1,
                3.0,
            ),
        )


def validate_required_backend(
    config: ProviderConfig, required_backend: Optional[str]
) -> None:
    """Keep a selected package capability manifest aligned with its runtime."""

    if required_backend is None:
        return
    if required_backend not in {"fake", "sdk", "replay"}:
        raise ValueError("BPX_REQUIRED_BACKEND must be 'fake', 'sdk', or 'replay'")
    if config.backend != required_backend:
        raise ValueError(
            "selected package manifest requires backend={}".format(required_backend)
        )


def validate_required_posture_capability(
    config: ProviderConfig, required: Optional[str]
) -> None:
    """Keep the pre-bootstrap gRPC registration aligned with runtime config."""

    if required is None:
        return
    normalized = required.strip().lower()
    if normalized not in {"true", "false"}:
        raise ValueError("BPX_REQUIRED_POSTURE_CAPABILITY must be true or false")
    expected = normalized == "true"
    if config.enable_posture_service != expected:
        raise ValueError(
            "selected package manifest requires enable_posture_service={}".format(
                str(expected).lower()
            )
        )


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("{} must be a non-empty string".format(name))
    return value.strip()


def _bounded_int(value: Any, name: str, lower: int, upper: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("{} must be an integer".format(name))
    if not lower <= value <= upper:
        raise ValueError("{} must be in {}..{}".format(name, lower, upper))
    return value


def _bounded_float(value: Any, name: str, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a number".format(name))
    result = float(value)
    if not math.isfinite(result) or not lower <= result <= upper:
        raise ValueError("{} must be finite and in {}..{}".format(name, lower, upper))
    return result


def _variance(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(name + " must contain six positive variances")
    result = tuple(_bounded_float(v, name, 1e-12, 1e12) for v in value)
    return result


def _topic(value: Any, name: str) -> str:
    result = _string(value, name)
    if not result.startswith("/") or " " in result:
        raise ValueError("{} must be an absolute ROS topic name".format(name))
    return result


def _frame(value: Any, name: str) -> str:
    result = _string(value, name)
    if result.startswith("/") or " " in result:
        raise ValueError("{} must be a ROS frame id without a leading slash".format(name))
    return result
