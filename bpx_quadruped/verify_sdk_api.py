"""Offline compatibility check for the externally supplied BPX SDK wheel."""

from __future__ import annotations

import argparse
import json
from .sdk_compat import validate_sdk_version
from types import ModuleType
from typing import Any, Dict, Iterable, Optional


REQUIRED_READ_METHODS = (
    "connect",
    "disconnect",
    "isConnected",
    "setRobotIp",
    "setRobotStateUploadPort",
    "setRobotStateUploadRate",
    "setTcpLocalPort",
    "getLegOdom",
    "getOdometryTimestamp",
    "getImuQuat",
    "getImuAcc",
    "getImuOmega",
    "getJointPosition",
    "getJointVelocity",
    "getJointTorque",
    "getMotorTemperature", "getDriverTemperature",
    "getCurrentMotionState", "getCurrentGait", "getSubGait",
    "getBatteryLevel", "getBatteryCurrent",
    "getJointStateTimestamp", "getImuTimestamp",
    "getMotionStateTimestamp", "getBatteryTimestamp",
)

MOTION_METHODS = (
    "setZeroPositionsFlag",
    "setVelocity",
    "setStandUp",
    "setSitDown",
    "setDamping",
)

REQUIRED_POSTURE_METHODS = (
    "setMotionCommandRate",
    "setVelocityControlFlag",
    "setStandUp",
    "setSitDown",
)


def validate_sdk_module(sdk: ModuleType) -> Dict[str, Any]:
    version = validate_sdk_version(sdk)
    request_type = getattr(sdk, "RequestRobotState", None)
    if request_type is None:
        raise RuntimeError("BPX SDK has no RequestRobotState")
    missing = [name for name in REQUIRED_READ_METHODS if not hasattr(request_type, name)]
    if missing:
        raise RuntimeError("RequestRobotState is missing: {}".format(", ".join(missing)))
    leaked_motion = [name for name in MOTION_METHODS if hasattr(request_type, name)]
    if leaked_motion:
        raise RuntimeError(
            "RequestRobotState unexpectedly exposes motion methods: {}".format(
                ", ".join(leaked_motion)
            )
        )
    return {
        "sdk_version": version,
        "state_api_methods_checked": len(REQUIRED_READ_METHODS),
        "request_robot_state_has_motion_methods": False,
    }


def validate_posture_sdk_module(sdk: ModuleType) -> Dict[str, Any]:
    """Check the narrow MotionLevelControl API used by the posture probe."""

    version = validate_sdk_version(sdk)
    control_type = getattr(sdk, "MotionLevelControl", None)
    if control_type is None:
        raise RuntimeError("BPX SDK has no MotionLevelControl")
    missing_state = [
        name for name in REQUIRED_READ_METHODS if not hasattr(control_type, name)
    ]
    if missing_state:
        raise RuntimeError(
            "MotionLevelControl state API is missing: {}".format(
                ", ".join(missing_state)
            )
        )
    missing = [
        name for name in REQUIRED_POSTURE_METHODS if not hasattr(control_type, name)
    ]
    if missing:
        raise RuntimeError(
            "MotionLevelControl is missing: {}".format(", ".join(missing))
        )
    motion_state_type = getattr(sdk, "MotionState", None)
    if motion_state_type is None:
        raise RuntimeError("BPX SDK has no MotionState enum")
    expected_states = {"LyingDown": 0, "Motion": 6}
    for name, expected in expected_states.items():
        member = getattr(motion_state_type, name, None)
        try:
            actual = int(member)
        except (TypeError, ValueError):
            actual = None
        if actual != expected:
            raise RuntimeError(
                "BPX SDK MotionState.{} must be {}, got {!r}".format(
                    name, expected, member
                )
            )
    return {
        "motion_level_control_state_methods_checked": len(REQUIRED_READ_METHODS),
        "motion_level_control_methods_checked": len(REQUIRED_POSTURE_METHODS),
        "motion_states": {"LyingDown": 0, "Motion": 6},
        "posture_commands": ["setStandUp", "setSitDown"],
        "supports_twist": False,
        "uses_joint_level_control": False,
    }


def validate_yaw_circle_sdk_module(sdk: ModuleType) -> Dict[str, Any]:
    """Check the additional API used only by the manual yaw-circle probe."""

    result = validate_posture_sdk_module(sdk)
    control_type = sdk.MotionLevelControl
    if not hasattr(control_type, "setVelocity"):
        raise RuntimeError("MotionLevelControl is missing: setVelocity")
    result.update(
        {
            "yaw_circle_probe_methods": ["setVelocity"],
            "yaw_circle_probe_only": True,
            "supports_twist": False,
        }
    )
    return result


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--posture",
        action="store_true",
        help="also verify the narrow stand/sit MotionLevelControl API",
    )
    parser.add_argument(
        "--yaw-circle",
        action="store_true",
        help="also verify setVelocity for the isolated manual yaw-circle probe",
    )
    parser.add_argument("--unified", action="store_true", help="verify unified posture/move/Twist API")
    args = parser.parse_args(argv)

    import bpx_sdk

    result = validate_sdk_module(bpx_sdk)
    if args.unified:
        result.update(validate_yaw_circle_sdk_module(bpx_sdk))
        if not hasattr(bpx_sdk.MotionLevelControl, "setWalk"):
            raise RuntimeError("MotionLevelControl is missing: setWalk")
        result.update(supports_twist=True, yaw_circle_probe_only=False, unified=True)
    elif args.yaw_circle:
        result.update(validate_yaw_circle_sdk_module(bpx_sdk))
    elif args.posture:
        result.update(validate_posture_sdk_module(bpx_sdk))
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
