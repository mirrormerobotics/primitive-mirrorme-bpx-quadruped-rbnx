"""Bounded BPX MotionLevelControl posture session.

This module owns one ``MotionLevelControl`` object, which inherits the
``RequestRobotState`` interface and therefore supplies both telemetry and the
two posture commands used here. It deliberately exposes no velocity, gait,
zero-position, damping, or joint-level command method.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib
import math
import threading
import time
from typing import Any, Callable, Optional, Protocol

from .model import RobotState
from .sdk_state_source import SdkStateConfig, SdkStateSource


MOTION_STATE_LYING_DOWN = 0
MOTION_STATE_MOTION = 6


@dataclass(frozen=True)
class SdkPostureConfig:
    state: SdkStateConfig
    motion_command_rate_hz: int = 50

    def validate(self) -> None:
        self.state.validate()
        if (
            isinstance(self.motion_command_rate_hz, bool)
            or not isinstance(self.motion_command_rate_hz, int)
            or not 1 <= self.motion_command_rate_hz <= 200
        ):
            raise ValueError("motion_command_rate_hz must be in 1..200")


@dataclass(frozen=True)
class PostureCycleConfig:
    stand_timeout_s: float = 15.0
    hold_duration_s: float = 3.0
    sit_timeout_s: float = 15.0
    poll_period_s: float = 0.2
    state_timeout_s: float = 0.5
    cleanup_sit_flush_s: float = 1.0

    def validate(self) -> None:
        bounded = {
            "stand_timeout_s": (self.stand_timeout_s, 0.1, 60.0),
            "hold_duration_s": (self.hold_duration_s, 0.1, 30.0),
            "sit_timeout_s": (self.sit_timeout_s, 0.1, 60.0),
            "poll_period_s": (self.poll_period_s, 0.01, 1.0),
            "state_timeout_s": (self.state_timeout_s, 0.1, 5.0),
            "cleanup_sit_flush_s": (self.cleanup_sit_flush_s, 0.1, 3.0),
        }
        for name, (value, lower, upper) in bounded.items():
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ValueError("{} must be a number".format(name))
            if not math.isfinite(float(value)) or not lower <= value <= upper:
                raise ValueError(
                    "{} must be finite and in {}..{}".format(name, lower, upper)
                )


class PostureSession(Protocol):
    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def read_state(self) -> RobotState:
        ...

    def request_stand_up(self) -> bool:
        ...

    def request_sit_down(self) -> bool:
        ...


class SdkPostureSession:
    """Own one SDK object and expose only stand/sit plus inherited state."""

    def __init__(
        self,
        config: SdkPostureConfig,
        *,
        control_factory: Optional[Callable[[], Any]] = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        config.validate()
        self._config = config
        self._control_factory = control_factory
        self._clock = clock
        self._sleep = sleep
        self._sdk_lock = threading.RLock()
        self._control: Optional[Any] = None
        self._source: Optional[SdkStateSource] = None

    @property
    def supports_twist(self) -> bool:
        return False

    def start(self) -> None:
        with self._sdk_lock:
            if self._source is not None:
                return

            def configured_control() -> Any:
                control = self._new_control()
                self._require_posture_api(control)
                control.setMotionCommandRate(self._config.motion_command_rate_hz)
                # Make the non-velocity intent explicit before connect starts the
                # SDK's periodic motion packet sender.
                control.setVelocityControlFlag(False)
                self._control = control
                return control

            source = SdkStateSource(
                self._config.state,
                request_factory=configured_control,
                clock=self._clock,
                sleep=self._sleep,
            )
            try:
                source.start()
            except BaseException:
                self._control = None
                raise
            self._source = source

    def stop(self) -> None:
        with self._sdk_lock:
            source, self._source = self._source, None
            control, self._control = self._control, None
            try:
                if control is not None:
                    control.setVelocityControlFlag(False)
            finally:
                if source is not None:
                    source.stop()

    def read_state(self) -> RobotState:
        with self._sdk_lock:
            if self._source is None:
                raise RuntimeError("BPX posture session is not started")
            return self._source.read_state()

    def request_stand_up(self) -> bool:
        with self._sdk_lock:
            return bool(self._require_started_control().setStandUp())

    def request_sit_down(self) -> bool:
        with self._sdk_lock:
            return bool(self._require_started_control().setSitDown())

    def _require_started_control(self) -> Any:
        if self._control is None or self._source is None:
            raise RuntimeError("BPX posture session is not started")
        return self._control

    def _new_control(self) -> Any:
        if self._control_factory is not None:
            return self._control_factory()
        sdk = importlib.import_module("bpx_sdk")
        from .sdk_compat import validate_sdk_version
        validate_sdk_version(sdk)
        control_type = getattr(sdk, "MotionLevelControl", None)
        if control_type is None:
            raise RuntimeError("BPX SDK has no MotionLevelControl")
        return control_type()

    @staticmethod
    def _require_posture_api(control: Any) -> None:
        required = (
            "setMotionCommandRate",
            "setVelocityControlFlag",
            "setStandUp",
            "setSitDown",
        )
        missing = [name for name in required if not callable(getattr(control, name, None))]
        if missing:
            raise RuntimeError(
                "MotionLevelControl is missing: {}".format(", ".join(missing))
            )


def run_posture_cycle(
    session: PostureSession,
    config: PostureCycleConfig,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    stop_requested: Callable[[], bool] = lambda: False,
    report: Callable[[str], None] = print,
) -> None:
    """Stand from a confirmed lying state, hold, then return to lying down."""

    config.validate()
    started = False
    stand_phase_started = False
    completed = False
    try:
        session.start()
        started = True
        initial = require_healthy_state(session, config, clock)
        if initial.motion_state != MOTION_STATE_LYING_DOWN:
            raise RuntimeError(
                "initial motion state must be LyingDown(0), got {}".format(
                    initial.motion_state
                )
            )
        report("initial state confirmed: LyingDown(0)")

        stand_phase_started = True
        command_until_state(
            session,
            config,
            target_state=MOTION_STATE_MOTION,
            target_name="Motion(6)",
            command=session.request_stand_up,
            command_name="setStandUp",
            timeout_s=config.stand_timeout_s,
            clock=clock,
            sleep=sleep,
            stop_requested=stop_requested,
            report=report,
        )

        report("standing confirmed; holding for {:.3f}s".format(config.hold_duration_s))
        hold_deadline = clock() + config.hold_duration_s
        while clock() < hold_deadline:
            raise_if_stopped(stop_requested)
            hold_state = require_healthy_state(session, config, clock)
            if hold_state.motion_state != MOTION_STATE_MOTION:
                raise RuntimeError(
                    "robot left Motion(6) during standing hold: {}".format(
                        hold_state.motion_state
                    )
                )
            sleep(min(config.poll_period_s, max(0.0, hold_deadline - clock())))

        command_until_state(
            session,
            config,
            target_state=MOTION_STATE_LYING_DOWN,
            target_name="LyingDown(0)",
            command=session.request_sit_down,
            command_name="setSitDown",
            timeout_s=config.sit_timeout_s,
            clock=clock,
            sleep=sleep,
            stop_requested=stop_requested,
            report=report,
        )
        completed = True
        report("posture cycle complete: LyingDown(0)")
    finally:
        if started and stand_phase_started and not completed:
            report("cycle interrupted or failed; sending bounded sit-down cleanup")
            flush_sit_down(session, config, clock=clock, sleep=sleep)
        if started:
            session.stop()


def command_until_state(
    session: PostureSession,
    config: PostureCycleConfig,
    *,
    target_state: int,
    target_name: str,
    command: Callable[[], bool],
    command_name: str,
    timeout_s: float,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
    stop_requested: Callable[[], bool],
    report: Callable[[str], None],
) -> None:
    deadline = clock() + timeout_s
    command_announced = False
    while True:
        raise_if_stopped(stop_requested)
        state = require_healthy_state(session, config, clock)
        if state.motion_state == target_state:
            report("state confirmed: {}".format(target_name))
            return
        if clock() >= deadline:
            raise TimeoutError(
                "{} did not reach {} within {:.3f}s".format(
                    command_name, target_name, timeout_s
                )
            )
        if not command():
            raise RuntimeError("{} returned false".format(command_name))
        if not command_announced:
            report("{} accepted; waiting for {}".format(command_name, target_name))
            command_announced = True
        sleep(config.poll_period_s)


def require_healthy_state(
    session: PostureSession,
    config: PostureCycleConfig,
    clock: Callable[[], float],
) -> RobotState:
    state = session.read_state()
    if not state.connected:
        raise RuntimeError("BPX motion state session disconnected")
    age_s = clock() - state.received_at_s
    if not math.isfinite(age_s) or not 0.0 <= age_s <= config.state_timeout_s:
        raise RuntimeError("BPX odometry state is stale")
    if state.motion_state is None:
        raise RuntimeError("BPX motion state is unavailable")
    return state


def raise_if_stopped(stop_requested: Callable[[], bool]) -> None:
    if stop_requested():
        raise InterruptedError("posture cycle interrupted")


def flush_sit_down(
    session: PostureSession,
    config: PostureCycleConfig,
    *,
    clock: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    deadline = clock() + config.cleanup_sit_flush_s
    while clock() < deadline:
        try:
            session.request_sit_down()
        except Exception:
            pass
        sleep(min(config.poll_period_s, max(0.0, deadline - clock())))
