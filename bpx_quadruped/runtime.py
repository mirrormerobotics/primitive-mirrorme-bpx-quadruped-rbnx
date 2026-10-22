"""Provider runtime seam for controlled, posture, and read-only adapters."""

from __future__ import annotations

import time
from typing import Callable, Optional, Protocol

from .backend import BpxStateSource
from .config import ProviderConfig
from .controller import QuadrupedController
from .fake_backend import FakeBpxBackend
from .model import CommandDecision, PlanarTwist, RobotState
from .posture_runtime import PostureRuntime, PostureRuntimeConfig
from .replay import ReplayStateSource
from .sdk_posture import SdkPostureConfig, SdkPostureSession
from .sdk_state_source import SdkStateConfig, SdkStateSource
from .telemetry import state_for_publication


class ProviderRuntime(Protocol):
    """Small interface consumed by the Robonix lifecycle provider."""

    @property
    def supports_twist(self) -> bool:
        ...

    @property
    def supports_posture(self) -> bool:
        ...

    def activate(self) -> None:
        ...

    def deactivate(self) -> None:
        ...

    def shutdown(self) -> None:
        ...

    def arm(self) -> CommandDecision:
        ...

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        ...

    def set_posture(self, posture_name: str) -> CommandDecision:
        ...

    def tick(self) -> RobotState:
        ...


class ReadOnlyRuntime:
    """Adapt a ``BpxStateSource`` without inventing a command interface."""

    def __init__(self, source: BpxStateSource) -> None:
        self._source = source
        self._active = False
        self._shutdown = False

    @property
    def supports_twist(self) -> bool:
        return False

    @property
    def supports_posture(self) -> bool:
        return False

    def activate(self) -> None:
        if self._shutdown:
            raise RuntimeError("read-only runtime is shut down")
        if self._active:
            return
        self._source.start()
        self._active = True

    def deactivate(self) -> None:
        if not self._active:
            return
        try:
            self._source.stop()
        finally:
            self._active = False

    def shutdown(self) -> None:
        if self._shutdown:
            return
        try:
            self.deactivate()
        finally:
            self._shutdown = True

    def arm(self) -> CommandDecision:
        return CommandDecision(False, "read-only runtime cannot arm")

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        del command
        return CommandDecision(False, "read-only runtime has no command interface")

    def set_posture(self, posture_name: str) -> CommandDecision:
        del posture_name
        return CommandDecision(False, "read-only runtime has no posture interface")

    def tick(self) -> RobotState:
        if not self._active:
            raise RuntimeError("read-only runtime is not active")
        return self._source.read_state()


def build_runtime(
    config: ProviderConfig,
    *,
    sdk_request_factory: Optional[Callable[[], object]] = None,
    sdk_control_factory: Optional[Callable[[], object]] = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> ProviderRuntime:
    """Build the single runtime adapter selected by validated configuration."""

    if config.enable_motion_service:
        from .motion_runtime import MotionRuntime
        from .sdk_motion import SdkMotionBackend
        backend = FakeBpxBackend(clock=clock) if config.backend == "fake" else SdkMotionBackend(
            SdkPostureConfig(SdkStateConfig(
                robot_ip=config.robot_ip, robot_state_port=config.robot_state_port,
                tcp_local_port=config.sdk_tcp_local_port, state_rate_hz=config.state_rate_hz,
                connect_timeout_s=config.sdk_connect_timeout_s, poll_period_s=config.sdk_poll_period_s,
            ), motion_command_rate_hz=config.command_rate_hz),
            control_factory=sdk_control_factory, clock=clock, sleep=sleep,
        )
        return MotionRuntime(backend, config.controller, command_rate_hz=config.command_rate_hz,
                             lease_s=config.motion_lease_s, clock=clock, sleep=sleep,
                             move_min_linear_mps=config.move_min_linear_mps,
                             move_min_angular_rps=config.move_min_angular_rps)

    if config.backend == "fake":
        return QuadrupedController(
            FakeBpxBackend(clock=clock),
            config.controller,
            clock=clock,
        )

    if config.backend == "replay":
        assert config.replay_path is not None
        return ReadOnlyRuntime(
            ReplayStateSource.from_path(config.replay_path, clock=clock)
        )

    state_config = SdkStateConfig(
        robot_ip=config.robot_ip,
        robot_state_port=config.robot_state_port,
        tcp_local_port=config.sdk_tcp_local_port,
        state_rate_hz=config.state_rate_hz,
        connect_timeout_s=config.sdk_connect_timeout_s,
        poll_period_s=config.sdk_poll_period_s,
    )
    if config.enable_posture_service:
        return PostureRuntime(
            SdkPostureSession(
                SdkPostureConfig(
                    state=state_config,
                    motion_command_rate_hz=config.command_rate_hz,
                ),
                control_factory=sdk_control_factory,
                clock=clock,
                sleep=sleep,
            ),
            PostureRuntimeConfig(
                stand_timeout_s=config.posture_stand_timeout_s,
                sit_timeout_s=config.posture_sit_timeout_s,
                poll_period_s=config.posture_poll_period_s,
                state_timeout_s=config.controller.state_timeout_s,
                cleanup_sit_flush_s=config.posture_cleanup_sit_flush_s,
            ),
            clock=clock,
            sleep=sleep,
        )

    source = SdkStateSource(
        state_config,
        request_factory=sdk_request_factory,
        clock=clock,
        sleep=sleep,
    )
    return ReadOnlyRuntime(source)


def state_is_publishable(state: RobotState, *, now_s: float, timeout_s: float) -> bool:
    """Reject disconnected, stale, future-dated, or invalid host samples."""

    return state_for_publication(state, now_s=now_s, timeout_s=timeout_s) is not None
