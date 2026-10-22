"""Lifecycle and fail-closed command handling for one BPX backend."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, Optional

from .backend import BpxBackend
from .model import (
    CommandDecision,
    ControllerConfig,
    ControllerSnapshot,
    LifecycleState,
    PlanarTwist,
    RobotState,
    SafetyState,
    StopCommand,
    VelocityCommand,
    ZERO_TWIST,
)


class QuadrupedController:
    """Deep module containing command validation, lifecycle and watchdogs."""

    def __init__(
        self,
        backend: BpxBackend,
        config: ControllerConfig,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        config.validate()
        self._backend = backend
        self._config = config
        self._clock = clock
        self._lock = threading.RLock()
        self._lifecycle = LifecycleState.INACTIVE
        self._safety = SafetyState.DISARMED
        self._fault_reason = ""
        self._target: Optional[PlanarTwist] = None
        self._target_at_s: Optional[float] = None
        self._last_output = ZERO_TWIST
        self._last_output_at_s = clock()

    @property
    def backend_name(self) -> str:
        return self._backend.name

    @property
    def supports_twist(self) -> bool:
        return True

    @property
    def supports_posture(self) -> bool:
        return False

    def set_posture(self, posture_name: str) -> CommandDecision:
        del posture_name
        return CommandDecision(False, "controlled twist runtime has no posture service")

    def activate(self) -> None:
        with self._lock:
            if self._lifecycle is LifecycleState.SHUTDOWN:
                raise RuntimeError("controller is shut down")
            if self._lifecycle is LifecycleState.ACTIVE:
                return
            self._backend.start()
            state = self._backend.read_state()
            now = self._clock()
            if not state.connected:
                self._backend.stop()
                raise RuntimeError("backend did not connect")
            if not self._state_is_fresh(state, now):
                self._backend.stop()
                raise RuntimeError("backend did not provide a fresh state")
            for _ in range(self._config.zero_preamble_count):
                self._backend.apply(StopCommand("activation zero preamble"))
            self._lifecycle = LifecycleState.ACTIVE
            self._safety = SafetyState.DISARMED
            self._fault_reason = ""
            self._target = None
            self._target_at_s = None
            self._last_output = ZERO_TWIST
            self._last_output_at_s = now

    def deactivate(self) -> None:
        with self._lock:
            if self._lifecycle in {LifecycleState.INACTIVE, LifecycleState.SHUTDOWN}:
                return
            try:
                self._emit_stop("deactivate")
            finally:
                try:
                    self._backend.stop()
                finally:
                    self._lifecycle = LifecycleState.INACTIVE
                    self._safety = SafetyState.DISARMED
                    self._target = None
                    self._target_at_s = None

    def shutdown(self) -> None:
        with self._lock:
            if self._lifecycle is LifecycleState.SHUTDOWN:
                return
            try:
                if self._lifecycle is LifecycleState.ACTIVE:
                    self._emit_stop("shutdown")
            finally:
                try:
                    self._backend.stop()
                finally:
                    self._lifecycle = LifecycleState.SHUTDOWN
                    self._safety = SafetyState.DISARMED
                    self._target = None
                    self._target_at_s = None

    def arm(self) -> CommandDecision:
        with self._lock:
            if not self._config.allow_motion:
                return CommandDecision(False, "allow_motion is false")
            if self._lifecycle is not LifecycleState.ACTIVE:
                return CommandDecision(False, "controller is not active")
            if self._safety is SafetyState.FAULT:
                return CommandDecision(False, "fault must be cleared before arm")
            state = self._backend.read_state()
            now = self._clock()
            if not state.connected:
                return CommandDecision(False, "backend is disconnected")
            if not self._state_is_fresh(state, now):
                return CommandDecision(False, "state is stale")
            self._emit_stop("arm zero preamble")
            self._safety = SafetyState.ARMED
            return CommandDecision(True, "armed")

    def disarm(self, reason: str = "operator disarm") -> None:
        with self._lock:
            if self._safety is not SafetyState.FAULT:
                self._safety = SafetyState.DISARMED
            self._target = None
            self._target_at_s = None
            self._emit_stop(reason)

    def clear_fault(self) -> CommandDecision:
        with self._lock:
            if self._safety is not SafetyState.FAULT:
                return CommandDecision(True, "no fault latched")
            if self._lifecycle is not LifecycleState.ACTIVE:
                return CommandDecision(False, "controller is not active")
            state = self._backend.read_state()
            now = self._clock()
            if not state.connected or not self._state_is_fresh(state, now):
                return CommandDecision(False, "backend is not healthy")
            self._emit_stop("clear fault zero preamble")
            self._fault_reason = ""
            self._safety = SafetyState.DISARMED
            return CommandDecision(True, "fault cleared; controller remains disarmed")

    def submit_twist(self, command: PlanarTwist) -> CommandDecision:
        with self._lock:
            if not command.is_finite():
                self._latch_fault("non-finite twist")
                return CommandDecision(False, "non-finite twist")
            if self._lifecycle is not LifecycleState.ACTIVE:
                return CommandDecision(False, "controller is not active")
            if self._safety is not SafetyState.ARMED:
                self._emit_stop("twist rejected while not armed")
                return CommandDecision(False, "controller is not armed")
            self._target = self._clamp_speed(command)
            self._target_at_s = self._clock()
            return CommandDecision(True, "accepted")

    def tick(self, state: Optional[RobotState] = None) -> RobotState:
        with self._lock:
            if state is None:
                state = self._backend.read_state()
            now = self._clock()
            if self._lifecycle is not LifecycleState.ACTIVE:
                return state
            if not state.connected:
                self._latch_fault("backend disconnected")
                return state
            if not self._state_is_fresh(state, now):
                self._latch_fault("state watchdog expired")
                return state
            if self._safety is not SafetyState.ARMED:
                self._emit_stop("not armed")
                return state
            if self._target is None or self._target_at_s is None:
                target = ZERO_TWIST
            elif now - self._target_at_s > self._config.command_timeout_s:
                self._target = None
                self._target_at_s = None
                self._emit_stop("command watchdog expired")
                return state
            else:
                target = self._target
            output = self._limit_acceleration(target, now)
            self._backend.apply(VelocityCommand(output))
            self._last_output = output
            self._last_output_at_s = now
            return state

    def snapshot(self) -> ControllerSnapshot:
        with self._lock:
            return ControllerSnapshot(
                lifecycle=self._lifecycle,
                safety=self._safety,
                fault_reason=self._fault_reason,
                last_output=self._last_output,
            )

    def trip_fault(self, reason: str) -> None:
        """Latch an adapter/feedback fault before attempting a hardware stop."""
        with self._lock:
            self._latch_fault(reason)

    def _state_is_fresh(self, state: RobotState, now: float) -> bool:
        age = now - state.received_at_s
        return math.isfinite(age) and 0.0 <= age <= self._config.state_timeout_s

    def _clamp_speed(self, command: PlanarTwist) -> PlanarTwist:
        return PlanarTwist(
            linear_x_mps=self._clamp(
                command.linear_x_mps,
                -self._config.max_linear_x_mps,
                self._config.max_linear_x_mps,
            ),
            linear_y_mps=self._clamp(
                command.linear_y_mps,
                -self._config.max_linear_y_mps,
                self._config.max_linear_y_mps,
            ),
            angular_z_rps=self._clamp(
                command.angular_z_rps,
                -self._config.max_angular_z_rps,
                self._config.max_angular_z_rps,
            ),
        )

    def _limit_acceleration(self, target: PlanarTwist, now: float) -> PlanarTwist:
        dt = max(0.0, now - self._last_output_at_s)
        linear_delta = self._config.max_linear_accel_mps2 * dt
        angular_delta = self._config.max_angular_accel_rps2 * dt
        return PlanarTwist(
            linear_x_mps=self._approach(
                self._last_output.linear_x_mps, target.linear_x_mps, linear_delta
            ),
            linear_y_mps=self._approach(
                self._last_output.linear_y_mps, target.linear_y_mps, linear_delta
            ),
            angular_z_rps=self._approach(
                self._last_output.angular_z_rps, target.angular_z_rps, angular_delta
            ),
        )

    def _emit_stop(self, reason: str) -> None:
        try:
            self._backend.apply(StopCommand(reason))
        finally:
            self._last_output = ZERO_TWIST
            self._last_output_at_s = self._clock()

    def _latch_fault(self, reason: str) -> None:
        self._fault_reason = reason
        self._safety = SafetyState.FAULT
        self._target = None
        self._target_at_s = None
        self._emit_stop(reason)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)
