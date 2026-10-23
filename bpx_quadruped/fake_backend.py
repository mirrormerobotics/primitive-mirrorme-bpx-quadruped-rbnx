"""Deterministic backend used by tests and no-hardware deployments."""

from __future__ import annotations

import math
import threading
import time
from typing import Callable, List

from .backend import BackendCommand
from .model import (
    PlanarTwist,
    PostureCommand,
    RobotState,
    StopCommand,
    VelocityCommand,
    ZERO_TWIST,
)


class FakeBpxBackend:
    def __init__(self, clock: Callable[[], float] = time.monotonic, *, posture=False) -> None:
        self._clock = clock
        self._lock = threading.RLock()
        self._connected = False
        self._freeze_state = False
        self._last_state_at = clock()
        self._last_integrated_at = self._last_state_at
        self._twist = ZERO_TWIST
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._motion_state = 0 if posture else None
        self.commands: List[BackendCommand] = []
        self.start_count = 0
        self.stop_count = 0

    @property
    def name(self) -> str:
        return "fake"

    def start(self) -> None:
        with self._lock:
            self.start_count += 1
            self._connected = True
            now = self._clock()
            self._last_state_at = now
            self._last_integrated_at = now

    def stop(self) -> None:
        with self._lock:
            self.stop_count += 1
            self._twist = ZERO_TWIST
            self._connected = False

    def apply(self, command: BackendCommand) -> None:
        with self._lock:
            self.commands.append(command)
            if isinstance(command, VelocityCommand):
                self._twist = command.twist
            elif isinstance(command, StopCommand):
                self._twist = ZERO_TWIST
            elif isinstance(command, PostureCommand):
                self._twist = ZERO_TWIST

    def read_state(self) -> RobotState:
        with self._lock:
            now = self._clock()
            dt = max(0.0, now - self._last_integrated_at)
            self._last_integrated_at = now
            self._yaw += self._twist.angular_z_rps * dt
            self._x += (
                self._twist.linear_x_mps * math.cos(self._yaw)
                - self._twist.linear_y_mps * math.sin(self._yaw)
            ) * dt
            self._y += (
                self._twist.linear_x_mps * math.sin(self._yaw)
                + self._twist.linear_y_mps * math.cos(self._yaw)
            ) * dt
            if not self._freeze_state:
                self._last_state_at = now
            return RobotState(
                motion_state=self._motion_state,
                connected=self._connected,
                received_at_s=self._last_state_at,
                position_m=(self._x, self._y, 0.0),
                orientation_xyzw=(0.0, 0.0, math.sin(self._yaw / 2.0), math.cos(self._yaw / 2.0)),
                linear_velocity_body_mps=(
                    self._twist.linear_x_mps,
                    self._twist.linear_y_mps,
                    0.0,
                ),
                angular_velocity_body_rps=(0.0, 0.0, self._twist.angular_z_rps),
                joint_position_rad=(0.0, 0.5, -1.0) * 4,
                joint_velocity_rad_s=(0.0,) * 12,
                joint_torque_nm=(0.0,) * 12,
            )

    def request_stand_up(self):
        with self._lock:
            self._motion_state = 6
            return True

    def request_sit_down(self):
        with self._lock:
            self._motion_state = 0
            self._twist = ZERO_TWIST
            return True

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self._connected = connected

    def set_state_frozen(self, frozen: bool) -> None:
        with self._lock:
            self._freeze_state = frozen

    @property
    def current_twist(self) -> PlanarTwist:
        with self._lock:
            return self._twist
