"""Unreleased, single-owner SDK velocity adapter for supervised acceptance."""

from .model import StopCommand, VelocityCommand
from .sdk_posture import SdkPostureSession, MOTION_STATE_MOTION


class SdkMotionBackend(SdkPostureSession):
    @property
    def name(self):
        return "bpx-sdk-motion-test"

    @staticmethod
    def _require_posture_api(control):
        SdkPostureSession._require_posture_api(control)
        for name in ("setWalk", "setVelocity"):
            if not callable(getattr(control, name, None)):
                raise RuntimeError("MotionLevelControl is missing: " + name)

    def prepare_motion(self):
        """Only local arm selects Walk and enables a previously zeroed command."""
        with self._sdk_lock:
            if self.read_state().motion_state != MOTION_STATE_MOTION:
                raise RuntimeError("stand must be confirmed before local arm")
            control = self._require_started_control()
            if not control.setVelocity(0.0, 0.0, 0.0):
                raise RuntimeError("SDK rejected zero preamble")
            # SDK 1.0.8 declares setWalk() -> None (not an acknowledgement).
            control.setWalk()
            control.setVelocityControlFlag(True)
            if not control.setVelocity(0.0, 0.0, 0.0):
                raise RuntimeError("SDK rejected enabled zero preamble")

    def apply(self, command):
        with self._sdk_lock:
            if isinstance(command, StopCommand) and self._source is None:
                return
            control = self._require_started_control()
            if isinstance(command, StopCommand):
                values = (0.0, 0.0, 0.0)
            elif isinstance(command, VelocityCommand):
                twist = command.twist
                if not twist.is_finite():
                    raise ValueError("non-finite SDK velocity")
                if abs(twist.linear_x_mps) > 0.3 or abs(twist.linear_y_mps) > 0.2 or abs(twist.angular_z_rps) > 0.5:
                    raise ValueError("SDK motion-test hard velocity limit exceeded")
                values = (twist.linear_x_mps, twist.linear_y_mps, twist.angular_z_rps)
            else:
                raise ValueError("unsupported SDK motion command")
            if not control.setVelocity(*values):
                raise RuntimeError("SDK rejected velocity")

    def disable_motion(self):
        with self._sdk_lock:
            if self._control is not None:
                try:
                    if not self._control.setVelocity(0.0, 0.0, 0.0):
                        raise RuntimeError("SDK rejected stop")
                    # The SDK sends periodically; leave zero queued for at least
                    # three send intervals before disabling its velocity flag.
                    self._sleep(max(.1, 3.0 / self._config.motion_command_rate_hz))
                finally:
                    self._control.setVelocityControlFlag(False)

    def stop(self):
        try:
            self.disable_motion()
        finally:
            super().stop()
