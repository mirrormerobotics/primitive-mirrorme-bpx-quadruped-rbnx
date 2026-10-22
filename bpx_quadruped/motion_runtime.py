"""Supervised motion, bounded moves, cancellation and local arm leases.

The command loop runs independently of telemetry publication and RPC threads.
No activation, reconnect, remote RPC or ROS message can grant a local lease.
"""

from dataclasses import dataclass, asdict
import json
import uuid
import math
import logging
import threading
import time

from .controller import QuadrupedController
from .model import CommandDecision, LifecycleState, PlanarTwist, SafetyState
from .sdk_posture import PostureCycleConfig, command_until_state
from .telemetry import state_for_publication, normalize_robot_state


def yaw(state):
    x, y, z, w = state.orientation_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class MoveRequest:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0
    duration_sec: float = 0.0
    forward_m: float = 0.0
    rotate_deg: float = 0.0

    def validate(self):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
               for v in vars(self).values()):
            raise ValueError("move fields must be finite numbers")
        if self.linear_z or self.angular_x or self.angular_y:
            raise ValueError("only planar movement is supported")
        if not 0 <= self.duration_sec <= 30:
            raise ValueError("duration must be 0..30 seconds")
        if abs(self.forward_m) > 1 or abs(self.rotate_deg) > 360:
            raise ValueError("move exceeds 1 metre or 360 degrees")
        if abs(self.linear_x) > .3 or abs(self.linear_y) > .2 or abs(self.angular_z) > .5:
            raise ValueError("move exceeds motion-test velocity bounds")


class MotionRuntime:
    supports_twist = True
    supports_posture = False
    supports_move = True

    def __init__(self, backend, config, *, command_rate_hz=50, lease_s=30,
                 clock=time.monotonic, sleep=time.sleep, threaded=True,
                 move_min_linear_mps=0.0, move_min_angular_rps=0.0):
        if not 10 <= command_rate_hz <= 100 or not 1 <= lease_s <= 120:
            raise ValueError("motion rate must be 10..100 Hz and local lease 1..120 s")
        if config.max_linear_x_mps > .3 or config.max_linear_y_mps > .2 or config.max_angular_z_rps > .5:
            raise ValueError("motion-test velocity limits cannot exceed 0.3/0.2/0.5")
        self.backend = backend
        for value, limit in ((move_min_linear_mps, min(.3, config.max_linear_x_mps)),
                             (move_min_angular_rps, min(.4, config.max_angular_z_rps))):
            if isinstance(value, bool) or not math.isfinite(value) or not 0 <= value <= limit:
                raise ValueError("bounded move minimum exceeds configured velocity limit")
        self.move_min_linear_mps = move_min_linear_mps
        self.move_min_angular_rps = move_min_angular_rps
        self.controller = QuadrupedController(backend, config, clock)
        self.config = config
        self.clock, self.sleep = clock, sleep
        self.period, self.lease_s = 1.0 / command_rate_hz, lease_s
        self.threaded = threaded
        self.lock = threading.RLock()
        self.operation = threading.Lock()
        self.cancelled = threading.Event()
        self.stopping = threading.Event()
        self.worker = None
        self.lease_until = 0.0
        self.state = None
        self.previous = None
        self.last_fault = None
        self.last_move = None
        self.released = False
        self.cancel_reason = ""
        self.last_event_at = 0.0

    def _event(self, name, **values):
        try:
            logging.getLogger(__name__).info("motion_event %s", json.dumps(
                {"event":name,"monotonic_s":self.clock(), **values}, allow_nan=False))
        except (TypeError, ValueError):
            logging.getLogger(__name__).error("motion_event %s contains invalid diagnostic values", name)

    def _healthy(self, state):
        if not state.connected:
            raise RuntimeError("feedback disconnected")
        try:
            state = normalize_robot_state(state)
        except (ValueError, TypeError) as exc:
            raise RuntimeError("feedback invalid: " + str(exc)) from exc
        age = self.clock() - state.received_at_s
        if age < 0:
            raise RuntimeError("feedback timestamp is in the future")
        if age > self.config.state_timeout_s:
            raise RuntimeError("feedback stale: age={:.3f}s limit={:.3f}s odometry_ms={}".format(
                age, self.config.state_timeout_s, state.vendor_timestamps.odometry_ms))
        return state

    def activate(self):
        with self.lock:
            if self.released:
                raise RuntimeError("control released; restart provider to acquire a new SDK session")
            if self.controller.snapshot().lifecycle is LifecycleState.ACTIVE:
                return
            try:
                self.controller.activate()
                self.state = self.backend.read_state()
            except Exception:
                self.backend.stop()
                raise
            self.previous = None
            self.lease_until = 0
            self.stopping.clear()
            if self.threaded:
                self.worker = threading.Thread(target=self._loop, name="bpx-motion-watchdog", daemon=True)
                self.worker.start()

    def _loop(self):
        while not self.stopping.wait(self.period):
            self.cycle()

    def _disable(self):
        method = getattr(self.backend, "disable_motion", None)
        if method:
            method()

    def _fault(self, reason):
        self.lease_until = 0
        self.cancelled.set()
        self.cancel_reason = "motion fault: " + reason
        if self.controller.snapshot().safety is not SafetyState.FAULT or self.last_fault is None:
            self.last_fault = dict(reason=reason, monotonic_s=self.clock(),
                                   feedback=self._feedback())
            self._event("fault", **self.last_fault)
        else:
            reason = self.controller.snapshot().fault_reason
        try:
            self.controller.trip_fault(reason)
        finally:
            self._disable()

    def cycle(self):
        """Validate feedback before issuing this cycle's command; latch all errors."""
        with self.lock:
            if self.controller.snapshot().lifecycle is not LifecycleState.ACTIVE:
                return
            try:
                state = self.backend.read_state()
                self.state = state
                healthy = self._healthy(state)
                state = self.state = healthy
                old = self.previous
                if old is not None:
                    old_ts, new_ts = old.vendor_timestamps.odometry_ms, state.vendor_timestamps.odometry_ms
                    if old_ts is not None and new_ts is not None and new_ts < old_ts:
                        raise RuntimeError("odometry timestamp reset")
                    # Conservative jump guard, not a substitute for physical calibration.
                    dt = max(0, state.received_at_s - old.received_at_s)
                    if (math.hypot(state.position_m[0] - old.position_m[0], state.position_m[1] - old.position_m[1]) > .1 + dt
                            or abs(wrap(yaw(state) - yaw(old))) > .2 + 2 * dt):
                        raise RuntimeError("odometry discontinuity")
                self.previous = state
                if self.controller.snapshot().safety is SafetyState.ARMED:
                    if state.motion_state not in (None, 6):
                        raise RuntimeError("robot left standing motion state")
                    if self.clock() >= self.lease_until:
                        self.disarm("local arm lease expired")
                self.controller.tick(state)
                snapshot = self.controller.snapshot()
                if snapshot.safety is SafetyState.FAULT:
                    self._fault(snapshot.fault_reason)
            except Exception as exc:
                try:
                    self._fault(str(exc))
                except Exception as stop_error:
                    # The fault was latched before attempting best-effort SDK stop.
                    # Subsequent cycles keep retrying; the robot watchdog is still required.
                    logging.getLogger(__name__).error("fault stop failed: %s", stop_error)

    def tick(self):
        with self.lock:
            if self.state is None:
                raise RuntimeError("motion runtime has no state")
            return self.state

    def arm(self):
        """Called only by the local Unix control endpoint, never by a remote RPC."""
        with self.lock:
            if self.operation.locked():
                return CommandDecision(False, "operation in progress")
            if self.controller.snapshot().safety is SafetyState.ARMED:
                return CommandDecision(False, "disarm before issuing a new local lease")
            decision = self.controller.arm()
            if not decision.accepted:
                return decision
            try:
                prepare = getattr(self.backend, "prepare_motion", None)
                if prepare:
                    prepare()
                self.lease_until = self.clock() + self.lease_s
                self.cancelled.clear()
                self.cancel_reason = ""
                self._event("armed", lease_s=self.lease_s)
                return CommandDecision(True, "locally armed for {} seconds".format(self.lease_s))
            except Exception as exc:
                self._fault(str(exc))
                return CommandDecision(False, str(exc))

    def disarm(self, reason="local disarm"):
        with self.lock:
            self.cancelled.set()
            self.cancel_reason = reason
            self.lease_until = 0
            try:
                try:
                    self.controller.disarm(reason)
                finally:
                    self._disable()
            except Exception as exc:
                # A revoked lease is not evidence that the SDK accepted stop.
                # Latch before retrying so a caller cannot immediately re-arm.
                failure = "stop failed during {}: {}".format(reason, exc)
                try:
                    self._fault(failure)
                except Exception as retry_error:
                    logging.getLogger(__name__).error("fault stop retry failed: %s", retry_error)
                if self.last_move is not None and reason == "move finished":
                    self._move_result(False, failure)
                raise

    def clear_fault(self):
        with self.lock:
            result = self.controller.clear_fault()
            if result.accepted:
                self.previous = None
                self._event("fault_cleared")
            return result

    def release(self):
        """Close the control session; cannot be re-acquired through a remote call."""
        with self.lock:
            self.released = True
        self.shutdown()
        self._event("control_released")
        return CommandDecision(True, "SDK session closed; restart provider before SDK control")

    def submit_twist(self, command):
        with self.lock:
            if self.operation.locked():
                return CommandDecision(False, "move/posture owns the command channel")
            if self.clock() >= self.lease_until:
                return CommandDecision(False, "local arm required")
            return self.controller.submit_twist(command)

    def set_posture(self, name):
        return CommandDecision(False, "use local stand/sit in the motion-test profile")

    def local_posture(self, name):
        if name not in ("stand", "sit"):
            return CommandDecision(False, "unsupported posture")
        if not self.operation.acquire(False):
            return CommandDecision(False, "operation already in progress")
        try:
            with self.lock:
                if self.controller.snapshot().lifecycle is not LifecycleState.ACTIVE:
                    return CommandDecision(False, "runtime is not active")
                if self.controller.snapshot().safety is SafetyState.FAULT:
                    return CommandDecision(False, "clear fault before posture")
                self.disarm("local posture")
                self.cancelled.clear()
            request = getattr(self.backend, "request_stand_up" if name == "stand" else "request_sit_down", None)
            if request is None:
                return CommandDecision(False, "backend has no posture API")
            command_until_state(self.backend, PostureCycleConfig(), target_state=6 if name == "stand" else 0,
                                target_name=name, command=request, command_name=name, timeout_s=15,
                                clock=self.clock, sleep=self.sleep, stop_requested=self.cancelled.is_set,
                                report=lambda text: None)
            return CommandDecision(True, name + " confirmed")
        except Exception as exc:
            with self.lock:
                self._fault(str(exc))
            return CommandDecision(False, str(exc))
        finally:
            self.operation.release()

    def execute_move(self, request, cancelled=lambda: False):
        """Use initial body heading for distance and unwrapped yaw for rotation.

        Contract priority is forward_m, rotate_deg, then timed planar velocity.
        All paths after admission disarm in finally; RPC completion never leaves
        an old target running. External Twist cannot overwrite an active move.
        """
        try:
            request.validate()
        except ValueError as exc:
            return CommandDecision(False, str(exc))
        if not self.operation.acquire(False):
            return CommandDecision(False, "operation already in progress")
        admitted = False
        try:
            with self.lock:
                if self.controller.snapshot().safety is not SafetyState.ARMED or self.clock() >= self.lease_until:
                    return CommandDecision(False, "local arm required")
                admitted = True
                self.last_move = None
                self.cancelled.clear()
                start = self.tick()
                start = self._healthy(start)
                initial_yaw = last_yaw = yaw(start)
                progress_yaw = 0.0
                start_time = self.clock()
                duration = request.duration_sec or (30.0 if request.forward_m or request.rotate_deg else 1.0)
                deadline = start_time + duration
                self.last_move = dict(id=uuid.uuid4().hex, request=asdict(request),
                                      started_at_s=start_time, deadline_s=deadline,
                                      start_position_m=list(start.position_m), start_yaw_rad=initial_yaw,
                                      phase="running", progress_m=0.0, progress_deg=0.0,
                                      target_velocity=asdict(PlanarTwist()))
                self._event("move_start", **self.last_move)
                self.last_event_at = 0
                best_progress = 0.0
                progress_at = start_time
            while True:
                with self.lock:
                    snapshot = self.controller.snapshot()
                    if snapshot.safety is SafetyState.FAULT:
                        return self._move_result(False, "motion fault: " + snapshot.fault_reason)
                    if cancelled() or self.cancelled.is_set():
                        return self._move_result(False, "move cancelled: " + (self.cancel_reason or "RPC cancelled/deadline"))
                    if self.controller.snapshot().safety is not SafetyState.ARMED or self.clock() >= self.lease_until:
                        return self._move_result(False, "local lease expired or motion disarmed")
                    state = self.tick()
                    state = self._healthy(state)
                    dx, dy = state.position_m[0] - start.position_m[0], state.position_m[1] - start.position_m[1]
                    current_yaw = yaw(state)
                    progress_yaw += wrap(current_yaw - last_yaw)
                    last_yaw = current_yaw
                    progress = dx * math.cos(initial_yaw) + dy * math.sin(initial_yaw)
                    self.last_move.update(progress_m=progress, progress_deg=math.degrees(progress_yaw),
                                          remaining_m=request.forward_m-progress,
                                          remaining_deg=request.rotate_deg-math.degrees(progress_yaw))
                    # Detect the observed "moves then stalls" failure without
                    # silently raising velocity or relaxing target tolerance.
                    if request.forward_m or request.rotate_deg:
                        signed_progress = (progress * math.copysign(1, request.forward_m)
                                           if request.forward_m else progress_yaw * math.copysign(1, request.rotate_deg))
                        threshold = .003 if request.forward_m else .008
                        if signed_progress >= best_progress + threshold:
                            best_progress, progress_at = signed_progress, self.clock()
                        self.last_move['no_progress_s'] = self.clock()-progress_at
                        if self.clock()-progress_at >= 3:
                            return self._move_result(False, "move stalled: insufficient odometry progress for 3s")
                    if (request.forward_m or request.rotate_deg) and self.clock() >= deadline:
                        return self._move_result(False, "move timeout")
                    if request.forward_m:
                        progress = dx * math.cos(initial_yaw) + dy * math.sin(initial_yaw)
                        cross = -dx * math.sin(initial_yaw) + dy * math.cos(initial_yaw)
                        remaining = request.forward_m - progress
                        if abs(cross) > .15 or abs(wrap(current_yaw - initial_yaw)) > .3 or progress * math.copysign(1, request.forward_m) < -.05:
                            self._fault("distance move deviated from initial body axis")
                            return self._move_result(False, "distance deviation")
                        if abs(remaining) <= .02:
                            return self._move_result(True, "distance reached")
                        if remaining * request.forward_m < 0:
                            return self._move_result(False, "distance overshoot; no automatic reversal")
                        speed = max(self.move_min_linear_mps, min(.2, abs(remaining)*1.5))
                        command = PlanarTwist(math.copysign(speed, remaining))
                    elif request.rotate_deg:
                        remaining = math.radians(request.rotate_deg) - progress_yaw
                        if progress_yaw * math.copysign(1, request.rotate_deg) < -.1:
                            self._fault("rotation reversed")
                            return self._move_result(False, "rotation reversed")
                        if abs(remaining) <= .03:
                            return self._move_result(True, "angle reached")
                        if remaining * request.rotate_deg < 0:
                            return self._move_result(False, "angle overshoot; no automatic reversal")
                        speed = max(self.move_min_angular_rps, min(.4, abs(remaining)*1.5))
                        command = PlanarTwist(angular_z_rps=math.copysign(speed, remaining))
                    else:
                        if self.clock() >= deadline:
                            return self._move_result(True, "duration completed")
                        command = PlanarTwist(request.linear_x, request.linear_y, request.angular_z)
                    if self.clock() >= deadline:
                        return self._move_result(False, "move timeout")
                    # Rotation is governed by accumulated yaw, not translation.
                    # Keep displacement bounds for distance and timed moves.
                    rotation_only = not request.forward_m and bool(request.rotate_deg)
                    if not rotation_only and math.hypot(dx, dy) > 1.15:
                        self._fault("move displacement bound exceeded")
                        return self._move_result(False, "move displacement bound exceeded")
                    self.last_move["target_velocity"] = asdict(command)
                    decision = self.controller.submit_twist(command)
                    if not decision.accepted:
                        return self._move_result(False, decision.reason)
                    if self.clock() - self.last_event_at >= .2:
                        self.last_event_at = self.clock()
                        self._event("move_progress", **self.status())
                self.sleep(self.period)
                if not self.threaded:
                    self.cycle()
        except Exception as exc:
            with self.lock:
                self._fault(str(exc))
            return self._move_result(False, str(exc))
        finally:
            try:
                if admitted:
                    self.disarm("move finished")
            finally:
                self.operation.release()

    def _move_result(self, accepted, reason):
        if self.last_move is not None:
            self.last_move.update(phase="finished", success=accepted, reason=reason, ended_at_s=self.clock())
            self._event("move_result", **self.last_move)
            if not accepted and any(word in reason for word in ("timeout", "fault", "stalled")):
                reason += "; progress={:.4f}m/{:.2f}deg remaining={:.4f}m/{:.2f}deg".format(
                    self.last_move['progress_m'], self.last_move['progress_deg'],
                    self.last_move.get('remaining_m',0), self.last_move.get('remaining_deg',0))
        return CommandDecision(accepted, reason)

    def _feedback(self):
        if self.state is None: return None
        state = self.state
        try:
            age = self.clock()-state.received_at_s
            return dict(connected=state.connected, age_s=age if math.isfinite(age) else None,
                        vendor_timestamps=asdict(state.vendor_timestamps),
                        motion_state=state.motion_state, gait=state.motion_gait)
        except (TypeError, ValueError, AttributeError):
            return {"invalid": True}

    def status(self):
        with self.lock:
            snapshot = self.controller.snapshot()
            return {"lifecycle": snapshot.lifecycle.value, "safety": snapshot.safety.value,
                    "fault": snapshot.fault_reason, "lease_remaining_s": max(0, self.lease_until - self.clock()),
                    "busy": self.operation.locked(), "released": self.released,
                    "health": "released" if self.released else snapshot.safety.value,
                    "feedback": self._feedback(), "last_fault": self.last_fault,
                    "last_move": self.last_move,
                    "bounded_move_minimum": {"linear_mps":self.move_min_linear_mps,
                                             "angular_rps":self.move_min_angular_rps},
                    "adapter_output": asdict(snapshot.last_output)}

    def deactivate(self):
        self.stopping.set()
        self.cancelled.set()
        with self.lock:
            try:
                self.disarm("deactivate")
            finally:
                self.controller.deactivate()
        if self.worker is not None:
            self.worker.join(timeout=2)

    def shutdown(self):
        try:
            self.deactivate()
        finally:
            self.controller.shutdown()
