import math
import unittest
from dataclasses import replace
from bpx_quadruped.fake_backend import FakeBpxBackend
from bpx_quadruped.model import ControllerConfig, PlanarTwist, VelocityCommand
from bpx_quadruped.motion_runtime import MotionRuntime, MoveRequest
from bpx_quadruped.config import ProviderConfig


class Clock:
    now = 100.
    def __call__(self): return self.now
    def sleep(self, dt): self.now += dt


class WalkBackend(FakeBpxBackend):
    """Model the documented <0.2 deadzone and actual 10Hz odom cadence."""
    cached = None
    def apply(self, command):
        if isinstance(command, VelocityCommand):
            t = command.twist
            if math.sqrt(t.linear_x_mps**2+t.linear_y_mps**2+t.angular_z_rps**2)<.2:
                command = VelocityCommand(PlanarTwist())
        super().apply(command)

    def read_state(self):
        fresh = super().read_state()
        if self.cached is None or self._clock()-self.cached.received_at_s >= .099:
            self.cached = fresh
        return self.cached


class WalkTest(unittest.TestCase):
    def make_runtime(self, minimum=.25):
        clock = Clock()
        backend = WalkBackend(clock)
        runtime = MotionRuntime(backend, ControllerConfig(allow_motion=True),
            threaded=False,clock=clock,sleep=clock.sleep,
            move_min_linear_mps=minimum,move_min_angular_rps=minimum)
        runtime.activate()
        self.addCleanup(runtime.shutdown)
        self.assertTrue(runtime.arm().accepted)
        return runtime,backend,clock

    def test_old_controller_reproduces_partial_move_then_stall(self):
        runtime,backend,_ = self.make_runtime(0)
        result = runtime.execute_move(MoveRequest(forward_m=.2,duration_sec=10))
        self.assertFalse(result.accepted)
        self.assertIn('stalled',result.reason)
        self.assertGreater(runtime.last_move['progress_m'],.04)
        self.assertLess(runtime.last_move['progress_m'],.12)

    def test_signed_distance_and_rotation_reach_with_deadzone(self):
        for request in (MoveRequest(forward_m=.2),MoveRequest(forward_m=-.2),
                        MoveRequest(rotate_deg=15),MoveRequest(rotate_deg=-15),
                        MoveRequest(rotate_deg=360),MoveRequest(rotate_deg=-360)):
            with self.subTest(request=request):
                runtime,backend,_ = self.make_runtime()
                result = runtime.execute_move(request)
                self.assertTrue(result.accepted,result.reason)
                self.assertTrue(backend.current_twist.is_zero())
                self.assertEqual(runtime.status()['safety'],'disarmed')

    def test_timed_and_twist_low_velocity_not_amplified(self):
        runtime,backend,clock = self.make_runtime()
        runtime.execute_move(MoveRequest(linear_x=.1,duration_sec=1))
        self.assertAlmostEqual(backend._x,0)
        runtime.arm()
        runtime.submit_twist(PlanarTwist(.1))
        for _ in range(10):
            clock.sleep(.02)
            runtime.cycle()
        self.assertLessEqual(runtime.controller.snapshot().last_output.linear_x_mps,.1)

    def test_within_tolerance_does_not_create_nonzero_command(self):
        runtime,backend,_ = self.make_runtime()
        self.assertTrue(runtime.execute_move(MoveRequest(forward_m=.01)).accepted)
        self.assertAlmostEqual(backend._x,0)
        self.assertTrue(all(not isinstance(c,VelocityCommand) or c.twist.is_zero() for c in backend.commands))

    def test_cancel_during_compensated_move_stops(self):
        runtime,backend,clock = self.make_runtime()
        start=clock()
        result=runtime.execute_move(MoveRequest(forward_m=1),cancelled=lambda:clock()-start>.7)
        self.assertFalse(result.accepted)
        self.assertTrue(backend.current_twist.is_zero())
        self.assertEqual(runtime.status()['safety'],'disarmed')

    def test_overshoot_stops_without_reverse(self):
        runtime,backend,clock = self.make_runtime()
        original=runtime.sleep
        def sleep(dt):
            original(dt)
            backend._x=.24
            backend.cached=None
            runtime.previous=None  # isolate crossing guard from separate jump guard
        runtime.sleep=sleep
        result=runtime.execute_move(MoveRequest(forward_m=.2))
        self.assertFalse(result.accepted)
        self.assertIn('overshoot',result.reason)
        self.assertTrue(backend.current_twist.is_zero())
        self.assertFalse(any(isinstance(c,VelocityCommand) and c.twist.linear_x_mps<0 for c in backend.commands))

    def test_bad_minimum_rejected(self):
        for value in (0,.19,.31,float('nan'),True):
            with self.assertRaises(ValueError):
                ProviderConfig.from_mapping({'move_min_linear_mps':value})
        with self.assertRaises(ValueError):
            MotionRuntime(FakeBpxBackend(),ControllerConfig(max_linear_x_mps=.2),move_min_linear_mps=.25)
