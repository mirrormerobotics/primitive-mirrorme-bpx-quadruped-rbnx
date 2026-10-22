from dataclasses import replace
import math
import socket
import json
import os
import tempfile
import threading
import unittest

from bpx_quadruped.config import ProviderConfig
from bpx_quadruped.fake_backend import FakeBpxBackend
from bpx_quadruped.local_control import LocalControl
from bpx_quadruped.model import ControllerConfig, PlanarTwist, SafetyState, VendorTimestamps
from bpx_quadruped.motion_runtime import MotionRuntime, MoveRequest
from bpx_quadruped.sdk_motion import SdkMotionBackend
from bpx_quadruped.sdk_posture import SdkPostureConfig


class Clock:
    def __init__(self): self.now = 100.0
    def __call__(self): return self.now
    def sleep(self, dt): self.now += dt


class MotionTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.backend = FakeBpxBackend(self.clock)
        self.runtime = MotionRuntime(self.backend, ControllerConfig(allow_motion=True),
                                     clock=self.clock, sleep=self.clock.sleep, threaded=False)
        self.runtime.activate()
        self.addCleanup(self.runtime.shutdown)

    def arm(self):
        self.assertTrue(self.runtime.arm().accepted)

    def test_activation_remote_move_and_twist_cannot_arm(self):
        self.assertEqual(self.runtime.status()["safety"], "disarmed")
        self.assertFalse(self.runtime.submit_twist(PlanarTwist(.1)).accepted)
        self.assertFalse(self.runtime.execute_move(MoveRequest(forward_m=.1)).accepted)

    def test_clock_advancing_during_read_is_not_future_state(self):
        read = self.backend.read_state
        def advancing_read():
            self.clock.sleep(.001)
            return read()
        self.backend.read_state = advancing_read
        self.arm()
        self.runtime.cycle()
        self.assertEqual(self.runtime.status()["safety"], "armed")
        self.runtime.controller.trip_fault("test")
        self.assertTrue(self.runtime.clear_fault().accepted)

    def test_stop_failure_still_revokes_target_and_arm(self):
        self.arm()
        self.runtime.submit_twist(PlanarTwist(.1))
        apply = self.backend.apply
        self.backend.apply = lambda command: (_ for _ in ()).throw(RuntimeError("stop failed"))
        try:
            with self.assertRaises(RuntimeError):
                self.runtime.disarm()
            self.assertEqual(self.runtime.status()["safety"], "fault")
            self.assertIn("stop failed", self.runtime.status()["fault"])
            self.assertFalse(self.runtime.arm().accepted)
        finally:
            self.backend.apply = apply
        self.assertTrue(self.runtime.clear_fault().accepted)
        self.arm()
        self.runtime.cycle()
        self.assertTrue(self.backend.current_twist.is_zero())
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_move_stop_rejection_overrides_success_and_requires_fault_clear(self):
        from bpx_quadruped.model import StopCommand
        self.arm()
        apply = self.backend.apply
        def reject_final_stop(command):
            if isinstance(command, StopCommand):
                raise RuntimeError("SDK rejected stop")
            return apply(command)
        self.backend.apply = reject_final_stop
        try:
            with self.assertRaisesRegex(RuntimeError, "SDK rejected stop"):
                self.runtime.execute_move(MoveRequest(linear_x=.1, duration_sec=.2))
            status = self.runtime.status()
            self.assertEqual(status['safety'], 'fault')
            self.assertFalse(status['last_move']['success'])
            self.assertIn('stop failed', status['last_move']['reason'])
            self.assertFalse(status['busy'])
            self.assertEqual(status['lease_remaining_s'], 0)
            self.assertFalse(self.runtime.arm().accepted)
        finally:
            self.backend.apply = apply
        self.assertTrue(self.runtime.clear_fault().accepted)
        self.assertEqual(self.runtime.status()['safety'], 'disarmed')

    def test_disable_failure_latches_even_when_zero_command_succeeds(self):
        self.arm()
        def fail_disable():
            raise RuntimeError('velocity flag disable failed')
        self.backend.disable_motion = fail_disable
        try:
            with self.assertRaisesRegex(RuntimeError, 'velocity flag disable failed'):
                self.runtime.disarm()
            self.assertEqual(self.runtime.status()['safety'], 'fault')
            self.assertFalse(self.runtime.arm().accepted)
        finally:
            del self.backend.disable_motion

    def test_distance_uses_initial_body_heading_and_stops(self):
        self.backend._yaw = math.pi / 2
        self.runtime.cycle()
        self.arm()
        result = self.runtime.execute_move(MoveRequest(forward_m=.2))
        self.assertTrue(result.accepted, result.reason)
        state = self.runtime.tick()
        self.assertLess(abs(state.position_m[0]), .01)
        self.assertAlmostEqual(state.position_m[1], .2, delta=.025)
        self.assertTrue(self.backend.current_twist.is_zero())
        self.assertEqual(self.runtime.status()["safety"], "disarmed")

    def test_signed_rotation_wrap_and_full_turn(self):
        for degrees in (-360, 360):
            self.runtime.previous = None
            self.runtime.cycle()
            self.arm()
            result = self.runtime.execute_move(MoveRequest(rotate_deg=degrees, duration_sec=30))
            self.assertTrue(result.accepted, result.reason)
            self.assertTrue(self.backend.current_twist.is_zero())

    def test_timed_velocity_priority_and_watchdog(self):
        self.arm()
        result = self.runtime.execute_move(MoveRequest(linear_y=.1, duration_sec=.3))
        self.assertTrue(result.accepted)
        self.assertTrue(self.backend.current_twist.is_zero())
        self.arm()
        self.runtime.submit_twist(PlanarTwist(.1))
        self.clock.sleep(.1)
        self.runtime.cycle()
        self.assertFalse(self.backend.current_twist.is_zero())
        self.clock.sleep(.3)
        self.runtime.cycle()
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_full_rotation_does_not_stop_for_translation(self):
        for degrees in (360, -360):
            with self.subTest(degrees=degrees):
                read = self.backend.read_state
                start = self.clock()
                def drifting_state():
                    state = read()
                    return replace(state, position_m=(state.position_m[0] + .1 * (self.clock()-start),
                                                       state.position_m[1], state.position_m[2]))
                self.backend.read_state = drifting_state
                self.runtime.previous = None
                self.runtime.cycle()
                try:
                    self.arm()
                    result = self.runtime.execute_move(MoveRequest(rotate_deg=degrees, duration_sec=30))
                    self.assertTrue(result.accepted, result.reason)
                    self.assertGreater(abs(self.runtime.last_move['progress_m']), 1.15)
                    self.assertTrue(self.backend.current_twist.is_zero())
                    self.assertEqual(self.runtime.status()['safety'], 'disarmed')
                finally:
                    self.backend.read_state = read

    def test_cancel_and_timeout_always_zero(self):
        self.arm()
        start = self.clock()
        result = self.runtime.execute_move(MoveRequest(forward_m=1), cancelled=lambda: self.clock() - start > .2)
        self.assertFalse(result.accepted)
        self.assertIn("cancelled", result.reason)
        self.assertTrue(self.backend.current_twist.is_zero())
        self.arm()
        result = self.runtime.execute_move(MoveRequest(forward_m=1, duration_sec=.1))
        self.assertFalse(result.accepted)
        self.assertIn("timeout", result.reason)
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_concurrent_twist_and_move_rejected_during_move(self):
        self.arm()
        original = self.runtime.sleep
        decisions = []
        def sleep(dt):
            decisions.append(self.runtime.submit_twist(PlanarTwist(.3)))
            decisions.append(self.runtime.execute_move(MoveRequest(forward_m=.1)))
            original(dt)
        self.runtime.sleep = sleep
        self.assertTrue(self.runtime.execute_move(MoveRequest(duration_sec=.1)).accepted)
        self.assertTrue(decisions)
        self.assertTrue(all(not d.accepted for d in decisions))

    def test_stale_disconnect_and_reconnect_require_clear_and_new_local_arm(self):
        for freeze in (True, False):
            self.arm()
            if freeze:
                self.backend.set_state_frozen(True)
                self.clock.sleep(1)
            else:
                self.backend.set_connected(False)
            self.runtime.cycle()
            self.assertEqual(self.runtime.status()["safety"], "fault")
            self.assertTrue(self.backend.current_twist.is_zero())
            self.backend.set_connected(True)
            self.backend.set_state_frozen(False)
            self.runtime.cycle()
            self.assertFalse(self.runtime.arm().accepted)
            self.assertTrue(self.runtime.clear_fault().accepted)
            self.assertFalse(self.runtime.submit_twist(PlanarTwist(.1)).accepted)

    def test_lease_and_lifecycle_do_not_replay_command(self):
        self.arm()
        self.runtime.submit_twist(PlanarTwist(.2))
        self.clock.sleep(31)
        self.runtime.cycle()
        self.assertEqual(self.runtime.status()["safety"], "disarmed")
        self.assertFalse(self.runtime.submit_twist(PlanarTwist(.1)).accepted)
        self.runtime.deactivate()
        self.runtime.activate()
        self.assertEqual(self.runtime.status()["safety"], "disarmed")

    def test_invalid_requests_have_no_output(self):
        self.arm()
        for request in (MoveRequest(forward_m=2), MoveRequest(rotate_deg=400),
                        MoveRequest(duration_sec=-1), MoveRequest(linear_z=.1),
                        MoveRequest(linear_x=float("nan")), MoveRequest(angular_z=2)):
            self.assertFalse(self.runtime.execute_move(request).accepted)
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_odom_jump_latches_fault(self):
        self.arm()
        self.runtime.cycle()
        self.backend._x += 1
        self.runtime.cycle()
        self.assertIn("discontinuity", self.runtime.status()["fault"])
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_timestamp_reset_latches_fault(self):
        self.arm()
        read = self.backend.read_state
        stamp = [100]
        self.backend.read_state = lambda: replace(read(), vendor_timestamps=VendorTimestamps(odometry_ms=stamp[0]))
        self.runtime.cycle()
        stamp[0] = 1
        self.runtime.cycle()
        self.assertIn("timestamp reset", self.runtime.status()["fault"])

    def test_local_socket_is_private_and_rejects_unknown_actions(self):
        with tempfile.TemporaryDirectory() as temp:
            path = temp + "/control.sock"
            server = LocalControl(self.runtime, path)
            try:
                self.assertEqual(os.stat(path).st_mode & 0o777, 0o600)
                for action, accepted in (("unknown", False), ("arm", True), ("disarm", True)):
                    with socket.socket(socket.AF_UNIX) as client:
                        client.connect(path)
                        client.sendall((action + "\n").encode())
                        response = json.loads(client.recv(4096))
                    self.assertEqual(response["accepted"], accepted)
            finally:
                server.close()

    def test_config_cannot_auto_arm_motion_or_mix_posture(self):
        for extra in ({"auto_arm_fake": True}, {"enable_posture_service": True}, {"backend": "replay"}):
            with self.assertRaises(ValueError):
                ProviderConfig.from_mapping({"backend": "fake", "allow_motion": True,
                                             "enable_motion_service": True, **extra})

    def test_fault_diagnostic_preserves_first_reason_and_vendor_timestamp(self):
        self.arm()
        read = self.backend.read_state
        self.backend.read_state = lambda: replace(read(), received_at_s=self.clock()-1,
                              vendor_timestamps=VendorTimestamps(odometry_ms=123))
        self.runtime.cycle()
        status=self.runtime.status()
        self.assertIn('feedback stale',status['fault'])
        self.assertEqual(status['last_fault']['feedback']['vendor_timestamps']['odometry_ms'],123)
        self.backend.read_state=lambda: replace(read(),connected=False)
        self.runtime.cycle()
        self.assertEqual(self.runtime.status()['fault'],status['fault'])

    def test_fault_is_reported_as_fault_not_generic_cancellation(self):
        self.arm()
        start=self.clock()
        def cancelled():
            if self.clock()-start>.1: self.runtime._fault('feedback disconnected')
            return False
        result=self.runtime.execute_move(MoveRequest(forward_m=.2),cancelled)
        self.assertIn('fault',result.reason)
        self.assertIn('feedback disconnected',result.reason)
        self.assertFalse(self.runtime.status()['last_move']['success'])

    def test_release_disconnects_and_cannot_be_remotely_reactivated(self):
        self.arm()
        self.runtime.submit_twist(PlanarTwist(.1))
        self.assertTrue(self.runtime.release().accepted)
        self.assertFalse(self.backend.read_state().connected)
        self.assertTrue(self.runtime.status()['released'])
        self.assertEqual(self.runtime.status()['health'],'released')
        self.assertFalse(self.runtime.arm().accepted)
        with self.assertRaises(RuntimeError): self.runtime.activate()
        count=len(self.backend.commands)
        self.runtime.cycle()
        self.assertEqual(len(self.backend.commands),count)

    def test_stalled_feedback_aborts_without_raising_speed_limit(self):
        self.arm()
        read=self.backend.read_state
        self.backend.read_state=lambda: replace(read(),position_m=(0,0,0))
        result=self.runtime.execute_move(MoveRequest(forward_m=.2,duration_sec=10))
        self.assertIn('stalled',result.reason)
        self.assertLess(self.runtime.status()['last_move']['ended_at_s']-100,4)
        self.assertEqual(self.runtime.status()['safety'],'disarmed')
        self.assertTrue(self.backend.current_twist.is_zero())

    def test_disconnected_and_invalid_feedback_are_distinct(self):
        state=self.backend.read_state()
        with self.assertRaisesRegex(RuntimeError,'disconnected'):
            self.runtime._healthy(replace(state,connected=False))
        with self.assertRaisesRegex(RuntimeError,'invalid'):
            self.runtime._healthy(replace(state,orientation_xyzw=(0,0,0,0)))


class SdkMotionTest(unittest.TestCase):
    def test_single_owner_walk_enable_zero_order_and_cleanup(self):
        calls = []
        class Control:
            def setVelocity(self, *values): calls.append(("velocity", values)); return True
            def setWalk(self): calls.append(("walk",))
            def setVelocityControlFlag(self, enabled): calls.append(("enable", enabled))
        class Source:
            def read_state(self):
                from bpx_quadruped.model import RobotState
                return RobotState(True, 0, motion_state=6)
            def stop(self): calls.append(("disconnect",))
        from bpx_quadruped.sdk_state_source import SdkStateConfig
        backend = SdkMotionBackend(SdkPostureConfig(SdkStateConfig()),
                                   sleep=lambda seconds: calls.append(("flush", seconds)))
        backend._control = Control()
        backend._source = Source()
        backend.prepare_motion()
        self.assertEqual(calls[:4], [("velocity", (0., 0., 0.)), ("walk",), ("enable", True), ("velocity", (0., 0., 0.))])
        backend.stop()
        self.assertEqual(calls[-1], ("disconnect",))
        self.assertIn(("enable", False), calls)
        self.assertIn(("flush", .1), calls)
        backend.stop()

    def test_rejected_stop_disables_flag_and_disconnects(self):
        calls = []
        class Control:
            def setVelocity(self, *values): return False
            def setVelocityControlFlag(self, enabled): calls.append(("enable", enabled))
        class Source:
            def stop(self): calls.append(("disconnect",))
        from bpx_quadruped.sdk_state_source import SdkStateConfig
        backend = SdkMotionBackend(SdkPostureConfig(SdkStateConfig()))
        backend._control, backend._source = Control(), Source()
        with self.assertRaises(RuntimeError):
            backend.stop()
        self.assertIn(("enable", False), calls)
        self.assertEqual(calls[-1], ("disconnect",))


if __name__ == "__main__":
    unittest.main()
