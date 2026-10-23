"""Standby feedback gaps recover; control-time and malformed-data faults latch."""
from dataclasses import replace
import unittest

from bpx_quadruped.config import ProviderConfig
from bpx_quadruped.runtime import build_runtime
from bpx_quadruped.motion_runtime import MoveRequest
from bpx_quadruped.model import PlanarTwist
from test_motion_runtime import Clock


class FeedbackRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        config = ProviderConfig.from_mapping(dict(backend='fake', allow_motion=True,
            enable_motion_service=True, enable_posture_service=True))
        self.r = build_runtime(config, clock=self.clock, sleep=self.clock.sleep)
        self.r.threaded = False
        self.r.activate()
        self.addCleanup(self.r.shutdown)

    def gap(self):
        self.r.backend.set_state_frozen(True)
        self.clock.sleep(.6)
        self.r.cycle()

    def stable(self):
        self.r.backend.set_state_frozen(False)
        for _ in range(13):
            self.clock.sleep(.1)
            self.r.cycle()

    def test_standby_recovers_and_only_new_commands_move(self):
        self.r.set_posture('stand')
        self.gap()
        self.assertEqual(self.r.status()['health'], 'waiting_feedback')
        self.assertEqual(self.r.status()['fault'], '')
        self.assertIsNone(self.r.last_fault)
        self.assertFalse(self.r.execute_move(MoveRequest(forward_m=.1)).accepted)
        self.assertFalse(self.r.set_posture('sit').accepted)
        self.assertFalse(self.r.submit_twist(PlanarTwist(.1)).accepted)
        self.assertTrue(self.r.backend.current_twist.is_zero())
        self.stable()
        self.assertTrue(self.r.status()['feedback_ready'])
        self.assertEqual(self.r.status()['safety'], 'disarmed')
        self.assertTrue(self.r.backend.current_twist.is_zero())
        self.assertTrue(self.r.execute_move(MoveRequest(forward_m=.1)).accepted)

    def test_first_stale_sample_at_rpc_admission_does_not_latch(self):
        self.r.backend.set_state_frozen(True)
        self.clock.sleep(.6)
        self.assertFalse(self.r.execute_move(MoveRequest(duration_sec=.1)).accepted)
        self.assertEqual(self.r.status()['health'], 'waiting_feedback')
        self.stable()
        self.assertTrue(self.r.set_posture('stand').accepted)

    def test_recovery_requires_continuous_samples_and_repeated_gap_resets_it(self):
        self.gap()
        self.r.backend.set_state_frozen(False)
        self.r.cycle()
        for _ in range(4):
            self.clock.sleep(.1); self.r.cycle()
        self.assertFalse(self.r.set_posture('stand').accepted)
        self.r.backend.set_connected(False)
        self.r.cycle()
        self.r.backend.set_connected(True)
        self.r.cycle()
        self.clock.sleep(2)  # A single fresh read after a long gap is not stability.
        self.assertFalse(self.r.set_posture('stand').accepted)
        self.stable()
        self.assertTrue(self.r.set_posture('stand').accepted)

    def test_control_time_gap_never_auto_clears(self):
        self.r.set_posture('stand')
        self.r.submit_twist(PlanarTwist(.1))
        self.gap()
        self.assertEqual(self.r.status()['safety'], 'fault')
        self.stable()
        self.assertEqual(self.r.status()['safety'], 'fault')
        self.assertFalse(self.r.execute_move(MoveRequest(duration_sec=.1)).accepted)

    def test_posture_gap_latches_despite_disarmed_controller(self):
        self.r.backend.request_stand_up = lambda: True
        def gap_during_posture(dt):
            self.gap()
        self.r.sleep = gap_during_posture
        self.assertFalse(self.r.set_posture('stand').accepted)
        self.assertEqual(self.r.status()['safety'], 'fault')
        self.stable()
        self.assertEqual(self.r.status()['safety'], 'fault')

    def test_move_gap_stops_and_remains_latched(self):
        self.r.set_posture('stand')
        self.r.sleep = lambda dt: self.gap()
        result = self.r.execute_move(MoveRequest(forward_m=.1))
        self.assertFalse(result.accepted)
        self.assertEqual(self.r.status()['safety'], 'fault')
        self.assertTrue(self.r.backend.current_twist.is_zero())
        self.stable()
        self.assertFalse(self.r.execute_move(MoveRequest(forward_m=.1)).accepted)

    def test_stop_failure_and_invalid_feedback_never_auto_recover(self):
        read = self.r.backend.read_state
        self.r.backend.read_state = lambda: replace(read(), orientation_xyzw=(0, 0, 0, 0))
        self.r.cycle()
        self.assertEqual(self.r.status()['safety'], 'fault')
        self.r.backend.read_state = read
        self.r.clear_fault()
        def fail():
            raise RuntimeError('SDK stop rejected')
        self.r.backend.disable_motion = fail
        try:
            self.gap()
            self.assertEqual(self.r.status()['safety'], 'fault')
        finally:
            del self.r.backend.disable_motion
        self.stable()
        self.assertEqual(self.r.status()['safety'], 'fault')


if __name__ == '__main__':
    unittest.main()
