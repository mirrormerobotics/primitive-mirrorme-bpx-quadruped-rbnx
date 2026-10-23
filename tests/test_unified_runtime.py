"""Unified posture/movement ownership and lifecycle recovery, no SDK hardware."""
from dataclasses import replace
import unittest

from bpx_quadruped.config import ProviderConfig
from bpx_quadruped.runtime import build_runtime
from bpx_quadruped.motion_runtime import MoveRequest
from bpx_quadruped.model import PlanarTwist
from test_motion_runtime import Clock


class UnifiedRuntimeTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        config = ProviderConfig.from_mapping(dict(backend='fake', allow_motion=True,
            enable_motion_service=True, enable_posture_service=True))
        self.runtime = build_runtime(config, clock=self.clock, sleep=self.clock.sleep)
        self.runtime.threaded = False
        self.runtime.activate()
        self.addCleanup(self.runtime.shutdown)

    def test_posture_move_posture_uses_one_owner_and_no_arm(self):
        r = self.runtime
        self.assertTrue(r.supports_posture)
        self.assertEqual(r.backend.start_count, 1)
        self.assertEqual(r.tick().motion_state, 0)
        self.assertFalse(r.execute_move(MoveRequest(forward_m=.1)).accepted)
        self.assertTrue(r.set_posture('stand').accepted)
        for req in (MoveRequest(forward_m=.1), MoveRequest(rotate_deg=15)):
            result = r.execute_move(req)
            self.assertTrue(result.accepted, result.reason)
        self.assertTrue(r.set_posture('sit').accepted)
        self.assertFalse(r.submit_twist(PlanarTwist(.1)).accepted)
        self.assertEqual(r.backend.start_count, 1)
        self.assertTrue(r.backend.current_twist.is_zero())

    def test_posture_and_move_cannot_overlap(self):
        r = self.runtime
        outcomes = []
        sleep = r.sleep
        def during_posture(dt):
            outcomes.extend([r.execute_move(MoveRequest(duration_sec=.1)),
                             r.submit_twist(PlanarTwist(.1)), r.set_posture('sit')])
            sleep(dt)
        r.sleep = during_posture
        self.assertTrue(r.set_posture('stand').accepted)
        self.assertTrue(outcomes)
        self.assertTrue(all(not x.accepted for x in outcomes))
        r.sleep = lambda dt: (outcomes.append(r.set_posture('sit')), sleep(dt))
        self.assertTrue(r.execute_move(MoveRequest(duration_sec=.1)).accepted)
        self.assertFalse(outcomes[-1].accepted)

    def test_fault_recovers_via_lifecycle_without_clear_fault(self):
        r = self.runtime
        self.assertTrue(r.set_posture('stand').accepted)
        r.submit_twist(PlanarTwist(.1))
        r.backend.set_connected(False)
        r.cycle()
        self.assertEqual(r.status()['safety'], 'fault')
        r.backend.set_connected(True)
        self.assertFalse(r.set_posture('sit').accepted)
        self.assertFalse(r.execute_move(MoveRequest(duration_sec=.1)).accepted)
        r.deactivate()
        r.activate()
        self.assertEqual(r.status()['fault'], '')
        self.assertTrue(r.backend.current_twist.is_zero())
        self.assertTrue(r.execute_move(MoveRequest(duration_sec=.1)).accepted)

    def test_posture_deadline_and_deactivation_cancel_without_followup_motion(self):
        r = self.runtime
        r.backend.request_stand_up = lambda: True  # Accepted but never reaches stand.
        start = self.clock()
        result = r.set_posture('stand', cancelled=lambda: self.clock()-start >= .2)
        self.assertFalse(result.accepted)
        self.assertIn('cancelled', result.reason)
        self.assertFalse(r.status()['busy'])
        r.sleep = lambda dt: (self.clock.sleep(dt), r.deactivate())
        self.assertFalse(r.set_posture('stand').accepted)
        self.assertEqual(r.status()['lifecycle'], 'inactive')
        self.assertTrue(r.backend.current_twist.is_zero())

    def test_posture_timeout_fault_and_invalid_name(self):
        r = self.runtime
        self.assertFalse(r.set_posture('jump').accepted)
        r.posture_config = replace(r.posture_config, stand_timeout_s=.2)
        r.backend.request_stand_up = lambda: True
        result = r.set_posture('stand')
        self.assertFalse(result.accepted)
        self.assertEqual(r.status()['safety'], 'fault')
        self.assertTrue(r.backend.current_twist.is_zero())

    def test_sit_stops_streaming_velocity_first(self):
        r = self.runtime
        r.set_posture('stand')
        r.submit_twist(PlanarTwist(.1))
        self.clock.sleep(.1)
        r.cycle()
        self.assertFalse(r.backend.current_twist.is_zero())
        self.assertTrue(r.set_posture('sit').accepted)
        self.assertTrue(r.backend.current_twist.is_zero())


if __name__ == '__main__':
    unittest.main()
