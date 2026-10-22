import unittest
from dataclasses import replace
from bpx_quadruped.model import RobotState
from bpx_quadruped.health import telemetry_health
from bpx_quadruped.config import ProviderConfig


class HealthTest(unittest.TestCase):
    def test_stale_invalid_disconnected_and_runtime_errors(self):
        state = RobotState(True, 1.)
        self.assertEqual(telemetry_health(state, 1.1, .5)['level'], 0)
        for sample, now, error in ((state, 2., ''), (state, .5, ''),
                (replace(state, connected=False), 1.1, ''),
                (replace(state, orientation_xyzw=(0.,)*4), 1.1, ''),
                (None, 1., 'read failed'), (state, 1.1, 'read failed')):
            self.assertEqual(telemetry_health(sample, now, .5, error)['level'], 2)

    def test_covariance_rejects_invalid_or_perfect_measurement(self):
        for value in ([0.]*6, [-1.]*6, [float('nan')]*6, [1.], 'bad'):
            with self.assertRaises(ValueError):
                ProviderConfig.from_mapping({'odom_pose_variance':value})
        self.assertEqual(ProviderConfig.from_mapping({'odom_twist_variance':[.1]*6}).odom_twist_variance, (.1,)*6)
