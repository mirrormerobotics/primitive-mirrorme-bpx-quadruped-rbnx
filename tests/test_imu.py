import unittest
from dataclasses import replace
from types import SimpleNamespace
from bpx_quadruped.config import ProviderConfig
from bpx_quadruped.imu import ImuProjector
from bpx_quadruped.model import RobotState, VendorTimestamps

try:
    from sensor_msgs.msg import Imu
    from builtin_interfaces.msg import Time
except ImportError:
    class Time:
        pass

    class Imu:
        def __init__(self):
            self.header = SimpleNamespace()
            for name in ("orientation", "angular_velocity", "linear_acceleration"):
                setattr(self, name, SimpleNamespace())
                setattr(self, name + "_covariance", [0.0] * 9)


class ImuTest(unittest.TestCase):
    def setUp(self):
        self.projector = ImuProjector()
        self.state = RobotState(True, 1.0, imu_orientation_xyzw=(0., 0., 0., 2.),
                               imu_angular_velocity_rps=(1., 2., 3.),
                               vendor_timestamps=VendorTimestamps(imu_ms=10))

    def project(self, state):
        return self.projector.project(state, Imu, Time(), "imu_sensor")

    def test_normalization_missing_measurement_and_unknown_covariance(self):
        msg = self.project(self.state)
        self.assertEqual(msg.header.frame_id, "imu_sensor")
        self.assertEqual(msg.orientation.w, 1.0)
        self.assertEqual(msg.angular_velocity.z, 3.0)
        self.assertEqual(list(msg.orientation_covariance), [0.0] * 9)
        self.assertEqual(msg.linear_acceleration_covariance[0], -1.0)

    def test_cached_imu_does_not_get_new_stamp_when_odom_updates(self):
        self.assertIsNotNone(self.project(self.state))
        self.assertIsNone(self.project(replace(self.state, received_at_s=2.,
                         vendor_timestamps=VendorTimestamps(imu_ms=10, odometry_ms=100))))
        self.assertIsNotNone(self.project(replace(self.state,
                         vendor_timestamps=VendorTimestamps(imu_ms=11))))
        self.projector.reset()
        self.assertIsNotNone(self.project(self.state))

    def test_disconnected_empty_and_invalid_data(self):
        self.assertIsNone(self.project(replace(self.state, connected=False)))
        self.assertIsNone(self.project(RobotState(True, 1.)))
        with self.assertRaises(ValueError):
            self.project(replace(self.state, imu_angular_velocity_rps=(float('nan'), 0., 0.)))

    def test_explicit_frame_required_and_opt_in(self):
        self.assertFalse(ProviderConfig.from_mapping({}).publish_imu)
        with self.assertRaises(ValueError):
            ProviderConfig.from_mapping({'publish_imu': True})
        with self.assertRaises(ValueError):
            ProviderConfig.from_mapping({'publish_imu': 'true'})
        config = ProviderConfig.from_mapping({'publish_imu': True, 'imu_frame': 'imu_sensor'})
        self.assertTrue(config.publish_imu)
        self.assertEqual(config.imu_topic, '/imu/data')
