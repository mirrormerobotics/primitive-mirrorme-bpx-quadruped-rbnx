"""Optional raw IMU projection; does not invent an extrinsic or covariance."""
from .telemetry import normalize_robot_state


class ImuProjector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_timestamp = None

    def project(self, state, message_type, stamp, frame):
        if not state.connected:
            return None
        state = normalize_robot_state(state)
        fields = (state.imu_orientation_xyzw, state.imu_angular_velocity_rps,
                  state.imu_linear_acceleration_mps2)
        if not any(fields):
            return None
        timestamp = state.vendor_timestamps.imu_ms
        if timestamp is not None and timestamp == self.last_timestamp:
            return None
        msg = message_type()
        msg.header.stamp, msg.header.frame_id = stamp, frame
        for values, target, covariance in zip(fields,
                (msg.orientation, msg.angular_velocity, msg.linear_acceleration),
                (msg.orientation_covariance, msg.angular_velocity_covariance,
                 msg.linear_acceleration_covariance)):
            # sensor_msgs/Imu: all-zero covariance means unknown, -1 in the
            # first element means this measurement is absent, not perfect.
            if not values:
                covariance[0] = -1.0
            else:
                for axis, value in zip("xyzw", values):
                    setattr(target, axis, value)
        self.last_timestamp = timestamp
        return msg
