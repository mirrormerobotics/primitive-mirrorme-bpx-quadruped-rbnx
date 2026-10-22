"""Offline real ROS transport check; synthetic source, no SDK or control API."""
import json
import time
from dataclasses import replace
from bpx_quadruped import main as provider
from bpx_quadruped.model import RobotState, VendorTimestamps
from robonix_api.ros import RosBackend
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage


class Source:
    supports_twist = False
    mode = 'fresh'
    def activate(self): pass
    def deactivate(self): pass
    def shutdown(self): pass
    def tick(self):
        now = time.monotonic()
        timestamp = int(now * 10) * 100
        state = RobotState(True, now, imu_orientation_xyzw=(0.,0.,0.,1.),
            imu_linear_acceleration_mps2=(0.,0.,9.81), imu_angular_velocity_rps=(0.,0.,0.),
            joint_position_rad=(0.,)*12, vendor_timestamps=VendorTimestamps(
                odometry_ms=timestamp, joint_ms=timestamp, imu_ms=timestamp))
        if self.mode == 'stale': return replace(state, received_at_s=now-2)
        if self.mode == 'invalid': return replace(state, orientation_xyzw=(0.,)*4)
        if self.mode == 'disconnected': return replace(state, connected=False)
        return state


# Only skip Atlas metadata registration; all message creation, publication,
# subscriptions and TF use the production provider and actual ROS transport.
provider.primitive.declare_capability = lambda *args, **kwargs: None
provider.on_init({'backend':'fake', 'publish_imu':True, 'imu_frame':'test_imu',
                  'publish_odom_tf':True})
assert provider._config.publish_imu
provider._runtime = Source()
backend = RosBackend.get()
messages = {key: [] for key in ('odom','imu','joints','tf','health')}
subscriptions = []
for key, cls, topic in [('odom',Odometry,'/odom'),('imu',Imu,'/imu/data'),
        ('joints',JointState,'/joint_states'),('tf',TFMessage,'/tf'),
        ('health',DiagnosticArray,'/diagnostics')]:
    subscriptions.append(backend.create_subscription(cls,topic,
        lambda msg, key=key: messages[key].append(msg),qos='best_effort'))

try:
    provider.on_activate()
    time.sleep(3)
    assert all(messages.values()), {k:len(v) for k,v in messages.items()}
    assert len(messages['imu']) < len(messages['odom']) * .5, 'cached IMU republished'
    assert messages['imu'][-1].header.frame_id == 'test_imu'
    assert messages['imu'][-1].linear_acceleration.z == 9.81
    assert all(messages['odom'][-1].pose.covariance[i*7] > 0 for i in range(6))
    def stamp(msg): return (msg.header.stamp.sec,msg.header.stamp.nanosec)
    odom_stamps = {stamp(msg) for msg in messages['odom']}
    assert any(stamp(t) in odom_stamps for msg in messages['tf'] for t in msg.transforms)
    assert messages['health'][-1].status[0].level == bytes([0])
    for mode in ('stale','invalid','disconnected'):
        provider._runtime.mode = mode
        time.sleep(.3)
        count = len(messages['odom'])
        time.sleep(.3)
        assert len(messages['odom']) == count, mode + ' odom leaked'
        assert messages['health'][-1].status[0].level == bytes([2]), mode
    provider._runtime.mode = 'fresh'
    time.sleep(.3)
    assert messages['health'][-1].status[0].level == bytes([0])
    for topic in ('/odom','/imu/data','/joint_states','/tf','/diagnostics'):
        assert backend.node.count_publishers(topic) == 1, topic
    print(json.dumps({'result':'PASS','hardware_connected':False,
        'samples':{k:len(v) for k,v in messages.items()},
        'checks':['ROS transport','unique publishers','IMU dedup','TF stamp',
                  'nonzero odom covariance','stale/invalid/disconnected suppression',
                  'health recovery']}))
finally:
    provider.on_shutdown()
