"""Passive ROS M3 evidence capture. No publishers, SDK or command clients."""
import argparse
import json
import statistics
import time
from collections import Counter
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('seconds must be 1..300')
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('bpx_m3_passive_capture')
    samples = {key: [] for key in ('odom','imu','joints','tf','health')}
    topics = [('odom',Odometry,'/odom'), ('imu',Imu,'/imu/data'),
              ('joints',JointState,'/joint_states'), ('tf',TFMessage,'/tf'),
              ('health',DiagnosticArray,'/diagnostics')]
    try:
        with path.open('x') as output:
            def receive(key, msg):
                now = time.monotonic()
                value = message_to_ordereddict(msg)
                samples[key].append((now, value))
                output.write(json.dumps({'topic':key,'monotonic':now,'data':value})+'\n')
            subscriptions = [node.create_subscription(cls, topic,
                lambda msg, key=key: receive(key,msg), qos_profile_sensor_data)
                for key,cls,topic in topics]
            deadline = time.monotonic()+args.seconds
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
            report = {'duration_s':args.seconds,'passive_only':True,
                      'physical_accuracy_verified':False, 'topics':{}}
            for key,cls,topic in topics:
                rows = samples[key]
                report['topics'][topic] = {'count':len(rows),
                    'publisher_count':node.count_publishers(topic),
                    'max_receive_gap_s':max((b[0]-a[0] for a,b in zip(rows,rows[1:])), default=None)}
            report['health_levels'] = dict(Counter(str(ord(s['level']) if isinstance(s['level'],str) and len(s['level'])==1 else s['level']) for _,msg in samples['health'] for s in msg['status']))
            report['health_messages'] = dict(Counter(s['message'] for _,msg in samples['health'] for s in msg['status']))
            report['frames'] = {key:sorted(set(msg['header']['frame_id'] for _,msg in samples[key])) for key in ('odom','imu')}
            report['last_health'] = samples['health'][-1][1] if samples['health'] else None
            report['imu_acceleration_mean'] = [statistics.mean(msg['linear_acceleration'][axis] for _,msg in samples['imu']) for axis in 'xyz'] if samples['imu'] else None
            positions = [[msg['pose']['pose']['position'][axis] for axis in 'xyz'] for _,msg in samples['odom']]
            report['observed_position_variance_only'] = [statistics.variance(p[i] for p in positions) for i in range(3)] if len(positions)>1 else None
            report['variance_note'] = 'Observed sample variation; no stationary/external-reference assumption and not calibrated covariance.'
            stamps = {json.dumps(msg['header']['stamp'],sort_keys=True) for _,msg in samples['odom']}
            transforms = [t for _,msg in samples['tf'] for t in msg['transforms'] if t['header']['frame_id']=='odom' and t['child_frame_id']=='base_link']
            report['odom_tf_matching_stamps'] = sum(json.dumps(t['header']['stamp'],sort_keys=True) in stamps for t in transforms)
            report['odom_tf_samples'] = len(transforms)
            report_path = path.with_suffix('.report.json')
            with report_path.open('x') as f:
                json.dump(report,f,indent=2)
            print(json.dumps(report,indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
