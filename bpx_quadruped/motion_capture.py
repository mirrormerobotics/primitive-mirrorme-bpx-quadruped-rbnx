"""Manual acceptance client: subscribe to odom and optionally invoke one move.

Never creates an SDK session, arms, changes gait, or issues posture commands.
"""
import argparse
import json
import math
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-only', action='store_true')
    parser.add_argument('--seconds', type=float, default=40)
    parser.add_argument('--forward-m', type=float, default=0)
    parser.add_argument('--rotate-deg', type=float, default=0)
    parser.add_argument('--linear-x', type=float, default=0)
    parser.add_argument('--linear-y', type=float, default=0)
    parser.add_argument('--angular-z', type=float, default=0)
    parser.add_argument('--duration-sec', type=float, default=10)
    parser.add_argument('--timeout-s', type=float, default=35)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('seconds must be 1..120')
    if not math.isfinite(args.timeout_s) or not .1 <= args.timeout_s <= 60:
        parser.error('timeout-s must be .1..60')
    from .motion_runtime import MoveRequest
    request = MoveRequest(**{k: getattr(args,k) for k in
                          ('forward_m','rotate_deg','linear_x','linear_y','angular_z','duration_sec')})
    request.validate()
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    lock = threading.Lock()
    def emit(event, **values):
        with lock:
            print(json.dumps(dict(event=event, monotonic=time.monotonic(), **values), allow_nan=False), flush=True)
    rclpy.init()
    node = rclpy.create_node('bpx_manual_motion_capture')
    received = []
    def on_odom(msg):
        received[:] = [time.monotonic()]
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        emit('odom', stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
             frame=msg.header.frame_id, child_frame=msg.child_frame_id,
             position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w],
             linear=[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z],
             angular=[msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z])
    subscription = node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
    def on_status(msg):
        try: emit('motion_status', status=json.loads(msg.data))
        except (ValueError,TypeError): emit('diagnostic_error', message='Invalid motion status JSON')
    status_subscription = node.create_subscription(String,'/bpx/motion_status',on_status,10)
    done = threading.Event()
    outcome = {'success': False}
    def invoke():
        from .move_client import call_move
        emit('request', command=vars(request), timeout_s=args.timeout_s)
        try:
            outcome.update(call_move(request, timeout_s=args.timeout_s))
        except Exception as exc:
            outcome.update(success=False, message=str(exc))
        finally:
            emit('result', **outcome)
            done.set()
    try:
        emit('capture_start', observe_only=args.observe_only)
        deadline=time.monotonic()+5
        while not received and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if not received:
            emit('error', message='No odom received; no move requested')
            return 2
        emit('ready')
        if not args.observe_only:
            threading.Thread(target=invoke,daemon=True).start()
        deadline=time.monotonic()+(args.seconds if args.observe_only else args.timeout_s+10)
        result_seen=None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if done.is_set():
                if result_seen is None: result_seen=time.monotonic()
                if time.monotonic()-result_seen>=2: break
        emit('capture_end', last_odom_age_s=time.monotonic()-received[-1])
        return 0 if args.observe_only or outcome['success'] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
