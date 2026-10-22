"""Integration-only acceptance. Runs inside the fake provider container."""
import json
import math
import os
import socket
import time

assert os.environ.get("BPX_REQUIRED_BACKEND") == "fake", "fake-only test"
from bpx_quadruped.local_control import SOCKET_PATH
from bpx_quadruped.move_client import call_move
from bpx_quadruped.motion_runtime import MoveRequest
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


def local(action):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(5)
        client.connect(SOCKET_PATH)
        client.sendall((action + "\n").encode())
        result = json.loads(client.makefile().readline())
        assert result.get("accepted", True), result
        return result


rclpy.init()
node = rclpy.create_node("bpx_motion_acceptance")
samples = []
sub = node.create_subscription(Odometry, "/odom", samples.append, qos_profile_sensor_data)
pub = node.create_publisher(Twist, "/cmd_vel", 10)


def sample():
    samples.clear()
    # DDS may retain odometry from while the synchronous RPC was running.
    # Validate a newly published sample, not an old queued velocity.
    requested_at = node.get_clock().now().nanoseconds
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        fresh = [msg for msg in samples if
                 msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec >= requested_at]
        if fresh:
            return fresh[-1]
    raise AssertionError("missing fresh odom")


def stopped():
    time.sleep(.35)
    msg = sample()
    assert abs(msg.twist.twist.linear.x) < .001 and abs(msg.twist.twist.angular.z) < .001


try:
    status = local("status")
    assert status["safety"] == "disarmed", status
    assert not call_move(MoveRequest(forward_m=.1))["success"]
    start = sample().pose.pose.position
    local("arm")
    assert call_move(MoveRequest(forward_m=.1))["success"]
    end = sample().pose.pose.position
    assert .07 < math.hypot(end.x-start.x, end.y-start.y) < .14
    assert local("status")["safety"] == "disarmed"
    stopped()
    for degrees in (30, -30):
        local("arm")
        assert call_move(MoveRequest(rotate_deg=degrees))["success"]
        stopped()
    local("arm")
    assert call_move(MoveRequest(linear_x=.1, duration_sec=.5))["success"]
    stopped()
    local("arm")
    try:
        call_move(MoveRequest(linear_x=.1, duration_sec=5), timeout_s=.3)
        raise AssertionError("deadline did not cancel RPC")
    except Exception as exc:
        import grpc
        assert isinstance(exc, grpc.RpcError), exc
    stopped()
    assert local("status")["safety"] == "disarmed"
    local("arm")
    msg = Twist()
    msg.linear.x = .1
    for _ in range(20):
        pub.publish(msg)
        time.sleep(.05)
    assert sample().twist.twist.linear.x > .02
    stopped()
    local("disarm")
    print("motion RPC distance/rotation/timed/cancellation and ROS watchdog: PASS")
finally:
    local("disarm")
    node.destroy_node()
    rclpy.shutdown()
