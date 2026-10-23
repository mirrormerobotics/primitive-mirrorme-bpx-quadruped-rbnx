"""Atlas + real provider + ROS/gRPC in an isolated, fake-only container."""
import json
import os
import socket
import subprocess
import sys
import time

assert os.environ.get('BPX_REQUIRED_BACKEND') == 'fake'
assert not os.environ.get('BPX_SDK_WHEEL'), 'never mount the SDK in this test'
from bpx_quadruped.move_client import call_move
from bpx_quadruped.posture_client import call_posture
from bpx_quadruped.motion_runtime import MoveRequest
from bpx_quadruped.local_control import SOCKET_PATH
from robonix_api import ATLAS
import grpc
import lifecycle_pb2
from robonix_contracts_pb2_grpc import RobonixLifecycleDriverStub
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

PROVIDER = 'mirrorme_bpx'


def driver(command, config=None):
    with ATLAS.connect_capability(consumer_id='unified-test', provider_id=PROVIDER,
            contract_id='robonix/lifecycle/driver', transport='grpc') as binding:
        with grpc.insecure_channel(binding.endpoint) as channel:
            response = RobonixLifecycleDriverStub(channel).Driver(
                lifecycle_pb2.Driver_Request(command=command, config_json=json.dumps(config or {})), timeout=10)
            assert response.ok, response
            return response


def status():
    with socket.socket(socket.AF_UNIX) as client:
        client.connect(SOCKET_PATH)
        client.sendall(b'status\n')
        return json.loads(client.makefile().readline())


def posture(name):
    result = call_posture(name, provider_id=PROVIDER, timeout_s=5)
    assert result['success'], result


def move(**kw):
    result = call_move(MoveRequest(**kw), provider_id=PROVIDER, timeout_s=10)
    assert result['success'], result


processes = []
try:
    processes.append(subprocess.Popen(['/target/release/robonix-atlas', '--listen', '127.0.0.1:50051',
                                      '--capabilities', '/robonix/capabilities']))
    with grpc.insecure_channel('127.0.0.1:50051') as channel:
        grpc.channel_ready_future(channel).result(timeout=10)
    processes.append(subprocess.Popen([sys.executable, '-m', 'bpx_quadruped.main']))
    config = dict(backend='fake', allow_motion=True, enable_posture_service=True,
                  enable_motion_service=True, publish_odom_tf=False)
    deadline = time.monotonic()+20
    while True:
        try:
            driver(0, config)
            break
        except Exception:
            if processes[-1].poll() is not None or time.monotonic() > deadline:
                raise
            time.sleep(.2)
    driver(1)
    assert status()['feedback']['motion_state'] == 0  # activation never stands
    assert not call_move(MoveRequest(forward_m=.1), provider_id=PROVIDER)['success']
    posture('stand')
    move(forward_m=.1)
    move(rotate_deg=15)
    move(linear_x=.1, duration_sec=.3)
    assert status()['adapter_output']['linear_x_mps'] == 0
    # Real ROS command and telemetry through the same provider/session.
    rclpy.init()
    node = rclpy.create_node('unified_contract_test')
    samples = []
    sub = node.create_subscription(Odometry, '/odom', samples.append, qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    msg = Twist(); msg.linear.x = .1
    for _ in range(30):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=.05)
        time.sleep(.03)
    assert samples, 'no odometry publication'
    assert status()['adapter_output']['linear_x_mps'] > 0
    time.sleep(.4)
    assert status()['adapter_output']['linear_x_mps'] == 0, 'command watchdog failed'
    posture('sit')
    assert not call_move(MoveRequest(duration_sec=.1), provider_id=PROVIDER)['success']
    posture('stand')
    # A malformed ROS command must latch FAULT; restore through standard Driver only.
    msg.linear.x = float('nan')
    for _ in range(5):
        pub.publish(msg); time.sleep(.05)
    assert status()['safety'] == 'fault'
    assert not call_move(MoveRequest(duration_sec=.1), provider_id=PROVIDER)['success']
    driver(2)
    driver(1)
    assert status()['fault'] == ''
    assert status()['adapter_output']['linear_x_mps'] == 0
    move(linear_x=.1, duration_sec=.2)
    posture('sit')
    node.destroy_node(); rclpy.shutdown()
    driver(3)
    print('UNIFIED PASS: Atlas posture/move, ROS twist/odom, watchdog and Driver fault recovery without arm/clear-fault', flush=True)
finally:
    for proc in reversed(processes):
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill(); proc.wait()
