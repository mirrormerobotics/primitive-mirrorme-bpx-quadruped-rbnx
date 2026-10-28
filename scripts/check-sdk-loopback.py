"""Real native SDK smoke test. Run ONLY in Docker --network none.
Loopback protocol reference: bpx_sdk_open tests/test_python_robot_features.py,
commit 5344e31ee825edb6ab17f019f6021f3d6097d40f. No robot connection.
"""
import json
import socket
import struct
import multiprocessing
import time
import bpx_sdk


def serve(server, stopped):
    server.settimeout(.1)
    while not stopped.is_set():
        try:
            conn, _ = server.accept()
        except socket.timeout:
            continue
        with conn:
            conn.settimeout(.05)
            buffer = b""
            response = None
            while not stopped.is_set():
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buffer += data
                except socket.timeout:
                    pass
                except OSError:
                    break
                while len(buffer) >= 24:
                    req, buffer = buffer[:24], buffer[24:]
                    session = struct.unpack_from("<H", req)[0]
                    mode = req[10]
                    if mode == 6:
                        packet = struct.pack("<BBHBBBB32s56s", 6, 1, session, 1, 0, 8, 0, b"LOOPBACK", b"")
                    elif mode == 4:
                        packet = struct.pack("<BBH", 4, 1, session) + bytes(60)
                    elif mode == 5:
                        packet = struct.pack("<BBH", 5, 0, session) + bytes(24) + bytes([2, 0, 0, 0])
                    else:
                        response = struct.pack("<HBxHBB", session, 1, 50, mode, 3) + bytes(24)
                        packet = response
                    conn.sendall(packet)
                if response:
                    try:
                        conn.sendall(response)
                    except OSError:
                        break


def wait(predicate):
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError("loopback feedback/command timed out")


def main():
    # Docker --network none must expose loopback only.
    import os
    assert set(os.listdir("/sys/class/net")) == {"lo"}, "requires isolated network namespace"
    stop = multiprocessing.Event()
    with socket.socket() as server, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", 10860))
        server.listen()
        thread = multiprocessing.Process(target=serve, args=(server, stop), daemon=True)
        thread.start()
        receiver.bind(("127.0.0.1", 9527))
        receiver.settimeout(.2)
        try:
            for kind in (bpx_sdk.RequestRobotState, bpx_sdk.MotionLevelControl):
                sdk = kind()
                try:
                    sdk.setRobotIp("127.0.0.1")
                    sdk.setRobotStateUploadPort(19873)
                    sdk.setRobotStateUploadRate(50)
                    assert sdk.connect()
                    wait(sdk.isConnected)
                    payload = bytes([6]) + bytes(19)
                    def feedback():
                        receiver.sendto(struct.pack("<HHI", 0x10, 20, 1234) + payload, ("127.0.0.1", 19873))
                        return sdk.getMotionStateTimestamp() == 1234
                    wait(feedback)
                    assert sdk.getCurrentMotionState() == 6
                    if kind is bpx_sdk.MotionLevelControl:
                        assert sdk.setWalk() is None
                        sdk.setVelocityControlFlag(True)
                        wait(lambda: sdk.setVelocity(.2, .1, .3))
                        def command(state, values):
                            try:
                                packet = receiver.recv(1024)
                            except socket.timeout:
                                return False
                            if len(packet) != 56 or packet[4] != state:
                                return False
                            actual = struct.unpack_from("<3f", packet, 8)
                            return all(abs(a-b) < 1e-5 for a,b in zip(actual,values))
                        wait(lambda: command(6, (.2, .1, .3)))
                        assert sdk.setVelocity(0, 0, 0)
                        wait(lambda: command(6, (0, 0, 0)))
                        sdk.setVelocityControlFlag(False)
                        assert sdk.setStandUp()
                        wait(lambda: command(1, (0, 0, 0)))
                        assert sdk.setSitDown()
                        wait(lambda: command(3, (0, 0, 0)))
                finally:
                    sdk.disconnect()
                assert not sdk.isConnected()
        finally:
            stop.set()
            thread.join(2)
            if thread.is_alive():
                thread.terminate()
                thread.join(2)
    print(json.dumps({"sdk_version": bpx_sdk.__version__, "loopback": "passed", "hardware": False}))

if __name__ == "__main__":
    main()
