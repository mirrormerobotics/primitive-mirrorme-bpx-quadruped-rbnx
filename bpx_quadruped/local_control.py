"""Local-only operator controls. Run the client with docker exec, never TCP."""

import argparse
import json
import os
import socket
import socketserver
import threading

SOCKET_PATH = "/tmp/bpx-motion-control.sock"


class LocalControl:
    def __init__(self, runtime, path=SOCKET_PATH):
        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                self.connection.settimeout(2)
                action = self.rfile.readline(128).decode().strip()
                try:
                    if action == "status":
                        result = runtime.status()
                    elif action in ("arm", "clear_fault", "release"):
                        decision = getattr(runtime, action)()
                        result = {"accepted": decision.accepted, "reason": decision.reason}
                    elif action in ("stand", "sit"):
                        decision = runtime.local_posture(action)
                        result = {"accepted": decision.accepted, "reason": decision.reason}
                    elif action in ("disarm", "cancel"):
                        runtime.disarm("local " + action)
                        result = {"accepted": True, "reason": action}
                    else:
                        result = {"accepted": False, "reason": "unknown local action"}
                except Exception as exc:
                    result = {"accepted": False, "reason": str(exc)}
                self.wfile.write((json.dumps(result) + "\n").encode())

        class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
            daemon_threads = True

        # Refuse an existing socket rather than removing another process's control.
        self.path = path
        self.server = Server(path, Handler)
        os.chmod(path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        os.unlink(self.path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "stand", "sit", "arm", "disarm", "clear_fault", "cancel", "release"))
    args = parser.parse_args(argv)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(20)
        client.connect(SOCKET_PATH)
        client.sendall((args.action + "\n").encode())
        with client.makefile("r") as reader:
            result = json.loads(reader.readline(4096))
    print(json.dumps(result))
    return 0 if result.get("accepted", True) else 2


if __name__ == "__main__":
    raise SystemExit(main())
