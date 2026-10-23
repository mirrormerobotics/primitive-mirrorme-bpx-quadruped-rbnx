"""Standard MoveCommand client; no manual arm is required."""
import argparse
import json
import uuid

from .motion_runtime import MoveRequest


def call_move(command, provider_id="mirrorme_bpx_motion", timeout_s=35):
    command.validate()
    if not 0.1 <= timeout_s <= 60:
        raise ValueError("RPC timeout must be 0.1..60 seconds")
    import grpc
    from robonix_api import ATLAS
    from chassis_pb2 import ExecuteMoveCommand_Request
    from robonix_contracts_pb2_grpc import RobonixPrimitiveQuadrupedMoveStub
    consumer = "bpx_move_" + uuid.uuid4().hex
    ATLAS.register_service(consumer, "robonix/client/manual")
    try:
        with ATLAS.connect_capability(consumer_id=consumer, provider_id=provider_id,
                                      contract_id="robonix/primitive/quadruped/move", transport="grpc") as binding:
            with grpc.insecure_channel(binding.endpoint, options=[("grpc.enable_http_proxy", 0)]) as channel:
                request = ExecuteMoveCommand_Request()
                for key, value in vars(command).items():
                    setattr(request.command, key, value)
                response = RobonixPrimitiveQuadrupedMoveStub(channel).ExecuteMoveCommand(request, timeout=timeout_s)
                return {"success": response.status.data.startswith("success: "), "message": response.status.data}
    finally:
        ATLAS.unregister(consumer)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in MoveRequest.__dataclass_fields__:
        parser.add_argument("--" + key.replace("_", "-"), type=float, default=0.0)
    parser.add_argument("--provider-id", default="mirrorme_bpx_motion")
    parser.add_argument("--timeout-s", type=float, default=35)
    args = vars(parser.parse_args())
    provider, timeout = args.pop("provider_id"), args.pop("timeout_s")
    try:
        result = call_move(MoveRequest(**args), provider, timeout)
    except Exception as exc:
        result = {"success": False, "message": str(exc)}
    print(json.dumps(result))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
