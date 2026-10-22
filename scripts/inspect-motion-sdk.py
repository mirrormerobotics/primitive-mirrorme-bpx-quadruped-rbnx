"""Offline API inspection: imports the SDK but never constructs/connects a robot."""
import json
from pathlib import Path
import bpx_sdk
from bpx_quadruped.verify_sdk_api import validate_posture_sdk_module

result = validate_posture_sdk_module(bpx_sdk)
result["motion_methods"] = {}
for name in ("setWalk", "setVelocity", "setVelocityControlFlag"):
    method = getattr(bpx_sdk.MotionLevelControl, name, None)
    if not callable(method):
        raise RuntimeError("missing motion method: " + name)
    result["motion_methods"][name] = method.__doc__
stub = Path(bpx_sdk.__file__).with_name("__init__.pyi").read_text()
result["declared_signatures"] = [line.strip() for line in stub.splitlines()
                                  if any("def " + name + "(" in line for name in result["motion_methods"])]
print(json.dumps(result, indent=2))
