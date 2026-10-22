"""Transport-independent health for every provider identity."""
from dataclasses import asdict
from .telemetry import normalize_robot_state


def telemetry_health(state, now, timeout, error=""):
    result = {"level": 2, "message": error or "no state", "connected": False}
    if state is None:
        return result
    try:
        state = normalize_robot_state(state)
        age = now - state.received_at_s
        result.update(connected=state.connected, age_s=age,
                      vendor_timestamps=asdict(state.vendor_timestamps))
        if error:
            return result
        if not state.connected:
            result["message"] = "disconnected"
        elif age < 0 or age > timeout:
            result["message"] = "feedback stale or future timestamp"
        else:
            result.update(level=0, message="feedback fresh")
    except (ValueError, TypeError, AttributeError) as exc:
        result["message"] = "invalid feedback: " + str(exc)
    return result
