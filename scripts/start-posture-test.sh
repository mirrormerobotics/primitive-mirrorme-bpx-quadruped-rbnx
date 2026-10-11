#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIRMATION="I_UNDERSTAND_POSTURE_RPC_CAN_MOVE_BPX"

# start.sh validates the CLI acknowledgement or the Robonix-injected
# BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE token before any Docker operation.
export BPX_REQUIRE_POSTURE_CONFIRMATION=1

export BPX_REQUIRED_BACKEND=sdk
export BPX_POSTURE_CAPABILITY=true
export BPX_REQUIRED_POSTURE_CAPABILITY=true
export RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-robonix.primitive.mirrorme.bpx.quadruped.posture_test}"
exec bash "$SCRIPT_DIR/start.sh" "$@"
