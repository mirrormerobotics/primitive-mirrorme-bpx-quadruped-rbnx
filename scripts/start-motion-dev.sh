#!/usr/bin/env bash
set -euo pipefail

export BPX_REQUIRE_POSTURE_CONFIRMATION=0
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Keep the command-capable development identity isolated from the production
# SDK package and fail if a deployment attempts to pair it with another backend.
export BPX_REQUIRED_BACKEND=fake
export BPX_POSTURE_CAPABILITY=false
export BPX_REQUIRED_POSTURE_CAPABILITY=false
export RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-robonix.primitive.mirrorme.bpx.quadruped.motion_dev}"
exec bash "$SCRIPT_DIR/start.sh" "$@"
