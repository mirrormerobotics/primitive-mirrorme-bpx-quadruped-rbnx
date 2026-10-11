#!/usr/bin/env bash
set -euo pipefail

export BPX_REQUIRE_POSTURE_CONFIRMATION=0
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Constructing MotionLevelControl does not move the robot. The provider keeps
# velocity control disabled and exposes only the explicit stand/sit posture RPC.
export BPX_REQUIRED_BACKEND=sdk
export BPX_POSTURE_CAPABILITY=true
export BPX_REQUIRED_POSTURE_CAPABILITY=true
exec bash "$SCRIPT_DIR/start.sh" "$@"
