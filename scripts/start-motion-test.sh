#!/usr/bin/env bash
set -euo pipefail
export BPX_REQUIRED_BACKEND=sdk
export BPX_POSTURE_CAPABILITY=false BPX_REQUIRED_POSTURE_CAPABILITY=false
export BPX_MOTION_CAPABILITY=true BPX_REQUIRE_MOTION_CONFIRMATION=1
exec bash "$(dirname "$0")/start.sh" "$@"
