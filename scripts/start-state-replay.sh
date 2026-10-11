#!/usr/bin/env bash
set -euo pipefail

export BPX_REQUIRE_POSTURE_CONFIRMATION=0
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

export BPX_REQUIRED_BACKEND=replay
export BPX_POSTURE_CAPABILITY=false
export BPX_REQUIRED_POSTURE_CAPABILITY=false
exec bash "$SCRIPT_DIR/start.sh" "$@"
