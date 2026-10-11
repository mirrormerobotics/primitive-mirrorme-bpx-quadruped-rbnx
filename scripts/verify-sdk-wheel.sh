#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:dev"
POSTURE=0
YAW_CIRCLE=0
SDK_PROFILE=release
cli_init 'bash scripts/verify-sdk-wheel.sh WHEEL [options]'
cli_option --image IMAGE 'Verification image'
cli_option --package-root PKG 'Package root'
cli_option --sdk-profile SDK_PROFILE 'release (default) or local-v1.0.8 (read-only tests only)'
cli_flag --posture POSTURE 'Verify posture API'
cli_flag --yaw-circle YAW_CIRCLE 'Verify yaw-circle API'
cli_parse "$@"
set -- "${CLI_ARGS[@]}"
(($# == 1)) || cli_die 'exactly one wheel path is required'
((POSTURE + YAW_CIRCLE <= 1)) || cli_die 'verification modes are mutually exclusive'
case "$SDK_PROFILE" in
  release) CHECKSUMS="$PKG/sdk/SHA256SUMS" ;;
  local-v1.0.8)
    ((POSTURE == 0 && YAW_CIRCLE == 0)) || cli_die 'local-v1.0.8 is restricted to read-only testing'
    CHECKSUMS="$PKG/sdk/SHA256SUMS.local-v1.0.8"
    echo 'WARNING: local-v1.0.8 read-only test artifact; not the release baseline.' >&2
    ;;
  *) cli_die '--sdk-profile must be release or local-v1.0.8' ;;
esac
WHEEL="${1:-}"
VERIFY_MODE=""
((POSTURE == 0)) || VERIFY_MODE=--posture
((YAW_CIRCLE == 0)) || VERIFY_MODE=--yaw-circle

[[ -n "$WHEEL" && -f "$WHEEL" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl [--posture|--yaw-circle]" >&2
  exit 2
}
[[ -z "$VERIFY_MODE" || "$VERIFY_MODE" == "--posture" || "$VERIFY_MODE" == "--yaw-circle" ]] || {
  echo "optional second argument must be --posture or --yaw-circle" >&2
  exit 2
}

wheel_name="$(basename "$WHEEL")"
expected="$(awk -v name="$wheel_name" '!/^#/ && NF == 2 && $2 == name {print $1}' "$CHECKSUMS")"
[[ "$expected" =~ ^[a-f0-9]{64}$ ]] || cli_die "wheel is not uniquely pinned in $CHECKSUMS: $wheel_name"
actual="$(sha256sum "$WHEEL" | awk '{print $1}')"
[[ "$actual" == "$expected" ]] || {
  echo "BPX SDK wheel checksum mismatch" >&2
  echo "expected: $expected" >&2
  echo "actual:   $actual" >&2
  exit 1
}

docker run --rm --network none \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -e BPX_SDK_VERIFY_MODE="$VERIFY_MODE" \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps --force-reinstall "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && python3 -m bpx_quadruped.verify_sdk_api $BPX_SDK_VERIFY_MODE'
