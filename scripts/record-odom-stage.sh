#!/usr/bin/env bash
# Only RequestRobotState is constructed; no automatic reconnect or reboot.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:dev"
NETWORK_MODE=host
SDK_PROFILE=release
cli_init "bash scripts/record-odom-stage.sh WHEEL OUTPUT_DIR STAGE [options] [capture options]"
cli_option --image IMAGE 'Runtime and wheel-verification image'
cli_option --network-mode NETWORK_MODE 'host (default) or bridge'
cli_option --package-root PKG 'Package root (normally inferred)'
cli_option --sdk-profile SDK_PROFILE 'release (default) or local-v1.0.8 (read-only tests only)'
CLI_PASSTHROUGH=1
cli_parse "$@"
set -- "${CLI_ARGS[@]}"
cli_network "$NETWORK_MODE"
[[ $# -ge 3 ]] || {
  echo "usage: $0 WHEEL OUTPUT_DIR STAGE [read-only probe options] [--note TEXT]" >&2
  exit 2
}
WHEEL="$(realpath "$1")"
OUTPUT_DIR="$(realpath "$2")"
STAGE="$3"
shift 3
[[ -f "$WHEEL" && -d "$OUTPUT_DIR" && "$STAGE" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]] || {
  echo "wheel/directory must exist; stage must be a safe 1..64 character name" >&2
  exit 2
}
[[ ! -e "$OUTPUT_DIR/$STAGE.jsonl" ]] || {
  echo "refusing to overwrite $OUTPUT_DIR/$STAGE.jsonl" >&2
  exit 2
}
probe_args=("$@")
robot_state_port=9873
for ((index = 0; index < ${#probe_args[@]}; index++)); do
  case "${probe_args[$index]}" in
    --wheel|--wheel=*|--output-dir|--output-dir=*|--stage|--stage=*)
      echo "wheel, output directory and stage must be positional arguments" >&2
      exit 2 ;;
    --robot-state-port)
      ((index + 1 < ${#probe_args[@]})) || exit 2
      robot_state_port="${probe_args[$((index + 1))]}" ;;
    --robot-state-port=*) robot_state_port="${probe_args[$index]#*=}" ;;
  esac
done
[[ "$robot_state_port" =~ ^[0-9]{4,5}$ ]] &&
  ((10#$robot_state_port >= 1024 && 10#$robot_state_port <= 65535)) || {
    echo "robot-state-port must be in 1024..65535" >&2
    exit 2
  }
bash "$PKG/scripts/verify-sdk-wheel.sh" "$WHEEL" --image "$IMAGE" --package-root "$PKG" --sdk-profile "$SDK_PROFILE"
case "$NETWORK_MODE" in
  host) network_args=(--network host) ;;
  bridge) network_args=(--network bridge --publish "$robot_state_port:$robot_state_port/udp") ;;
  *) echo "--network-mode must be host or bridge" >&2; exit 2 ;;
esac
wheel_name="$(basename "$WHEEL")"
# exec + --init forwards Ctrl-C/SIGTERM to the probe, which flushes and disconnects.
exec docker run --rm --init "${network_args[@]}" \
  -e PYTHONPATH=/pkg -e PYTHONDONTWRITEBYTECODE=1 \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -v "$WHEEL:/sdk/$wheel_name:ro" -v "$PKG:/pkg:ro" \
  -v "$OUTPUT_DIR:/capture" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps --force-reinstall "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && cd /pkg && exec python3 -m bpx_quadruped.odom_acceptance capture --wheel "/sdk/$BPX_SDK_WHEEL_NAME" --output-dir /capture "$@"' \
  capture --stage "$STAGE" "$@"
