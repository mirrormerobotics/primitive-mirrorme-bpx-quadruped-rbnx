#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:dev"
NETWORK_MODE=host
SDK_PROFILE=release
cli_init "bash scripts/run-read-only-probe.sh WHEEL [options] [probe options]"
cli_option --image IMAGE 'Runtime and wheel-verification image'
cli_option --network-mode NETWORK_MODE 'host (default) or bridge'
cli_option --package-root PKG 'Package root (normally inferred)'
cli_option --sdk-profile SDK_PROFILE 'release (default) or local-v1.0.8 (read-only tests only)'
CLI_PASSTHROUGH=1
cli_parse "$@"
set -- "${CLI_ARGS[@]}"
cli_network "$NETWORK_MODE"
WHEEL="${1:-}"
shift || true

[[ -n "$WHEEL" && -f "$WHEEL" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl [probe options]" >&2
  exit 2
}

bash "$PKG/scripts/verify-sdk-wheel.sh" "$WHEEL" --image "$IMAGE" --package-root "$PKG" --sdk-profile "$SDK_PROFILE"

wheel_name="$(basename "$WHEEL")"
robot_state_port=9873
probe_args=("$@")
for ((index = 0; index < ${#probe_args[@]}; index++)); do
  case "${probe_args[$index]}" in
    --robot-state-port)
      ((index + 1 < ${#probe_args[@]})) || {
        echo "--robot-state-port requires a value" >&2
        exit 2
      }
      robot_state_port="${probe_args[$((index + 1))]}"
      ;;
    --robot-state-port=*)
      robot_state_port="${probe_args[$index]#*=}"
      ;;
  esac
done

docker_network_args=()
case "$NETWORK_MODE" in
  host)
    docker_network_args=(--network host)
    ;;
  bridge)
    docker_network_args=(
      --network bridge
      --publish "$robot_state_port:$robot_state_port/udp"
    )
    ;;
  *)
    echo "--network-mode must be host or bridge" >&2
    exit 2
    ;;
esac

docker run --rm \
  "${docker_network_args[@]}" \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -e PYTHONPATH=/pkg \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  -v "$PKG:/pkg:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps --force-reinstall "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && cd /pkg && exec python3 -m bpx_quadruped.read_only_probe "$@"' \
  probe "$@"
