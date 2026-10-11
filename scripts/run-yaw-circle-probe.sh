#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:dev"
NETWORK_MODE=host
CONTAINER="bpx_motion_level_yaw_circle_probe"
cli_init "bash scripts/run-yaw-circle-probe.sh WHEEL --robot-ip IP --confirm-physical-yaw-circle [options] [probe options]"
cli_option --image IMAGE 'Runtime and wheel-verification image'
cli_option --network-mode NETWORK_MODE 'host (default) or bridge'
cli_option --package-root PKG 'Package root (normally inferred)'
cli_option --container-name CONTAINER 'Hardware probe container name'
CLI_PASSTHROUGH=1
cli_parse "$@"
set -- "${CLI_ARGS[@]}"
cli_network "$NETWORK_MODE"
WHEEL="${1:-}"
LABEL_KEY="ai.robonix.hardware-test"
LABEL_VALUE="mirrorme-bpx-yaw-circle"
shift || true

[[ -n "$WHEEL" && -f "$WHEEL" ]] || {
  echo "usage: $0 /path/to/bpx_sdk_open-1.0.8-cp310-...x86_64.whl --robot-ip IPV4 --confirm-physical-yaw-circle [options]" >&2
  exit 2
}

confirmed=0
robot_ip=""
robot_state_port=9873
probe_args=("$@")
for ((index = 0; index < ${#probe_args[@]}; index++)); do
  case "${probe_args[$index]}" in
    --confirm-physical-yaw-circle)
      confirmed=1
      ;;
    --robot-ip)
      ((index + 1 < ${#probe_args[@]})) || {
        echo "--robot-ip requires a value" >&2
        exit 2
      }
      robot_ip="${probe_args[$((index + 1))]}"
      ;;
    --robot-ip=*)
      robot_ip="${probe_args[$index]#*=}"
      ;;
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
((confirmed)) || {
  echo "refusing physical yaw motion: --confirm-physical-yaw-circle is required" >&2
  exit 2
}
[[ -n "$robot_ip" ]] || {
  echo "refusing physical yaw motion: --robot-ip is required" >&2
  exit 2
}
[[ "$robot_state_port" =~ ^[0-9]+$ ]] \
  && ((robot_state_port >= 1024 && robot_state_port <= 65535)) || {
  echo "--robot-state-port must be an integer in 1024..65535" >&2
  exit 2
}

bash "$PKG/scripts/verify-sdk-wheel.sh" "$WHEEL" --image "$IMAGE" --package-root "$PKG" --yaw-circle

wheel_name="$(basename "$WHEEL")"
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

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
  existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
  echo "refusing to replace existing container $CONTAINER (label=$existing_label)" >&2
  exit 1
fi

started=0
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if ((started)) && docker inspect "$CONTAINER" >/dev/null 2>&1; then
    existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
    if [[ "$existing_label" == "$LABEL_VALUE" ]]; then
      docker stop -t 10 "$CONTAINER" >/dev/null 2>&1 || true
    fi
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

echo "WARNING: this command will stand, rotate, stop, and sit BPX at $robot_ip"
echo "The robot must start fully lying down in a clear circular area with a reachable physical E-stop."

started=1
docker run --rm --init \
  --name "$CONTAINER" \
  --label "$LABEL_KEY=$LABEL_VALUE" \
  "${docker_network_args[@]}" \
  -e BPX_SDK_WHEEL_NAME="$wheel_name" \
  -e PYTHONPATH=/pkg \
  -v "$(realpath "$WHEEL"):/sdk/$wheel_name:ro" \
  -v "$PKG:/pkg:ro" \
  --entrypoint /bin/bash \
  "$IMAGE" \
  -lc 'python3 -m pip install --no-index --no-deps --force-reinstall "/sdk/$BPX_SDK_WHEEL_NAME" >/dev/null && cd /pkg && exec python3 -m bpx_quadruped.yaw_circle_probe "$@"' \
  probe "$@"
