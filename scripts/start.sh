#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
CONTAINER="${BPX_RBNX_CONTAINER:-rbnx_mirrorme_bpx_quadruped}"
LABEL_KEY="ai.robonix.package"
LABEL_VALUE="robonix.primitive.mirrorme.bpx.quadruped"
RBNX_INSTANCE_NAME="${RBNX_INSTANCE_NAME:-robonix.primitive.mirrorme.bpx.quadruped}"
SDK_WHEEL="${BPX_SDK_WHEEL:-}"
# Environment defaults below are the Robonix package lifecycle contract.
# Explicit operator options always take precedence.
NETWORK_MODE="${BPX_DOCKER_NETWORK_MODE:-host}"
STATE_UDP_PORT="${BPX_DOCKER_STATE_UDP_PORT:-9873}"
DOCKER_HOST_ADDRESS="${BPX_DOCKER_HOST_ADDRESS:-host.docker.internal}"
ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
STOP_TIMEOUT_S=30
CONFIRM_POSTURE=0
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
cli_init 'bash scripts/start.sh [options] (also accepted by start-* wrappers)'
cli_option --package-root PKG 'Package root'
cli_option --image IMAGE 'Runtime image'
cli_option --container-name CONTAINER 'Primitive container name'
cli_option --instance-name RBNX_INSTANCE_NAME 'Robonix instance name'
cli_option --sdk-wheel SDK_WHEEL 'Pinned external SDK wheel'
cli_option --network-mode NETWORK_MODE 'host (default) or bridge'
cli_option --robot-state-port STATE_UDP_PORT 'Bridge UDP port (default: 9873)'
cli_option --docker-host-address DOCKER_HOST_ADDRESS 'Atlas host as seen from bridge'
cli_option --stop-timeout-s STOP_TIMEOUT_S 'Cleanup timeout (default: 30)'
cli_option --ros-domain-id ROS_DOMAIN_ID 'ROS 2 domain ID'
cli_option --atlas-address ROBONIX_ATLAS 'Atlas endpoint'
cli_option --provider-bind-host ROBONIX_PROVIDER_BIND_HOST 'Provider bind address'
cli_option --advertise-host ROBONIX_ADVERTISE_HOST 'Provider advertised address'
cli_option --rmw-implementation RMW_IMPLEMENTATION 'ROS middleware implementation'
cli_option --zenoh-router ROBONIX_ZENOH_ROUTER 'Zenoh router'
cli_option --zenoh-mode ROBONIX_ZENOH_MODE 'Zenoh mode'
cli_flag --confirm-physical-posture-service CONFIRM_POSTURE 'Allow manually invoked posture-test RPC service'
cli_parse "$@"
cli_no_args
cli_network "$NETWORK_MODE"
cli_port --robot-state-port "$STATE_UDP_PORT"
[[ "$STOP_TIMEOUT_S" =~ ^[0-9]{1,4}$ ]] || cli_die 'invalid --stop-timeout-s'
[[ "$CONTAINER" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || cli_die 'invalid --container-name'
if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  [[ "$ROS_DOMAIN_ID" =~ ^[0-9]{1,3}$ ]] && ((10#$ROS_DOMAIN_ID <= 232)) ||
    cli_die '--ros-domain-id must be in 0..232'
fi
if [[ "${BPX_REQUIRE_POSTURE_CONFIRMATION:-0}" == 1 ]]; then
  if ((CONFIRM_POSTURE == 0)) && [[ "${BPX_CONFIRM_PHYSICAL_POSTURE_SERVICE:-}" != I_UNDERSTAND_POSTURE_RPC_CAN_MOVE_BPX ]]; then
    cli_die 'posture-test requires --confirm-physical-posture-service with a local operator present'
  fi
fi
if [[ "${BPX_REQUIRED_BACKEND:-}" == sdk && -z "$SDK_WHEEL" ]]; then
  cli_die '--sdk-wheel is required for the SDK backend'
fi
export RBNX_INSTANCE_NAME
export ROBONIX_ATLAS ROBONIX_PROVIDER_BIND_HOST ROBONIX_ADVERTISE_HOST
export ROS_DOMAIN_ID RMW_IMPLEMENTATION ROBONIX_ZENOH_ROUTER ROBONIX_ZENOH_MODE
BPX_DOCKER_NETWORK_MODE="$NETWORK_MODE"
export BPX_DOCKER_NETWORK_MODE
cd "$PKG"

command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 1
}
command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx is required to locate robonix-api" >&2
  exit 1
}

DOCKER_SDK_ARGS=()
if [[ -n "$SDK_WHEEL" ]]; then
  [[ -f "$SDK_WHEEL" ]] || {
    echo "--sdk-wheel is not a file: $SDK_WHEEL" >&2
    exit 1
  }
  wheel_name="$(basename "$SDK_WHEEL")"
  expected_sha="$(awk -v name="$wheel_name" '$2 == name {print $1}' "$PKG/sdk/SHA256SUMS")"
  [[ -n "$expected_sha" ]] || {
    echo "--sdk-wheel is not a pinned artifact: $wheel_name" >&2
    exit 1
  }
  actual_sha="$(sha256sum "$SDK_WHEEL" | awk '{print $1}')"
  [[ "$actual_sha" == "$expected_sha" ]] || {
    echo "--sdk-wheel checksum mismatch: $wheel_name" >&2
    exit 1
  }
  sdk_wheel_path="$(realpath "$SDK_WHEEL")"
  BPX_SDK_WHEEL="/vendor-sdk/$wheel_name"
  export BPX_SDK_WHEEL
  DOCKER_SDK_ARGS+=(
    -v "$sdk_wheel_path:/vendor-sdk/$wheel_name:ro"
    -e BPX_SDK_WHEEL
  )
fi

DOCKER_NETWORK_ARGS=()
case "$NETWORK_MODE" in
  host)
    DOCKER_NETWORK_ARGS=(--network host)
    ;;
  bridge)
    state_udp_port="$STATE_UDP_PORT"
    docker_host_address="$DOCKER_HOST_ADDRESS"
    [[ "$state_udp_port" =~ ^[0-9]+$ ]] &&
      ((10#$state_udp_port >= 1024 && 10#$state_udp_port <= 65535)) || {
      echo "BPX_DOCKER_STATE_UDP_PORT must be in 1024..65535" >&2
      exit 2
    }
    DOCKER_NETWORK_ARGS=(
      --network bridge
      --publish "$state_udp_port:$state_udp_port/udp"
      --add-host host.docker.internal:host-gateway
    )
    case "${ROBONIX_ATLAS:-127.0.0.1:50051}" in
      127.0.0.1:*)
        ROBONIX_ATLAS="${docker_host_address}:${ROBONIX_ATLAS##*:}"
        export ROBONIX_ATLAS
        ;;
    esac
    ;;
  *)
    echo "BPX_DOCKER_NETWORK_MODE must be host or bridge" >&2
    exit 2
    ;;
esac

if docker inspect "$CONTAINER" >/dev/null 2>&1; then
  existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
  [[ "$existing_label" == "$LABEL_VALUE" ]] || {
    echo "refusing to replace container $CONTAINER with unexpected label $existing_label" >&2
    exit 1
  }
  docker rm -f "$CONTAINER" >/dev/null
fi

cleanup() {
  bash "$PKG/scripts/stop.sh" --container-name "$CONTAINER" --stop-timeout-s "$STOP_TIMEOUT_S" || true
}
trap cleanup EXIT INT TERM

docker run --rm \
  --name "$CONTAINER" \
  --label "$LABEL_KEY=$LABEL_VALUE" \
  "${DOCKER_NETWORK_ARGS[@]}" \
  --ipc host \
  -e ROBONIX_ATLAS \
  -e ROBONIX_PROVIDER_BIND_HOST \
  -e ROBONIX_ADVERTISE_HOST \
  -e BPX_DOCKER_NETWORK_MODE \
  -e RBNX_INSTANCE_NAME \
  -e RBNX_DEPLOY_MANAGED \
  -e BPX_REQUIRED_BACKEND \
  -e BPX_POSTURE_CAPABILITY \
  -e BPX_REQUIRED_POSTURE_CAPABILITY \
  -e ROBONIX_DRIVER_CONTRACT_ID \
  -e ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK \
  -e ROS_DOMAIN_ID \
  -e RMW_IMPLEMENTATION \
  -e ROBONIX_ZENOH_ROUTER \
  -e ROBONIX_ZENOH_MODE \
  -v "$PKG:/pkg" \
  -v "$(rbnx path robonix-api):/robonix-api:ro" \
  "${DOCKER_SDK_ARGS[@]}" \
  "$IMAGE" &
runner_pid=$!
wait "$runner_pid"
