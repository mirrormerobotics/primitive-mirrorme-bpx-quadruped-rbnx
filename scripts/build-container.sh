#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:0.1.0-dev.4"
TEST_IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:test"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
CLEAN=0
USE_TUNA_MIRROR=1
cli_init 'bash scripts/build-container.sh [options]'
cli_option --package-root PKG 'Package root'
cli_option --image IMAGE 'Runtime image'
cli_option --test-image TEST_IMAGE 'Test image'
cli_option --use-tuna-mirror USE_TUNA_MIRROR '1 (default) or 0'
cli_flag --clean CLEAN 'Build without Docker cache'
cli_parse "$@"
cli_no_args
[[ "$USE_TUNA_MIRROR" == 0 || "$USE_TUNA_MIRROR" == 1 ]] || cli_die '--use-tuna-mirror must be 0 or 1'
cd "$PKG"

command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 1
}

DOCKER_FLAGS=(--network=host)
((CLEAN)) && DOCKER_FLAGS+=(--no-cache)
DOCKER_FLAGS+=(--build-arg "USE_TUNA_MIRROR=$USE_TUNA_MIRROR")

docker build "${DOCKER_FLAGS[@]}" --target test -t "$TEST_IMAGE" -f docker/Dockerfile .
docker build "${DOCKER_FLAGS[@]}" --target runtime -t "$IMAGE" -f docker/Dockerfile .

echo "built $IMAGE after containerized tests passed"
