#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
# RBNX_BUILD_CLEAN is supplied by the package lifecycle, not an operator setting.
CLEAN="${RBNX_BUILD_CLEAN:-0}"
IMAGE="${BPX_RBNX_IMAGE:-primitive-mirrorme-bpx-quadruped-rbnx:dev}"
TEST_IMAGE="primitive-mirrorme-bpx-quadruped-rbnx:test"
USE_TUNA_MIRROR=1
cli_init 'bash scripts/build.sh [options]'
cli_option --package-root PKG 'Package root'
cli_option --image IMAGE 'Runtime image'
cli_option --test-image TEST_IMAGE 'Test image'
cli_option --use-tuna-mirror USE_TUNA_MIRROR '1 (default) or 0'
cli_flag --clean CLEAN 'Clean codegen and Docker builds'
cli_parse "$@"
cli_no_args
cd "$PKG"

command -v rbnx >/dev/null 2>&1 || {
  echo "rbnx is required for Robonix code generation; use scripts/build-container.sh for the offline image" >&2
  exit 1
}

CODEGEN_FLAGS=()
[[ "$CLEAN" == "1" ]] && CODEGEN_FLAGS+=(--clean)
rbnx codegen -p "$PKG" --ros2 "${CODEGEN_FLAGS[@]}"
bash scripts/build-container.sh --package-root "$PKG" --image "$IMAGE" --test-image "$TEST_IMAGE" --use-tuna-mirror "$USE_TUNA_MIRROR" "${CODEGEN_FLAGS[@]}"
