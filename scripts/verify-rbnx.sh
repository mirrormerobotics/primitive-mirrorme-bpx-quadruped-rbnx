#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROBONIX_SOURCE=""
EXPECTED_COMMIT="f6cacfc4b402eb57c75b553d375c6cfaba11dce7"
IMAGE="mirrorme-bpx-rbnx-codegen:dev"
CACHE=""
TARGET_CACHE=""
CARGO_CACHE=""
CONFIG_CACHE=""
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
cli_init 'bash scripts/verify-rbnx.sh [SOURCE] [options]'
cli_option --package-root PKG 'Package root'
cli_option --robonix-source ROBONIX_SOURCE 'Fixed Robonix checkout'
cli_option --expected-commit EXPECTED_COMMIT 'Expected full Robonix commit'
cli_option --tool-image IMAGE 'Robonix tool image'
cli_option --tool-cache CACHE 'Cache base directory'
cli_option --target-cache TARGET_CACHE 'Rust target cache'
cli_option --cargo-cache CARGO_CACHE 'Cargo cache'
cli_option --config-cache CONFIG_CACHE 'Robonix config cache'
cli_parse "$@"
((${#CLI_ARGS[@]} <= 1)) || cli_die 'at most one source path is allowed'
if ((${#CLI_ARGS[@]})); then
  [[ -z "$ROBONIX_SOURCE" ]] || cli_die 'source specified twice'
  ROBONIX_SOURCE="${CLI_ARGS[0]}"
fi
CACHE="${CACHE:-$PKG/.artifacts/rbnx-tool}"
TARGET_CACHE="${TARGET_CACHE:-$CACHE/target}"
CARGO_CACHE="${CARGO_CACHE:-$CACHE/cargo}"
CONFIG_CACHE="${CONFIG_CACHE:-$CACHE/config}"

[[ -n "$ROBONIX_SOURCE" && -d "$ROBONIX_SOURCE/.git" ]] || {
  echo "usage: $0 /path/to/fixed/robonix/source" >&2
  exit 2
}
command -v docker >/dev/null 2>&1 || {
  echo "docker is required" >&2
  exit 1
}

ROBONIX_SOURCE="$(realpath "$ROBONIX_SOURCE")"
actual_commit="$(git -C "$ROBONIX_SOURCE" rev-parse HEAD)"
[[ "$actual_commit" == "$EXPECTED_COMMIT" ]] || {
  echo "unexpected Robonix commit: $actual_commit" >&2
  echo "expected: $EXPECTED_COMMIT" >&2
  exit 1
}

submodule_status="$(git -C "$ROBONIX_SOURCE" submodule status -- \
  capabilities/lib/common_interfaces \
  capabilities/lib/rcl_interfaces \
  capabilities/lib/unique_identifier_msgs)"
if printf '%s\n' "$submodule_status" | rg -q '^[-+U]'; then
  echo "Robonix interface submodules are missing or not at recorded commits" >&2
  printf '%s\n' "$submodule_status" >&2
  exit 1
fi

mkdir -p "$TARGET_CACHE" "$CARGO_CACHE" "$CONFIG_CACHE"
TARGET_CACHE="$(realpath "$TARGET_CACHE")"
CARGO_CACHE="$(realpath "$CARGO_CACHE")"
CONFIG_CACHE="$(realpath "$CONFIG_CACHE")"

docker build --network host \
  -t "$IMAGE" \
  -f "$PKG/docker/Dockerfile.rbnx-tool" \
  "$PKG"

COMMON_MOUNTS=(
  -v "$ROBONIX_SOURCE:/src:ro"
  -v "$TARGET_CACHE:/target"
  -v "$CARGO_CACHE:/cargo"
  -v "$CONFIG_CACHE:/root/.robonix"
  -e CARGO_HOME=/cargo
  -e CARGO_TARGET_DIR=/target
)

docker run --rm --network host "${COMMON_MOUNTS[@]}" -w /src "$IMAGE" \
  cargo build --locked --release -p robonix-cli --bin rbnx

docker run --rm --network host "${COMMON_MOUNTS[@]}" -w /src "$IMAGE" \
  cargo build --locked --release -p robonix-codegen --bin robonix-codegen

docker run --rm --network host "${COMMON_MOUNTS[@]}" "$IMAGE" \
  /target/release/rbnx setup /src

docker run --rm --network host "${COMMON_MOUNTS[@]}" \
  -v "$PKG:/pkg" \
  -e RBNX_INVOCATION_CWD=/pkg \
  -e ROBONIX_CODEGEN_BIN=/target/release/robonix-codegen \
  -w /pkg \
  "$IMAGE" \
  /target/release/rbnx validate /pkg

docker run --rm --network host "${COMMON_MOUNTS[@]}" \
  -v "$PKG:/pkg" \
  -e RBNX_INVOCATION_CWD=/pkg \
  -e ROBONIX_CODEGEN_BIN=/target/release/robonix-codegen \
  -w /pkg \
  "$IMAGE" \
  /target/release/rbnx codegen -p /pkg --ros2 --clean
