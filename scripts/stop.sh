#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${BPX_RBNX_CONTAINER:-rbnx_mirrorme_bpx_quadruped}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
STOP_TIMEOUT_S=30
cli_init 'bash scripts/stop.sh [options]'
cli_option --container-name CONTAINER 'Container to stop (label-checked)'
cli_option --stop-timeout-s STOP_TIMEOUT_S 'Graceful shutdown timeout (default: 30)'
cli_parse "$@"
cli_no_args
[[ "$STOP_TIMEOUT_S" =~ ^[0-9]{1,4}$ ]] || cli_die '--stop-timeout-s must be a nonnegative integer <= 9999'
[[ "$CONTAINER" != -* ]] || cli_die 'invalid container name'
LABEL_KEY="ai.robonix.package"
LABEL_VALUE="robonix.primitive.mirrorme.bpx.quadruped"

docker inspect "$CONTAINER" >/dev/null 2>&1 || exit 0
existing_label="$(docker inspect --format "{{ index .Config.Labels \"$LABEL_KEY\" }}" "$CONTAINER")"
[[ "$existing_label" == "$LABEL_VALUE" ]] || {
  echo "refusing to stop container $CONTAINER with unexpected label $existing_label" >&2
  exit 1
}

docker stop --time "$STOP_TIMEOUT_S" "$CONTAINER" >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
