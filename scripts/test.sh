#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
source "$(dirname "${BASH_SOURCE[0]}")/lib/cli.sh"
cli_init 'bash scripts/test.sh [options]'
cli_option --package-root PKG 'Package root'
cli_parse "$@"
cli_no_args
cd "$PKG"

exec python3 -m unittest discover -s tests -v
