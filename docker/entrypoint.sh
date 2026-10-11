#!/usr/bin/env bash
set -eo pipefail

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

if [[ -d /pkg ]]; then
  cd /pkg
  export PYTHONPATH="/pkg:${PYTHONPATH:-}"
fi
if [[ -d /robonix-api ]]; then
  export PYTHONPATH="/robonix-api:${PYTHONPATH:-}"
fi

# robonix-api normally derives the advertised provider address with `ip route`.
# The slim ROS runtime image does not ship iproute2, so bridge deployments must
# publish the container's reachable IPv4 address explicitly.
if [[ "${BPX_DOCKER_NETWORK_MODE:-host}" == "bridge" && -z "${ROBONIX_ADVERTISE_HOST:-}" ]]; then
  ROBONIX_ADVERTISE_HOST="$(hostname -i | awk '{print $1}')"
  [[ -n "$ROBONIX_ADVERTISE_HOST" ]] || {
    echo "failed to determine bridge container address" >&2
    exit 1
  }
  export ROBONIX_ADVERTISE_HOST
fi

if [[ -n "${BPX_SDK_WHEEL:-}" ]]; then
  [[ -f "$BPX_SDK_WHEEL" ]] || {
    echo "mounted BPX_SDK_WHEEL is missing: $BPX_SDK_WHEEL" >&2
    exit 1
  }
  python3 -m pip install --no-index --no-deps --force-reinstall "$BPX_SDK_WHEEL"
  python3 -c 'import bpx_sdk; assert bpx_sdk.__version__ == "1.0.8"'
fi

exec "$@"
