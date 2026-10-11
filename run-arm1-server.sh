#!/usr/bin/env bash
# Robot PC only: wait for the existing control system's real Arm1 goals.
set -Eeuo pipefail
ARM1_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ARM1_WORKSPACE/arm1-control.sh" serve --hardware "$@"
