#!/usr/bin/env bash
set -Eeuo pipefail
ARM1_PACKAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ARM1_PACKAGE/jetson-run.sh" server "$@"
