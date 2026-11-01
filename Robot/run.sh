#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/env.sh"
exec python3 "$root/Scripts/capture.py" "$@"
