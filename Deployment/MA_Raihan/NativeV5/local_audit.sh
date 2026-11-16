#!/bin/bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/ros_ws/devel/setup.bash"
exec python2 "$root/scripts/local_audit.py" "$@"
