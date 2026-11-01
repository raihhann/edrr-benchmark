#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/env.sh"
ros2 node list --no-daemon --spin-time 2
ros2 topic list --no-daemon --spin-time 2 -t
