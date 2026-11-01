#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
source "$root/env.sh"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
cd "$root"
# The unverified raw-register driver is intentionally excluded from the default build.
colcon --log-base "$root/Logs/build" build --base-paths "$root/Packages/ma_turtlebot_bringup" \
 --build-base "$root/Build" --install-base "$root/Install" --executor sequential
