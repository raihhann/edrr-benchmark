#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
source "$root/env.sh"
prefix="$root/Dependencies/root/opt/ros/humble"
export LIBCAMERA_IPA_MODULE_PATH="$prefix/lib/libcamera"
export LIBCAMERA_IPA_PROXY_PATH="$prefix/lib/libcamera"
export LIBCAMERA_IPA_CONFIG_PATH="$prefix/share/libcamera/ipa"
export LIBCAMERA_PIPELINE_HANDLERS_MATCH_LIST=rpi/vc4
exec "$prefix/lib/camera_ros/camera_node" --ros-args \
 -r __node:=pi -r __ns:=/camera \
 -p width:=640 -p height:=480 -p format:=RGB888 \
 -p frame_id:=camera_rgb_optical_frame \
 -p FrameDurationLimits:='[50000,50000]'
