#!/bin/bash
# Interactive Pi startup. Control remains explicit; LiDAR is separately selected.
set -eo pipefail
root="$HOME/MA_Raihan"
lidar=ask
base_args=()
for arg in "$@"; do
  case "$arg" in
    --lidar|--no-lidar)
      [[ "$lidar" == ask ]] || { echo 'Choose only one of --lidar / --no-lidar.' >&2; exit 2; }
      [[ "$arg" == --lidar ]] && lidar=yes || lidar=no;;
    --control) base_args+=(--control);;
    -h|--help)
      echo 'Usage: bash ~/start_pi.sh [--control] [--lidar | --no-lidar]'
      echo 'Interactive startup asks about LiDAR; non-interactive startup defaults to no.'
      echo 'Default robot startup is passive. --control explicitly enables the control driver.'
      exit 0;;
    *) echo "Unknown option: $arg" >&2; exit 2;;
  esac
done
if [[ "$lidar" == ask ]]; then
  lidar=no
  if [[ -t 0 ]]; then
    while true; do
      read -r -p 'Is the RoboSense LiDAR connected and powered? Start it? [y/N]: ' answer || answer=n
      case "$answer" in
        y|Y|yes|YES) lidar=yes; break;;
        ''|n|N|no|NO) break;;
        *) echo 'Please enter y or n.';;
      esac
    done
  fi
fi
bash "$root/start.sh" "${base_args[@]}"
bash "$root/scripts/local_startup.sh"
if [[ "$lidar" == yes ]]; then
  bash "$root/scripts/start_lidar.sh"
else
  echo 'Skipping LiDAR startup. An already-running LiDAR driver is left running.'
fi
