#!/bin/bash
# Start existing Jetson camera/ROS/rosbridge scripts from the Pi.
set -uo pipefail
mode=start
case "${1:-}" in
  '') ;;
  --check) mode=check;;
  -h|--help)
    echo 'Usage: bash ~/start_jetsons.sh [--check]'
    echo 'Default: run start_13.sh, start_14.sh and start_15.sh over SSH.'
    echo '--check: verify SSH and startup files without starting nodes.'
    echo 'Does not start Pi control, navigate, or start RealSense.'
    exit 0;;
  *) echo "Unknown option: $1" >&2; exit 2;;
esac
[[ $# -le 1 ]] || { echo 'Too many arguments.' >&2; exit 2; }
logdir="$HOME/MA_Raihan/logs/jetson_startup/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$logdir"
failures=()
for role in 13 14 15; do
  echo "=== Jetson .$role ($mode) ==="
  command="test -x \"\$HOME/start_$role.sh\" && bash -n \"\$HOME/start_$role.sh\""
  if [[ "$mode" == start ]]; then
    command="$command && bash \"\$HOME/start_$role.sh\""
  fi
  if timeout 180 ssh -n -o BatchMode=yes -o StrictHostKeyChecking=yes \
      -o ConnectTimeout=30 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 \
      -o ControlMaster=no -o ControlPath=none \
      "unitree@192.168.123.$role" "$command" 2>&1 | tee "$logdir/$role.log"; then
    echo "[OK] .$role: startup command completed ($mode)."
  else
    echo "[FAILED] .$role: inspect $logdir/$role.log" >&2
    failures+=("$role")
  fi
done
echo "Logs: $logdir"
if [[ ${#failures[@]} -gt 0 ]]; then
  echo "Failed controllers: ${failures[*]}. Fix access/startup and rerun." >&2
  exit 1
fi
if [[ "$mode" == start ]]; then
  echo 'Jetson startup commands completed. V5 still checks actual sensor messages before collection.'
else
  echo 'All three SSH connections and startup scripts verified. No nodes started.'
fi
