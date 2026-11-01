#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "$0")" && pwd)
# Stop only processes started by this folder, including their process groups.
for name in runner camera bringup guard; do
 if [[ -f "$root/Runs/$name.pid" ]]; then
  pid=$(cat "$root/Runs/$name.pid")
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  if [[ -r /proc/$pid/cmdline ]] && tr '\0' ' ' < /proc/$pid/cmdline | grep -Fq "$root"; then
   kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
   sleep 1
  fi
 fi
done
echo 'Stop signals sent only to MA_Raihan processes. Original workspaces unchanged.'
