#!/bin/bash
set -eo pipefail
root="$HOME/MA_Raihan"
backup="$root/updates/realsense_pi_20261116"
if [[ -s "$root/pids/realsense" ]];then
 pid=$(cat "$root/pids/realsense")
 if [[ "$pid" =~ ^[0-9]+$ && -r /proc/$pid/cmdline ]] && tr '\0' ' ' < /proc/$pid/cmdline | grep -Fq "$root/config/realsense_pi.launch";then kill -INT "$pid";sleep 3;fi
fi
for target in scripts/local_capture.py scripts/local_audit.py scripts/start_realsense.sh config/realsense_pi.launch scripts/start_pi_entrypoint.sh scripts/start_pi.sh;do
 saved="$backup/${target//\//_}"
 if [[ -f "$saved" ]];then cp "$saved" "$root/$target";fi
done
if [[ -f "$backup/home_start_pi.sh" ]];then cp "$backup/home_start_pi.sh" "$HOME/start_pi.sh";fi
echo 'Previous files restored; managed RealSense launch stopped. Robot base driver untouched. New unused helper files retained.'
