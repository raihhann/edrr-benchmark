#!/bin/bash
set -e
backup=$(cd -- "$(dirname -- "$0")" && pwd)
hardware=$(cd -- "$backup/../.." && pwd)
cp -p "$backup/start_pi.sh.before" "$hardware/start_pi.sh"
cp -p "$backup/cmds.md.before" "$hardware/../cmds.md"
for name in start_lidar.sh lidar_settings.py;do
 if [[ -f "$backup/$name.before" ]];then cp -p "$backup/$name.before" "$hardware/MA_Raihan/scripts/$name";fi
done
echo 'Restored the laptop entry point and command guide. New unused helper files retained.'
