#!/bin/bash
# Stop only MA_Raihan nodes; add --restore to return to previous camera processes.
set -e
exec bash "$HOME/MA_Raihan/stop.sh" "$@"
