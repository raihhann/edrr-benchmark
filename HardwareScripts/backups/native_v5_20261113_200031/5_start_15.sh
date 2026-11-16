#!/bin/bash
# MA_Raihan entry point. Previous version is backed up in MA_Raihan/inventory.
set -e
exec bash "$HOME/MA_Raihan/start.sh" --takeover "$@"
