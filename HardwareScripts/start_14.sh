#!/bin/bash
set -eo pipefail
bash "$HOME/MA_Raihan/start.sh" --takeover "$@"
bash "$HOME/MA_Raihan/scripts/local_startup.sh"
