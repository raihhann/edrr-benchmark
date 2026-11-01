#!/usr/bin/env bash
prefix=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/Install
export AMENT_PREFIX_PATH="$prefix:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$prefix/lib:${LD_LIBRARY_PATH:-}"
for d in "$prefix"/lib/python*/site-packages "$prefix"/lib/python*/dist-packages "$prefix"/local/lib/python*/dist-packages; do
 [[ -d "$d" ]] && export PYTHONPATH="$d:${PYTHONPATH:-}"
done
