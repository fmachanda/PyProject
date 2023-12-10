#!/bin/bash

PROJ_DIR=$(dirname "$(dirname "$0")")

if command -v python3 &>/dev/null; then
    PYTHON="python3"
else
    PYTHON="python"
fi

"$PYTHON" "$PROJ_DIR/gcs/gui_v2.py" &
