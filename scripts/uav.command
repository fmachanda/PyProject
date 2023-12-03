#!/bin/bash

PROJ_DIR=$(dirname "$(dirname "$0")")

if command -v python3 &>/dev/null; then
    PYTHON="python3"
else
    PYTHON="python"
fi

clear
echo "Using Python interpreter: $(which $PYTHON)"
echo "-----"

"$PYTHON" "$PROJ_DIR/uav/run.py"
