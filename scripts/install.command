#!/bin/bash

PROJ_DIR=$(dirname "$(dirname "$0")")

if ! cd "$PROJ_DIR"; then
    echo "ERROR: Could not find script path. Try again manually."
    kill -INT $$
fi

if command -v python3 &>/dev/null; then
    PYTHON="python3"
else
    PYTHON="python"
fi

if ! command -v "$PYTHON" &>/dev/null; then
    echo "ERROR: $PYTHON is not installed or not in the system's PATH."
    kill -INT $$
fi

if ! command -v "git" &>/dev/null; then
    echo "ERROR: git is not installed or not in the system's PATH."
    kill -INT $$
fi

clear
echo "Using Python interpreter: $(which $PYTHON)"
echo "**********************************************"
echo "Installing and updating git submodules..."
echo "----------------------------------------------"
if ! git submodule update --init --recursive --remote --merge; then
    echo "ERROR: Git submodule install/update failed. Try again manually."
    kill -INT $$
fi
echo "----------------------------------------------"
echo "Git submodules installed and updated!"
echo "**********************************************"
echo

echo
echo "**********************************************"
echo "Installing and updating pip modules..."
echo "----------------------------------------------"
if ! "$PYTHON" -m pip install -r requirements.txt -U; then
    echo "ERROR: Pip module install/update failed. Try again manually."
    kill -INT $$
fi
echo "----------------------------------------------"
echo "Pip modules installed and updated!"
echo "**********************************************"
echo

echo "Installation successful!"
kill -INT $$
