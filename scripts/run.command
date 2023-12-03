#!/bin/bash

SCRIPT_DIR=$(dirname "$0")

open "$SCRIPT_DIR/uav.command" & open "$SCRIPT_DIR/gcs.command" &
