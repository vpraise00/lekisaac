#!/bin/bash
# Copyright (c) 2024, lekisaac Project
# SPDX-License-Identifier: Apache-2.0

#===============================================================================
# LeKiwi Teleoperation Example Script
#
# This script demonstrates how to run LeKiwi teleoperation with:
# - SO101 Leader for arm control
# - Keyboard (WASD/ZX) for mobile base control
#
# Controls:
#   Arm: Move SO101-Leader device
#   Base Movement: W/A/S/D (forward/left/backward/right)
#   Base Rotation: Z/X (left/right)
#   Start: B
#   Reset (failed): R
#   Reset (success): N
#===============================================================================

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEKISAAC_ROOT="$(dirname "$SCRIPT_DIR")"

# Default settings
PORT="/dev/ttyACM0"
TASK="LeKisaac-LeKiwi-Teleop-v0"
DEVICE="cuda"
NUM_ENVS=1
STEP_HZ=30

# Recording settings
RECORD=false
DATASET_FILE="${LEKISAAC_ROOT}/datasets/lekiwi_teleop.hdf5"
NUM_DEMOS=0  # 0 = infinite

# Base velocity settings
BASE_LINEAR_SPEED=0.3   # m/s
BASE_ANGULAR_SPEED=0.5  # rad/s

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --port)
            PORT="$2"
            shift 2
            ;;
        --record)
            RECORD=true
            shift
            ;;
        --dataset-file)
            DATASET_FILE="$2"
            shift 2
            ;;
        --num-demos)
            NUM_DEMOS="$2"
            shift 2
            ;;
        --recalibrate)
            RECALIBRATE="--recalibrate"
            shift
            ;;
        --quality)
            QUALITY="--quality"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --port PORT           Serial port for SO101-Leader (default: /dev/ttyACM0)"
            echo "  --record              Enable recording"
            echo "  --dataset-file FILE   Output HDF5 file path"
            echo "  --num-demos N         Number of demos to record (0 = infinite)"
            echo "  --recalibrate         Recalibrate SO101-Leader"
            echo "  --quality             Enable high quality rendering"
            echo "  --help                Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Build command
CMD="python ${LEKISAAC_ROOT}/scripts/teleop_lekiwi.py"
CMD="$CMD --task=$TASK"
CMD="$CMD --port=$PORT"
CMD="$CMD --device=$DEVICE"
CMD="$CMD --num_envs=$NUM_ENVS"
CMD="$CMD --step_hz=$STEP_HZ"
CMD="$CMD --base_linear_speed=$BASE_LINEAR_SPEED"
CMD="$CMD --base_angular_speed=$BASE_ANGULAR_SPEED"

if [ "$RECORD" = true ]; then
    CMD="$CMD --record"
    CMD="$CMD --dataset_file=$DATASET_FILE"
    CMD="$CMD --num_demos=$NUM_DEMOS"
fi

if [ -n "$RECALIBRATE" ]; then
    CMD="$CMD $RECALIBRATE"
fi

if [ -n "$QUALITY" ]; then
    CMD="$CMD $QUALITY"
fi

CMD="$CMD --enable_cameras"

echo "==============================================================================="
echo "LeKiwi Teleoperation"
echo "==============================================================================="
echo ""
echo "Running command:"
echo "  $CMD"
echo ""
echo "Controls:"
echo "  Arm:           Move SO101-Leader device"
echo "  Base Forward:  W"
echo "  Base Backward: S"
echo "  Base Left:     A"
echo "  Base Right:    D"
echo "  Rotate Left:   Z"
echo "  Rotate Right:  X"
echo "  Start:         B"
echo "  Reset (fail):  R"
echo "  Reset (success): N"
echo ""
echo "==============================================================================="

# Run the command
exec $CMD
