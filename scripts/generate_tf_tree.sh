#!/bin/bash

echo "=========================================="
echo "      Morphobot TF Tree Generator"
echo "=========================================="

source /opt/ros/kilted/setup.bash

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

echo ""
echo "[INFO] Checking active TF topics..."

ros2 topic list | grep -E "^/tf$|^/tf_static$"

echo ""
echo "[INFO] Generating TF tree..."
echo "[INFO] Make sure the robot simulation is already running."
echo ""

ros2 run tf2_tools view_frames

echo ""
echo "=========================================="
echo "TF tree generation complete."
echo "=========================================="
echo ""
echo "Generated file:"
echo "    frames.pdf"
echo ""
echo "Open with:"
echo "    xdg-open frames.pdf"
echo ""