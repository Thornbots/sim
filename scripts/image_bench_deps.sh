#!/usr/bin/env bash
# Install the image bench CPU inference dependencies in the container workspace.
set -euo pipefail
python3 -m pip install --user --break-system-packages onnxruntime==1.23.2
colcon build --base-paths src/realsense-yolov8-nitros-bridge --packages-select realsense_yolov8_nitros_bridge --cmake-args -DISAAC_COMPONENTS=OFF
