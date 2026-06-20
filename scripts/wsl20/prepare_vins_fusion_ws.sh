#!/usr/bin/env bash
set -euo pipefail

# Clone or copy and build VINS-Fusion in a ROS Noetic catkin workspace.
# Usage:
#   source /opt/ros/noetic/setup.bash
#   bash scripts/wsl20/prepare_vins_fusion_ws.sh ~/catkin_ws_vins

WS_DIR="${1:-$HOME/catkin_ws_vins}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOCAL_VINS_SRC="${LOCAL_VINS_SRC:-$PROJECT_DIR/data/public_samples/vins_zip_extract/VINS-Fusion-master}"

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ROS Noetic setup.bash not found. Run install_ros_noetic_vins_deps.sh first." >&2
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
mkdir -p "$WS_DIR/src"

if [ ! -d "$WS_DIR/src/VINS-Fusion" ]; then
  if [ -d "$LOCAL_VINS_SRC/vins_estimator" ]; then
    echo "Using local VINS-Fusion source: $LOCAL_VINS_SRC"
    mkdir -p "$WS_DIR/src/VINS-Fusion"
    tar \
      --exclude='./support_files/paper' \
      --exclude='.git' \
      -C "$LOCAL_VINS_SRC" -cf - . | tar -C "$WS_DIR/src/VINS-Fusion" -xf -
  else
    git -c http.version=HTTP/1.1 clone --depth 1 \
      https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git "$WS_DIR/src/VINS-Fusion"
  fi
fi

cd "$WS_DIR"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"${CATKIN_JOBS:-4}"

echo "VINS-Fusion workspace ready:"
echo "source $WS_DIR/devel/setup.bash"
