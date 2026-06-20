#!/usr/bin/env bash
set -euo pipefail

# Run VINS-Fusion baseline on a EuRoC rosbag.
# This starts roscore, vins_node, optional rviz, and rosbag play.
# Usage:
#   bash scripts/wsl20/run_vins_fusion_euroc_baseline.sh ~/catkin_ws_vins /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag

WS_DIR="${1:-$HOME/catkin_ws_vins}"
BAG_PATH="${2:?Usage: run_vins_fusion_euroc_baseline.sh WS_DIR BAG_PATH}"
RVIZ="${RVIZ:-0}"
BAG_DURATION="${BAG_DURATION:-0}"
BAG_RATE="${BAG_RATE:-1.0}"
VINS_EXTRA_ARGS="${VINS_EXTRA_ARGS:-}"

if [ ! -f "$BAG_PATH" ]; then
  echo "bag not found: $BAG_PATH" >&2
  exit 2
fi

set +u
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

BASE_CONFIG="$WS_DIR/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml"
if [ ! -f "$BASE_CONFIG" ]; then
  echo "VINS EuRoC config not found: $BASE_CONFIG" >&2
  exit 2
fi

mkdir -p "$WS_DIR/results"
RUN_NAME="$(basename "$BAG_PATH" .bag)_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS_DIR/results/$RUN_NAME"
mkdir -p "$LOG_DIR"
CONFIG="$LOG_DIR/euroc_stereo_imu_config.yaml"
cp "$BASE_CONFIG" "$CONFIG"
cp "$(dirname "$BASE_CONFIG")"/cam*.yaml "$LOG_DIR"/
sed -i "s#^output_path:.*#output_path: \"$LOG_DIR/\"#" "$CONFIG"

cleanup() {
  set +e
  pkill -f "vins_node" || true
  pkill -f "loop_fusion_node" || true
  pkill -f "rosbag play" || true
  pkill -f "roscore" || true
}
trap cleanup EXIT

roscore >"$LOG_DIR/roscore.log" 2>&1 &
sleep 3

rosrun vins vins_node "$CONFIG" $VINS_EXTRA_ARGS >"$LOG_DIR/vins_node.log" 2>&1 &
sleep 3

if [ "$RVIZ" = "1" ]; then
  roslaunch vins vins_rviz.launch >"$LOG_DIR/rviz.log" 2>&1 &
  sleep 2
fi

if [ "$BAG_DURATION" = "0" ]; then
  rosbag play -r "$BAG_RATE" "$BAG_PATH" >"$LOG_DIR/rosbag_play.log" 2>&1
else
  rosbag play -r "$BAG_RATE" --duration="$BAG_DURATION" "$BAG_PATH" >"$LOG_DIR/rosbag_play.log" 2>&1
fi
sleep 5

echo "Run complete. Logs: $LOG_DIR"
echo "VINS trajectory: $LOG_DIR/vio.csv"
