#!/usr/bin/env bash
set -euo pipefail

# Run VINS-Fusion with an external LightGlue frontend on a EuRoC rosbag.
# Usage:
#   bash scripts/wsl20/run_vins_lightglue_euroc.sh ~/catkin_ws_vins /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag

WS_DIR="${1:-$HOME/catkin_ws_vins}"
BAG_PATH="${2:?Usage: run_vins_lightglue_euroc.sh WS_DIR BAG_PATH}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LIGHTGLUE_NODE="$PROJECT_DIR/scripts/ros1/lightglue_feature_tracker_node.py"

MAX_KEYPOINTS="${MAX_KEYPOINTS:-512}"
MATCH_CONFIDENCE="${MATCH_CONFIDENCE:-0.1}"
RESIZE_LONG_EDGE="${RESIZE_LONG_EDGE:-0}"
PUBLISH_STEREO="${PUBLISH_STEREO:-1}"
DEVICE="${DEVICE:-auto}"
BAG_DURATION="${BAG_DURATION:-0}"
BAG_RATE="${BAG_RATE:-1.0}"
LIGHTGLUE_PYTHON="${LIGHTGLUE_PYTHON:-python3}"

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
RUN_NAME="$(basename "$BAG_PATH" .bag)_lightglue_k${MAX_KEYPOINTS}_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS_DIR/results/$RUN_NAME"
mkdir -p "$LOG_DIR"
CONFIG="$LOG_DIR/euroc_stereo_imu_config.yaml"
cp "$BASE_CONFIG" "$CONFIG"
cp "$(dirname "$BASE_CONFIG")"/cam*.yaml "$LOG_DIR"/
sed -i "s#^output_path:.*#output_path: \"$LOG_DIR/\"#" "$CONFIG"

cleanup() {
  set +e
  pkill -f "lightglue_feature_tracker_node.py" || true
  pkill -f "vins_node" || true
  pkill -f "rosbag play" || true
  pkill -f "roscore" || true
}
trap cleanup EXIT

roscore >"$LOG_DIR/roscore.log" 2>&1 &
sleep 3

PYTHONUNBUFFERED=1 "$LIGHTGLUE_PYTHON" "$LIGHTGLUE_NODE" \
  _image0_topic:=/cam0/image_raw \
  _image1_topic:=/cam1/image_raw \
  _feature_topic:=/feature_tracker/feature \
  _cam0_calib:="$LOG_DIR/cam0_pinhole.yaml" \
  _cam1_calib:="$LOG_DIR/cam1_pinhole.yaml" \
  _max_num_keypoints:="$MAX_KEYPOINTS" \
  _match_confidence:="$MATCH_CONFIDENCE" \
  _resize_long_edge:="$RESIZE_LONG_EDGE" \
  _publish_stereo:="$PUBLISH_STEREO" \
  _device:="$DEVICE" \
  >"$LOG_DIR/lightglue_feature_tracker.log" 2>&1 &
sleep 8

rosrun vins vins_node "$CONFIG" _external_feature_only:=true >"$LOG_DIR/vins_node.log" 2>&1 &
sleep 3

if [ "$BAG_DURATION" = "0" ]; then
  rosbag play -r "$BAG_RATE" "$BAG_PATH" >"$LOG_DIR/rosbag_play.log" 2>&1
else
  rosbag play -r "$BAG_RATE" --duration="$BAG_DURATION" "$BAG_PATH" >"$LOG_DIR/rosbag_play.log" 2>&1
fi
sleep 5

echo "Run complete. Logs: $LOG_DIR"
echo "VINS trajectory: $LOG_DIR/vio.csv"
