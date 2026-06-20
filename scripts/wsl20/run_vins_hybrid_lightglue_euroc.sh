#!/usr/bin/env bash
set -euo pipefail

# Run VINS-Fusion with a hybrid KLT + LightGlue recovery frontend on a EuRoC rosbag.
# Usage:
#   LIGHTGLUE_PYTHON=/home/gx/venvs/lightglue-cu128-py39/bin/python \
#   bash scripts/wsl20/run_vins_hybrid_lightglue_euroc.sh ~/catkin_ws_vins /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag

WS_DIR="${1:-$HOME/catkin_ws_vins}"
BAG_PATH="${2:?Usage: run_vins_hybrid_lightglue_euroc.sh WS_DIR BAG_PATH}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HYBRID_NODE="$PROJECT_DIR/scripts/ros1/hybrid_klt_lightglue_feature_tracker_node.py"

MAX_CNT="${MAX_CNT:-180}"
MIN_DIST="${MIN_DIST:-30}"
KLT_MIN_TRACKS="${KLT_MIN_TRACKS:-90}"
LOW_PARALLAX_PX="${LOW_PARALLAX_PX:-1.0}"
RECOVERY_INTERVAL="${RECOVERY_INTERVAL:-10}"
MAX_RECOVERIES_PER_FRAME="${MAX_RECOVERIES_PER_FRAME:-80}"
PUBLISH_EVERY="${PUBLISH_EVERY:-2}"
MAX_KEYPOINTS="${MAX_KEYPOINTS:-512}"
MATCH_CONFIDENCE="${MATCH_CONFIDENCE:-0.1}"
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
RUN_NAME="$(basename "$BAG_PATH" .bag)_hybrid_lg_k${MAX_KEYPOINTS}_m${MAX_CNT}_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS_DIR/results/$RUN_NAME"
mkdir -p "$LOG_DIR"
CONFIG="$LOG_DIR/euroc_stereo_imu_config.yaml"
cp "$BASE_CONFIG" "$CONFIG"
cp "$(dirname "$BASE_CONFIG")"/cam*.yaml "$LOG_DIR"/
sed -i "s#^output_path:.*#output_path: \"$LOG_DIR/\"#" "$CONFIG"

cleanup() {
  set +e
  pkill -f "hybrid_klt_lightglue_feature_tracker_node.py" || true
  pkill -f "vins_node" || true
  pkill -f "rosbag play" || true
  pkill -f "roscore" || true
}
trap cleanup EXIT

roscore >"$LOG_DIR/roscore.log" 2>&1 &
sleep 3

PYTHONUNBUFFERED=1 "$LIGHTGLUE_PYTHON" "$HYBRID_NODE" \
  _image0_topic:=/cam0/image_raw \
  _image1_topic:=/cam1/image_raw \
  _feature_topic:=/feature_tracker/feature \
  _cam0_calib:="$LOG_DIR/cam0_pinhole.yaml" \
  _cam1_calib:="$LOG_DIR/cam1_pinhole.yaml" \
  _max_cnt:="$MAX_CNT" \
  _min_dist:="$MIN_DIST" \
  _klt_min_tracks:="$KLT_MIN_TRACKS" \
  _low_parallax_px:="$LOW_PARALLAX_PX" \
  _recovery_interval:="$RECOVERY_INTERVAL" \
  _max_recoveries_per_frame:="$MAX_RECOVERIES_PER_FRAME" \
  _publish_every:="$PUBLISH_EVERY" \
  _max_num_keypoints:="$MAX_KEYPOINTS" \
  _match_confidence:="$MATCH_CONFIDENCE" \
  _publish_stereo:="$PUBLISH_STEREO" \
  _device:="$DEVICE" \
  >"$LOG_DIR/hybrid_feature_tracker.log" 2>&1 &
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
