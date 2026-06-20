#!/usr/bin/env bash
set -euo pipefail

# Run VINS-Fusion with the C++ internal recovery bridge enabled and a candidate
# publisher node. The default candidate backend is fake_grid for bridge dry-run.
#
# Usage:
#   BACKEND=fake_grid BAG_DURATION=30 bash scripts/wsl20/run_vins_cpp_recovery_bridge_euroc.sh \
#     ~/catkin_ws_vins /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag

WS_DIR="${1:-$HOME/catkin_ws_vins}"
BAG_PATH="${2:?Usage: run_vins_cpp_recovery_bridge_euroc.sh WS_DIR BAG_PATH}"
BACKEND="${BACKEND:-fake_grid}"
RVIZ="${RVIZ:-0}"
BAG_DURATION="${BAG_DURATION:-0}"
BAG_RATE="${BAG_RATE:-1.0}"
RECOVERY_MAX_CNT="${RECOVERY_MAX_CNT:-30}"
RECOVERY_PREV_MATCH_RADIUS="${RECOVERY_PREV_MATCH_RADIUS:-6.0}"
RECOVERY_MIN_DIST_RATIO="${RECOVERY_MIN_DIST_RATIO:-0.2}"
RECOVERY_TIME_TOLERANCE="${RECOVERY_TIME_TOLERANCE:-0.02}"
RECOVERY_IMAGE_DELAY_MS="${RECOVERY_IMAGE_DELAY_MS:-0}"
RECOVERY_REQUEST_MIN_TRACKS="${RECOVERY_REQUEST_MIN_TRACKS:-80}"
RECOVERY_REQUEST_LOST_RATIO="${RECOVERY_REQUEST_LOST_RATIO:-0.35}"
RECOVERY_REQUEST_TIMEOUT_MS="${RECOVERY_REQUEST_TIMEOUT_MS:-0}"
RECOVERY_REQUEST_MAX_MEAN_FLOW="${RECOVERY_REQUEST_MAX_MEAN_FLOW:-0}"
RECOVERY_REQUEST_MIN_BLUR_SCORE="${RECOVERY_REQUEST_MIN_BLUR_SCORE:-0}"
RECOVERY_REQUEST_BRIGHTNESS_DELTA="${RECOVERY_REQUEST_BRIGHTNESS_DELTA:-0}"
PUBLISH_EVERY_N="${PUBLISH_EVERY_N:-1}"
FAKE_GRID_STEP="${FAKE_GRID_STEP:-5}"
MAX_CANDIDATES="${MAX_CANDIDATES:-20000}"
SUPERPOINT_ONNX="${SUPERPOINT_ONNX:-}"
LIGHTGLUE_ONNX="${LIGHTGLUE_ONNX:-}"
PIPELINE_ONNX="${PIPELINE_ONNX:-/mnt/f/research/research/engineering/vins-lightglue-vio/third_party/models/superpoint_lightglue_pipeline.onnx}"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-/home/gx/opt/onnxruntime-linux-x64-1.27.0}"
MODEL_WIDTH="${MODEL_WIDTH:-0}"
MODEL_HEIGHT="${MODEL_HEIGHT:-0}"
MIN_SCORE="${MIN_SCORE:-0.0}"
USE_CUDA="${USE_CUDA:-0}"
CUDA_DEVICE_ID="${CUDA_DEVICE_ID:-0}"
PYTORCH_NVIDIA_LIB_ROOT="${PYTORCH_NVIDIA_LIB_ROOT:-/home/gx/venvs/lightglue-cu128-py39/lib/python3.9/site-packages/nvidia}"
REQUEST_ONLY="${REQUEST_ONLY:-1}"
REQUEST_PREV_RADIUS="${REQUEST_PREV_RADIUS:-12.0}"
REQUEST_DISTANCE_PENALTY="${REQUEST_DISTANCE_PENALTY:-0.03}"
REQUEST_TIME_TOLERANCE="${REQUEST_TIME_TOLERANCE:-0.02}"

if [ -d "$ONNXRUNTIME_ROOT/lib" ]; then
  export LD_LIBRARY_PATH="$ONNXRUNTIME_ROOT/lib:${LD_LIBRARY_PATH:-}"
fi
if [ "$USE_CUDA" = "1" ] && [ -d "$PYTORCH_NVIDIA_LIB_ROOT" ]; then
  for d in "$PYTORCH_NVIDIA_LIB_ROOT"/*/lib; do
    [ -d "$d" ] && export LD_LIBRARY_PATH="$d:${LD_LIBRARY_PATH:-}"
  done
fi

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
RUN_NAME="$(basename "$BAG_PATH" .bag)_cpp_recovery_${BACKEND}_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS_DIR/results/$RUN_NAME"
mkdir -p "$LOG_DIR"
CONFIG="$LOG_DIR/euroc_stereo_imu_config.yaml"
cp "$BASE_CONFIG" "$CONFIG"
cp "$(dirname "$BASE_CONFIG")"/cam*.yaml "$LOG_DIR"/
sed -i "s#^output_path:.*#output_path: \"$LOG_DIR/\"#" "$CONFIG"

cleanup() {
  set +e
  pkill -f "lightglue_recovery_candidate_node" || true
  pkill -f "vins_node" || true
  pkill -f "loop_fusion_node" || true
  pkill -f "rosbag play" || true
  pkill -f "roscore" || true
}
trap cleanup EXIT

roscore >"$LOG_DIR/roscore.log" 2>&1 &
sleep 3

rosrun vins vins_node "$CONFIG" \
  _enable_lightglue_recovery_bridge:=true \
  _recovery_max_cnt:="$RECOVERY_MAX_CNT" \
  _recovery_time_tolerance:="$RECOVERY_TIME_TOLERANCE" \
  _recovery_prev_match_radius:="$RECOVERY_PREV_MATCH_RADIUS" \
  _recovery_min_dist_ratio:="$RECOVERY_MIN_DIST_RATIO" \
  _recovery_image_delay_ms:="$RECOVERY_IMAGE_DELAY_MS" \
  _recovery_request_min_tracks:="$RECOVERY_REQUEST_MIN_TRACKS" \
  _recovery_request_lost_ratio:="$RECOVERY_REQUEST_LOST_RATIO" \
  _recovery_request_timeout_ms:="$RECOVERY_REQUEST_TIMEOUT_MS" \
  _recovery_request_max_mean_flow:="$RECOVERY_REQUEST_MAX_MEAN_FLOW" \
  _recovery_request_min_blur_score:="$RECOVERY_REQUEST_MIN_BLUR_SCORE" \
  _recovery_request_brightness_delta:="$RECOVERY_REQUEST_BRIGHTNESS_DELTA" \
  >"$LOG_DIR/vins_node.log" 2>&1 &
sleep 2

rosrun vins lightglue_recovery_candidate_node \
  _backend:="$BACKEND" \
  _image_topic:=/cam0/image_raw \
  _output_topic:=/feature_tracker/recovery_candidates \
  _publish_every_n:="$PUBLISH_EVERY_N" \
  _fake_grid_step:="$FAKE_GRID_STEP" \
  _max_candidates:="$MAX_CANDIDATES" \
  _superpoint_onnx:="$SUPERPOINT_ONNX" \
  _lightglue_onnx:="$LIGHTGLUE_ONNX" \
  _pipeline_onnx:="$PIPELINE_ONNX" \
  _model_width:="$MODEL_WIDTH" \
  _model_height:="$MODEL_HEIGHT" \
  _min_score:="$MIN_SCORE" \
  _use_cuda:="$USE_CUDA" \
  _cuda_device_id:="$CUDA_DEVICE_ID" \
  _request_only:="$REQUEST_ONLY" \
  _request_prev_radius:="$REQUEST_PREV_RADIUS" \
  _request_distance_penalty:="$REQUEST_DISTANCE_PENALTY" \
  _request_time_tolerance:="$REQUEST_TIME_TOLERANCE" \
  >"$LOG_DIR/lightglue_recovery_candidate_node.log" 2>&1 &
sleep 2

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
echo "Accepted recovery lines:"
grep -h "LightGlue recovery bridge accepted" "$LOG_DIR/vins_node.log" || true
