#!/usr/bin/env bash
set -euo pipefail

# Run a conservative degradation-aware recovery sweep on one EuRoC bag.
#
# Usage:
#   bash scripts/wsl20/run_degradation_recovery_sweep.sh ~/catkin_ws_vins /mnt/f/datasets/EuRoC_bags/MH_04_difficult.bag

WS_DIR="${1:-$HOME/catkin_ws_vins}"
BAG_PATH="${2:?Usage: run_degradation_recovery_sweep.sh WS_DIR BAG_PATH}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

run_case() {
  local name="$1"
  shift
  echo "=== degradation recovery case: $name ==="
  env "$@" bash "$SCRIPT_DIR/run_vins_cpp_recovery_bridge_euroc.sh" "$WS_DIR" "$BAG_PATH"
}

COMMON=(
  BACKEND=onnx
  USE_CUDA=1
  REQUEST_ONLY=1
  MODEL_WIDTH=376
  MODEL_HEIGHT=240
  MAX_CANDIDATES=40
  MIN_SCORE=0.0
  RECOVERY_REQUEST_TIMEOUT_MS=30
  RECOVERY_TIME_TOLERANCE=0.05
  ONNXRUNTIME_ROOT=/home/gx/opt/onnxruntime-linux-x64-gpu_cuda12-1.27.0
  PIPELINE_ONNX=/mnt/f/research/research/engineering/vins-lightglue-vio/third_party/models/superpoint_lightglue_pipeline.onnx
)

run_case no_geometry_gate \
  "${COMMON[@]}" \
  RECOVERY_MAX_CNT=3 \
  RECOVERY_PREV_MATCH_RADIUS=16.0 \
  RECOVERY_MIN_DIST_RATIO=0.2 \
  RECOVERY_REQUEST_MIN_TRACKS=80 \
  RECOVERY_REQUEST_LOST_RATIO=0.25 \
  RECOVERY_REQUEST_MAX_MEAN_FLOW=0 \
  RECOVERY_REQUEST_MIN_BLUR_SCORE=0 \
  RECOVERY_REQUEST_BRIGHTNESS_DELTA=0 \
  RECOVERY_MAX_FLOW_ERROR=0 \
  REQUEST_PREV_RADIUS=16.0 \
  REQUEST_DISTANCE_PENALTY=0.03

run_case geometry_gate_relaxed \
  "${COMMON[@]}" \
  RECOVERY_MAX_CNT=2 \
  RECOVERY_PREV_MATCH_RADIUS=14.0 \
  RECOVERY_MIN_DIST_RATIO=0.25 \
  RECOVERY_REQUEST_MIN_TRACKS=80 \
  RECOVERY_REQUEST_LOST_RATIO=0.25 \
  RECOVERY_REQUEST_MAX_MEAN_FLOW=40 \
  RECOVERY_REQUEST_MIN_BLUR_SCORE=15 \
  RECOVERY_REQUEST_BRIGHTNESS_DELTA=50 \
  RECOVERY_MAX_FLOW_ERROR=20 \
  RECOVERY_MIN_FLOW_TRACKS=6 \
  REQUEST_PREV_RADIUS=14.0 \
  REQUEST_DISTANCE_PENALTY=0.04
