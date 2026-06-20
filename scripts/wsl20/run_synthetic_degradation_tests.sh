#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${1:-$HOME/catkin_ws_vins}"
DATA_ROOT="${2:-/mnt/f/datasets/EuRoC_bags}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SEQUENCES=(
  MH_01_easy_blur60
  MH_01_easy_exposure60
  MH_01_easy_frameskip60
)

for seq in "${SEQUENCES[@]}"; do
  echo "=== baseline: $seq ==="
  BAG_RATE=1.0 bash "$SCRIPT_DIR/run_vins_fusion_euroc_baseline.sh" "$WS_DIR" "$DATA_ROOT/$seq.bag"

  echo "=== degradation-aware recovery: $seq ==="
  BACKEND=onnx \
  USE_CUDA=1 \
  REQUEST_ONLY=1 \
  MODEL_WIDTH=376 \
  MODEL_HEIGHT=240 \
  MAX_CANDIDATES=40 \
  MIN_SCORE=0.0 \
  RECOVERY_MAX_CNT=3 \
  RECOVERY_PREV_MATCH_RADIUS=14.0 \
  RECOVERY_MIN_DIST_RATIO=0.25 \
  RECOVERY_TIME_TOLERANCE=0.05 \
  RECOVERY_REQUEST_MIN_TRACKS=120 \
  RECOVERY_REQUEST_LOST_RATIO=0.10 \
  RECOVERY_REQUEST_TIMEOUT_MS=80 \
  RECOVERY_REQUEST_MAX_MEAN_FLOW=35 \
  RECOVERY_REQUEST_MIN_BLUR_SCORE=20 \
  RECOVERY_REQUEST_BRIGHTNESS_DELTA=30 \
  REQUEST_PREV_RADIUS=14.0 \
  REQUEST_DISTANCE_PENALTY=0.04 \
  ONNXRUNTIME_ROOT=/home/gx/opt/onnxruntime-linux-x64-gpu_cuda12-1.27.0 \
  PIPELINE_ONNX=/mnt/f/research/research/engineering/vins-lightglue-vio/third_party/models/superpoint_lightglue_pipeline.onnx \
  BAG_RATE=1.0 \
  bash "$SCRIPT_DIR/run_vins_cpp_recovery_bridge_euroc.sh" "$WS_DIR" "$DATA_ROOT/$seq.bag"
done
