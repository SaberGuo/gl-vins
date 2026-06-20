#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/noetic/setup.bash
source /home/gx/catkin_ws_vins/devel/setup.bash
set -u

LOG="${LOG:-/tmp/lg_node_gpu_smoke.log}"
BAG_PATH="${BAG_PATH:-/mnt/f/datasets/EuRoC_bags/MH_01_easy.bag}"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-/home/gx/opt/onnxruntime-linux-x64-gpu_cuda12-1.27.0}"
PYTORCH_NVIDIA_LIB_ROOT="${PYTORCH_NVIDIA_LIB_ROOT:-/home/gx/venvs/lightglue-cu128-py39/lib/python3.9/site-packages/nvidia}"

rm -f "$LOG" /tmp/lg_roscore.log /tmp/lg_bag.log

cleanup() {
  set +e
  [ -n "${NODE_PID:-}" ] && kill "$NODE_PID" 2>/dev/null || true
  [ -n "${BAG_PID:-}" ] && kill "$BAG_PID" 2>/dev/null || true
  [ -n "${ROSCORE_PID:-}" ] && kill "$ROSCORE_PID" 2>/dev/null || true
}
trap cleanup EXIT

roscore >/tmp/lg_roscore.log 2>&1 &
ROSCORE_PID=$!
sleep 2

export LD_LIBRARY_PATH="$ONNXRUNTIME_ROOT/lib:${LD_LIBRARY_PATH:-}"
if [ -d "$PYTORCH_NVIDIA_LIB_ROOT" ]; then
  for d in "$PYTORCH_NVIDIA_LIB_ROOT"/*/lib; do
    [ -d "$d" ] && export LD_LIBRARY_PATH="$d:${LD_LIBRARY_PATH:-}"
  done
fi

rosrun vins lightglue_recovery_candidate_node \
  _backend:=onnx \
  _use_cuda:=1 \
  _publish_every_n:=5 \
  _model_width:=376 \
  _model_height:=240 \
  _max_candidates:=80 \
  _pipeline_onnx:=/mnt/f/research/research/engineering/vins-lightglue-vio/third_party/models/superpoint_lightglue_pipeline.onnx \
  >"$LOG" 2>&1 &
NODE_PID=$!
sleep 3

rosbag play -r 0.2 --duration=4 "$BAG_PATH" >/tmp/lg_bag.log 2>&1 &
BAG_PID=$!
wait "$BAG_PID" || true
sleep "${POST_BAG_WAIT:-120}"

cat "$LOG"
