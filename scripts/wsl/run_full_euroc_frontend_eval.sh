#!/usr/bin/env bash
set -euo pipefail

# Run LightGlue GPU and ORB frontend tests on full EuRoC sequences.
# Usage:
#   source .venv-wsl/bin/activate
#   bash scripts/wsl/run_full_euroc_frontend_eval.sh /mnt/f/datasets/EuRoC MH_01_easy MH_05_difficult

DATA_ROOT="${1:-/mnt/f/datasets/EuRoC}"
shift || true

if [ "$#" -eq 0 ]; then
  set -- MH_01_easy MH_05_difficult
fi

export PYTHONPATH="$(pwd)/src"

for seq in "$@"; do
  sequence_root="$DATA_ROOT/$seq/mav0"
  if [ ! -d "$sequence_root" ]; then
    echo "Missing sequence root: $sequence_root" >&2
    exit 2
  fi

  echo "Running frontend evaluation for $seq"
  python -m vins_lightglue.cli match \
    --dataset-config configs/datasets/euroc.yaml \
    --sequence-root "$sequence_root" \
    --camera cam0 \
    --resize-long-edge 752 \
    --max-num-keypoints 1024 \
    --device cuda \
    --output "runs/euroc_${seq}_lightglue_gpu_k1024/matches.csv"

  python -m vins_lightglue.cli orb-match \
    --dataset-config configs/datasets/euroc.yaml \
    --sequence-root "$sequence_root" \
    --camera cam0 \
    --resize-long-edge 752 \
    --max-num-keypoints 1024 \
    --output "runs/euroc_${seq}_orb_k1024/matches.csv"
done
