#!/usr/bin/env bash
set -euo pipefail

# Download full EuRoC sequences for frontend/VINS tests.
# Usage:
#   bash scripts/wsl/download_euroc_full.sh /mnt/f/datasets/EuRoC MH_01_easy MH_05_difficult

DATA_ROOT="${1:-/mnt/f/datasets/EuRoC}"
shift || true

if [ "$#" -eq 0 ]; then
  set -- MH_01_easy MH_05_difficult
fi

declare -A EUROC_PATHS=(
  [MH_01_easy]="machine_hall/MH_01_easy/MH_01_easy.zip"
  [MH_02_easy]="machine_hall/MH_02_easy/MH_02_easy.zip"
  [MH_03_medium]="machine_hall/MH_03_medium/MH_03_medium.zip"
  [MH_04_difficult]="machine_hall/MH_04_difficult/MH_04_difficult.zip"
  [MH_05_difficult]="machine_hall/MH_05_difficult/MH_05_difficult.zip"
  [V1_01_easy]="vicon_room1/V1_01_easy/V1_01_easy.zip"
  [V1_02_medium]="vicon_room1/V1_02_medium/V1_02_medium.zip"
  [V1_03_difficult]="vicon_room1/V1_03_difficult/V1_03_difficult.zip"
  [V2_01_easy]="vicon_room2/V2_01_easy/V2_01_easy.zip"
  [V2_02_medium]="vicon_room2/V2_02_medium/V2_02_medium.zip"
  [V2_03_difficult]="vicon_room2/V2_03_difficult/V2_03_difficult.zip"
)

BASE_URL="https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
mkdir -p "$DATA_ROOT"

for seq in "$@"; do
  rel="${EUROC_PATHS[$seq]:-}"
  if [ -z "$rel" ]; then
    echo "Unknown EuRoC sequence: $seq" >&2
    exit 2
  fi
  zip_path="$DATA_ROOT/$seq.zip"
  seq_dir="$DATA_ROOT/$seq"
  url="$BASE_URL/$rel"

  if [ -d "$seq_dir/mav0" ]; then
    echo "Already extracted: $seq_dir"
    continue
  fi

  if [ ! -f "$zip_path" ]; then
    echo "Downloading $seq from $url"
    wget -c -O "$zip_path" "$url"
  fi

  mkdir -p "$seq_dir"
  unzip -n "$zip_path" -d "$seq_dir"
  if [ -d "$seq_dir/mav0" ]; then
    echo "Ready: $seq_dir/mav0"
  else
    echo "Extracted but mav0 not found under $seq_dir; inspect archive layout." >&2
  fi
done
