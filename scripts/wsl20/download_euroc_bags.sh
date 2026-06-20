#!/usr/bin/env bash
set -euo pipefail

# Download EuRoC ROS bags for VINS-Fusion trajectory tests.
# Usage:
#   bash scripts/wsl20/download_euroc_bags.sh /mnt/f/datasets/EuRoC_bags MH_01_easy MH_05_difficult

DATA_ROOT="${1:-/mnt/f/datasets/EuRoC_bags}"
shift || true

if [ "$#" -eq 0 ]; then
  set -- MH_01_easy MH_05_difficult
fi

declare -A EUROC_BAG_PATHS=(
  [MH_01_easy]="machine_hall/MH_01_easy/MH_01_easy.bag"
  [MH_02_easy]="machine_hall/MH_02_easy/MH_02_easy.bag"
  [MH_03_medium]="machine_hall/MH_03_medium/MH_03_medium.bag"
  [MH_04_difficult]="machine_hall/MH_04_difficult/MH_04_difficult.bag"
  [MH_05_difficult]="machine_hall/MH_05_difficult/MH_05_difficult.bag"
  [V1_01_easy]="vicon_room1/V1_01_easy/V1_01_easy.bag"
  [V1_02_medium]="vicon_room1/V1_02_medium/V1_02_medium.bag"
  [V1_03_difficult]="vicon_room1/V1_03_difficult/V1_03_difficult.bag"
  [V2_01_easy]="vicon_room2/V2_01_easy/V2_01_easy.bag"
  [V2_02_medium]="vicon_room2/V2_02_medium/V2_02_medium.bag"
  [V2_03_difficult]="vicon_room2/V2_03_difficult/V2_03_difficult.bag"
)

BASE_URL="https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset"
HF_MIRROR_MH01="https://hf-mirror.com/datasets/kavehsgh/EuRoC_MAV_Dataset_Machine_Hall_Easy_01/resolve/main/MH_01_easy.bag?download=true"
mkdir -p "$DATA_ROOT"

for seq in "$@"; do
  rel="${EUROC_BAG_PATHS[$seq]:-}"
  if [ -z "$rel" ]; then
    echo "Unknown EuRoC sequence: $seq" >&2
    exit 2
  fi
  bag_path="$DATA_ROOT/$seq.bag"
  if [ -s "$bag_path" ]; then
    echo "Already downloaded: $bag_path"
    continue
  elif [ -f "$bag_path" ]; then
    echo "Removing empty or incomplete target before retry: $bag_path"
    rm -f "$bag_path"
  fi
  url="$BASE_URL/$rel"
  echo "Downloading $seq bag from $url"
  if command -v aria2c >/dev/null 2>&1; then
    downloader=(aria2c -x 8 -s 8 -k 4M --file-allocation=none --allow-overwrite=true -d "$DATA_ROOT" -o "$seq.bag")
    "${downloader[@]}" "$url" || {
      if [ "$seq" = "MH_01_easy" ]; then
        echo "Official download failed; trying hf-mirror MH_01_easy mirror."
        "${downloader[@]}" "$HF_MIRROR_MH01"
      else
        exit 1
      fi
    }
  else
    wget -c -O "$bag_path" "$url" || {
      if [ "$seq" = "MH_01_easy" ]; then
        echo "Official download failed; trying hf-mirror MH_01_easy mirror."
        wget -c -O "$bag_path" "$HF_MIRROR_MH01"
      else
        exit 1
      fi
    }
  fi
done
