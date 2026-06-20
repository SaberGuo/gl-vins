# VINS + LightGlue on EuRoC MH_01 in WSL20

Date: 2026-06-20

## Setup

- OS: WSL Ubuntu 20.04
- ROS: Noetic
- Dataset: `/mnt/f/datasets/EuRoC_bags/MH_01_easy.bag`
- Baseline: VINS-Fusion `euroc_stereo_imu_config.yaml`
- Learned frontend: `scripts/ros1/lightglue_feature_tracker_node.py`
- Feature message: `/feature_tracker/feature` as `sensor_msgs/PointCloud`
- VINS patch: `external_feature_only` disables the internal image frontend and consumes external feature messages.

## Important Fixes

- `rosNodeTest.cpp` now supports `_external_feature_only:=true`.
- `feature_callback` now checks `channels.size() > 8` before reading optional ground-truth channels. The original `> 5` condition segfaulted for normal six-channel VINS feature messages.
- The LightGlue runner supports:
  - `BAG_DURATION`
  - `BAG_RATE`
  - `MAX_KEYPOINTS`
  - `DEVICE`
  - `PUBLISH_STEREO`
- LightGlue node falls back to CPU when CUDA is visible but unusable.

## Baseline Result

Original VINS-Fusion frontend on full `MH_01_easy.bag`:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_20260619_235742`
- VIO rows: 1831
- Leica Sim(3) aligned RMSE: `0.193380 m`
- Mean: `0.180628 m`
- Median: `0.165925 m`
- P95: `0.316011 m`
- Max: `0.433639 m`

This is the reference result for same-bag comparison.

## LightGlue Smoke Test

Command:

```bash
BAG_DURATION=30 BAG_RATE=0.2 MAX_KEYPOINTS=128 DEVICE=auto PUBLISH_STEREO=1 \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_lightglue_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Result:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k128_20260620_003452`
- VIO rows: 96
- VINS initialized successfully.
- LightGlue frames logged: 53
- CPU frontend time: about `1.3-1.6 s/frame`
- Stereo matches: roughly `41-72` per processed frame
- Leica Sim(3) aligned RMSE on this short segment: `0.110588 m`

Interpretation: the ROS/VINS integration path works. This short, slowed-down segment is not a fair full-sequence comparison.

## Full Bag LightGlue Run

Command:

```bash
BAG_RATE=1.0 MAX_KEYPOINTS=128 DEVICE=auto PUBLISH_STEREO=1 \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_lightglue_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Result:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k128_20260620_003818`
- VIO rows: 121
- LightGlue frames logged: 66
- Leica aligned samples: 121
- Sim(3) scale: `0.000038`
- RMSE: `3.382848 m`
- Mean: `2.620185 m`
- Median: `1.838076 m`
- P95: `7.243967 m`
- Max: `9.913330 m`

Interpretation: this full-bag run is a failed comparison. VINS initializes, but the CPU LightGlue frontend is too slow and produces sparse, bursty feature updates. The estimator later diverges. This result should be used as a systems bottleneck finding, not as evidence that LightGlue features are worse than the original VINS frontend.

## GPU Status

The machine exposes an `NVIDIA GeForce RTX 5060 Laptop GPU`, but current WSL Python uses `torch==2.4.1+cu121`, which does not support CUDA capability `sm_120`.

Observed error:

```text
RuntimeError: CUDA error: no kernel image is available for execution on the device
```

Current runner therefore falls back to CPU. A fair test needs a PyTorch build that supports this GPU or a different supported CUDA GPU.

## GPU Fix and Full Bag Reruns

Installed a dedicated Python 3.9 environment at:

```bash
/home/gx/venvs/lightglue-cu128-py39
```

Key packages:

- `torch==2.8.0+cu128`
- `torchvision==0.23.0+cu128`
- `opencv-python-headless==4.11.0.86`
- `kornia==0.7.3`
- editable LightGlue from `/home/gx/src/LightGlue`

CUDA validation:

```text
torch 2.8.0+cu128
cuda available: True
device: NVIDIA GeForce RTX 5060 Laptop GPU
CUDA matmul: OK
SuperPoint CUDA: OK
```

Because ROS Noetic in this WSL distro uses Python 3.8, the LightGlue node was updated to avoid compiled `cv_bridge` and parse `sensor_msgs/Image` directly. The runner now accepts:

```bash
LIGHTGLUE_PYTHON=/home/gx/venvs/lightglue-cu128-py39/bin/python
```

### 30 Second GPU Smoke Tests

`MAX_KEYPOINTS=256`:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k256_20260620_085442`
- VIO rows: 329
- Sim(3) scale: `0.939282`
- RMSE: `0.055439 m`
- P95: `0.100791 m`

`MAX_KEYPOINTS=512`:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k512_20260620_090026`
- VIO rows: 294
- Sim(3) scale: `0.922347`
- RMSE: `0.050382 m`
- P95: `0.096861 m`

### Full Bag GPU Results

Command template:

```bash
BAG_RATE=1.0 MAX_KEYPOINTS=512 DEVICE=cuda PUBLISH_STEREO=1 \
LIGHTGLUE_PYTHON=/home/gx/venvs/lightglue-cu128-py39/bin/python \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_lightglue_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Full `MAX_KEYPOINTS=256`:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k256_20260620_085558`
- VIO rows: 2010
- Logged frontend samples: 90
- Logged frontend mean latency: `117.0 ms`
- Logged stereo matches mean/min/max: `152.3 / 112 / 188`
- Sim(3) scale: `0.792509`
- RMSE: `0.829410 m`
- Mean: `0.727135 m`
- Median: `0.665567 m`
- P95: `1.468729 m`
- Max: `2.033843 m`

Full `MAX_KEYPOINTS=512`:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_lightglue_k512_20260620_090144`
- VIO rows: 1923
- Logged frontend samples: 90
- Logged frontend mean latency: `152.2 ms`
- Logged stereo matches mean/min/max: `355.0 / 261 / 422`
- Sim(3) scale: `0.792605`
- RMSE: `0.612067 m`
- Mean: `0.535317 m`
- Median: `0.478543 m`
- P95: `1.059701 m`
- Max: `1.504681 m`

### Current Interpretation

The GPU support issue is resolved. LightGlue now runs on the RTX 5060 and VINS receives enough feature updates to complete the full EuRoC `MH_01_easy` bag.

However, the current LightGlue frontend is still worse than the original VINS-Fusion frontend on the full bag:

- Original VINS frontend RMSE: `0.193380 m`
- LightGlue k256 GPU RMSE: `0.829410 m`
- LightGlue k512 GPU RMSE: `0.612067 m`

The most likely remaining causes are frontend semantics and calibration parity, not raw GPU throughput:

- The current node uses OpenCV pinhole undistortion, while the VINS baseline uses MEI camera models.
- LightGlue matching is publishing relatively sparse temporal updates compared with KLT tracking behavior.
- Track-ID continuity and velocity estimates are still simple pairwise-match approximations, not a full VINS-style feature manager.

## Next Experiments

1. Replace the OpenCV pinhole normalization with MEI/camodocal-equivalent normalization for parity with the VINS baseline.
2. Add explicit publish-rate metrics and unthrottled frontend CSV logs, because ROS log throttling only records samples every two seconds.
3. Improve track management: retain longer temporal tracks, reject inconsistent stereo matches, and compute velocities from stable track history instead of one previous frame.
