# WSL Ubuntu 20.04 + ROS Noetic + VINS-Fusion Plan

## Current Check

- Distro: Ubuntu 20.04.6 LTS
- WSL: WSL2
- GPU visible: NVIDIA GeForce RTX 5060 Laptop GPU
- OpenCV dev package: 4.2.0 available
- ROS state: ROS Noetic is installed; `/opt/ros/noetic/setup.bash`, `roscore`, `rosbag`, and `catkin_make` are available.
- VINS-Fusion build state: built successfully in `/home/gx/catkin_ws_vins` after applying OpenCV4 compatibility shims and using `catkin_make -j4`.
- EuRoC bag state: `MH_01_easy.bag` downloaded successfully via `hf-mirror.com` to `/mnt/f/datasets/EuRoC_bags/MH_01_easy.bag`; `rosbag info` verifies a 186 s bag with `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`, and `/leica/position`.

## Why Ubuntu 20.04

VINS-Fusion is a ROS1 project. Ubuntu 20.04 + ROS Noetic is the clean target for building and running it. Ubuntu 22.04 should remain the PyTorch/LightGlue frontend environment unless using Docker/compatibility workarounds.

## Step 1: Install ROS Noetic and VINS Dependencies

Inside `Ubuntu-20.04`:

```bash
cd /mnt/f/research/research/engineering/vins-lightglue-vio
bash scripts/wsl20/install_ros_noetic_vins_deps.sh
```

This needs your sudo password. It installs ROS Noetic, Ceres, Eigen, OpenCV, cv_bridge, image_transport, tf, catkin tools, and related build dependencies.

Verify:

```bash
source /opt/ros/noetic/setup.bash
roscore --version
catkin_make --version
```

## Step 2: Build VINS-Fusion

```bash
cd /mnt/f/research/research/engineering/vins-lightglue-vio
source /opt/ros/noetic/setup.bash
bash scripts/wsl20/prepare_vins_fusion_ws.sh ~/catkin_ws_vins
```

The script prefers the locally extracted VINS-Fusion source under `data/public_samples/vins_zip_extract/VINS-Fusion-master`, which avoids WSL GitHub clone timeouts. It also defaults to `CATKIN_JOBS=4` because the default `catkin_make -j24` was too slow/noisy in WSL.

## Step 3: Download EuRoC ROS Bags

```bash
bash scripts/wsl20/download_euroc_bags.sh /mnt/f/datasets/EuRoC_bags MH_01_easy MH_05_difficult
```

Use rosbag for the VINS-Fusion baseline because it preserves ROS topics and timing. Use the extracted `mav0` zip layout for frontend-only LightGlue/ORB CSV tests.

If the official ETH rosbag URL times out, `download_euroc_bags.sh` now falls back to the `hf-mirror.com` mirror for `MH_01_easy`.

## Step 4: Run VINS-Fusion Baseline

```bash
bash scripts/wsl20/run_vins_fusion_euroc_baseline.sh \
  ~/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Optional RViz:

```bash
RVIZ=1 bash scripts/wsl20/run_vins_fusion_euroc_baseline.sh \
  ~/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

## Step 5: ATE/RPE Evaluation

Install `evo`:

```bash
bash scripts/wsl20/install_evo.sh
export PATH="$HOME/.local/bin:$PATH"
```

The exact VINS output trajectory path depends on `config/euroc/euroc_stereo_imu_config.yaml`. After the first run, inspect the VINS output file and align it against EuRoC ground truth.

## Step 6: LightGlue Backend Integration

After baseline VINS-Fusion works:

1. Replace VINS `feature_tracker` with `scripts/ros1/lightglue_feature_tracker_node.py`.
2. Publish VINS-style `sensor_msgs/PointCloud` on the expected feature topic.
3. Keep estimator, IMU topics, and EuRoC bag playback unchanged.
4. Compare baseline vs LightGlue frontend with the same bag and config.

## Immediate Blocker

No current blocker for the `MH_01_easy` baseline smoke test. The next blocker is full pose ATE/RPE: the rosbag contains `/leica/position` but not the full EuRoC ground-truth pose CSV, so current evaluation is position-only against Leica after Sim(3) alignment.

## 2026-06-19 Build Log Summary

- Installed/verified ROS Noetic dependency path in Ubuntu 20.04.
- Copied VINS-Fusion from the local zip extraction instead of cloning from GitHub.
- Added `opencv4_compat.h` plus CMake force-include hooks for `camera_models`, `loop_fusion`, and `vins_estimator`.
- Verified successful build targets:
  - `/home/gx/catkin_ws_vins/devel/lib/vins/vins_node`
  - `/home/gx/catkin_ws_vins/devel/lib/loop_fusion/loop_fusion_node`
  - `/home/gx/catkin_ws_vins/devel/lib/global_fusion/global_fusion_node`
  - `/home/gx/catkin_ws_vins/devel/lib/vins/kitti_odom_test`
  - `/home/gx/catkin_ws_vins/devel/lib/vins/kitti_gps_test`

## 2026-06-19 Baseline Result

- Downloaded and verified `/mnt/f/datasets/EuRoC_bags/MH_01_easy.bag`.
- `rosbag info`: duration 186 s, 47,283 messages, topics `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`, `/leica/position`.
- VINS-Fusion run: `/home/gx/catkin_ws_vins/results/MH_01_easy_20260619_235742`.
- Output trajectory: `/home/gx/catkin_ws_vins/results/MH_01_easy_20260619_235742/vio.csv`.
- VINS initialized successfully and wrote 1,831 trajectory rows.
- Leica-position smoke metric from `scripts/wsl20/evaluate_vins_leica_position.py`:
  - Sim(3) scale: 1.046527
  - RMSE: 0.193380 m
  - mean: 0.180628 m
  - median: 0.165925 m
  - p95: 0.316011 m
  - max: 0.433639 m
