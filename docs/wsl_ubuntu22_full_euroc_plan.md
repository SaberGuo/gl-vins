# WSL Ubuntu 22.04 + Full EuRoC Plan

## Current Machine Check

- WSL default distro: `Ubuntu-22.04`
- Version: Ubuntu 22.04.5 LTS
- WSL: WSL2
- GPU visible from WSL: NVIDIA GeForce RTX 5060 Laptop GPU
- Windows driver: 591.86

## Important ROS/VINS Constraint

VINS-Fusion is a ROS1 project. The most friction-free ROS1 setup is Ubuntu 20.04 + ROS Noetic. Ubuntu 22.04 does not have a normal official ROS1 Noetic target, so direct VINS-Fusion backend evaluation on 22.04 is not the cleanest route.

Recommended split:

1. **Ubuntu 22.04 WSL**: full EuRoC frontend tests with PyTorch GPU, LightGlue, ORB, CSV summaries.
2. **Ubuntu 20.04 WSL or Docker Noetic**: VINS-Fusion backend trajectory tests, ATE/RPE, estimator logs.

This keeps the learned frontend experiments reproducible on 22.04 while avoiding ROS1-on-22.04 dependency churn.

## Step 1: Setup Frontend Environment in Ubuntu 22.04

From Windows project root:

```powershell
wsl
cd /mnt/f/research/research/engineering/vins-lightglue-vio
bash scripts/wsl/setup_ubuntu22_frontend.sh
source .venv-wsl/bin/activate
```

Expected check:

```text
torch: 2.x+cu128
cuda: True
device: NVIDIA GeForce RTX 5060 Laptop GPU
```

## Step 2: Download Full EuRoC

Start with one easy and one difficult sequence:

```bash
cd /mnt/f/research/research/engineering/vins-lightglue-vio
bash scripts/wsl/download_euroc_full.sh /mnt/f/datasets/EuRoC MH_01_easy MH_05_difficult
```

Optional later:

```bash
bash scripts/wsl/download_euroc_full.sh /mnt/f/datasets/EuRoC MH_04_difficult V1_03_difficult
```

## Step 3: Run Full Frontend Evaluation

```bash
source .venv-wsl/bin/activate
bash scripts/wsl/run_full_euroc_frontend_eval.sh /mnt/f/datasets/EuRoC MH_01_easy MH_05_difficult
```

Summarize:

```bash
python scripts/wsl/summarize_frontend_runs.py \
  runs/euroc_MH_01_easy_lightglue_gpu_k1024/matches.csv \
  runs/euroc_MH_01_easy_orb_k1024/matches.csv \
  runs/euroc_MH_05_difficult_lightglue_gpu_k1024/matches.csv \
  runs/euroc_MH_05_difficult_orb_k1024/matches.csv
```

## Step 4: VINS-Fusion Backend Evaluation

Preferred choices:

- Use existing `Ubuntu-20.04` WSL distro for ROS Noetic.
- Or use a Docker Noetic image from inside WSL.

Evaluation target:

| System | Frontend | Backend | Dataset |
|---|---|---|---|
| Baseline | VINS feature_tracker | VINS-Fusion estimator | EuRoC full |
| Proposed | LightGlue feature tracker | VINS-Fusion estimator | EuRoC full |

Metrics:

- ATE RMSE via `evo_ape`.
- RPE via `evo_rpe`.
- Initialization success/failure.
- Feature count accepted by estimator.
- Track length and drop rate.
- Frontend runtime and estimator runtime.

## Risk Notes

- Pairwise LightGlue matches are not enough; VINS needs persistent feature tracks with stable IDs.
- VINS expects normalized camera coordinates and velocities in the feature message.
- LightGlue k1024 is around 17-18 Hz on the current GPU in the public sample test. Full sequence performance may differ.
- If real-time is not stable, test keyframe-only LightGlue plus KLT propagation between keyframes.
