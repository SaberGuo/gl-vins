# Public EuRoC Sample Frontend Test - 2026-06-19

## Dataset

- Source: public GitHub sample `Realle0815/ORB_SLAM3-ROS2/TEST_DATASET/sample_euroc_MH05`
- Local path: `data/public_samples/sample_euroc_MH05/mav0`
- Camera: `cam0`
- Images downloaded: 30
- Adjacent pairs evaluated: 20
- Image resize: long edge 752
- Keypoint budget: 1024
- Runtime environment: Windows, Python 3.13.12, RTX 5060 Laptop GPU, driver 591.86, PyTorch 2.11.0+cu128

This is a frontend smoke test, not a full VINS trajectory benchmark. It checks whether the learned frontend can run on public EuRoC-format images and compares pairwise match density/runtime against ORB.

## Commands

```powershell
cd F:\research\research\engineering\vins-lightglue-vio
.\scripts\download_public_euroc_sample.ps1 -ImageCount 30
.\scripts\run_public_sample_frontend_eval.ps1 -MaxPairs 20
.\scripts\run_gpu_keypoint_sweep.ps1 -MaxPairs 20
```

## Initial CPU Results

| Frontend | Pairs | Mean matches | Median matches | Mean frontend time |
|---|---:|---:|---:|---:|
| SuperPoint + LightGlue | 20 | 786.0 | 788.0 | 2355.8 ms |
| ORB + BF ratio matching | 20 | 550.8 | 519.5 | 30.8 ms |

## GPU Results

For GPU runs, the first pair includes CUDA/model warm-up. `Stable mean ms` excludes the first pair and is the more useful runtime estimate.

| Frontend | Pairs | Mean matches | Median matches | Mean ms all | Stable mean ms | Stable Hz |
|---|---:|---:|---:|---:|---:|---:|
| LightGlue GPU k1024 | 20 | 786.95 | 789.00 | 114.35 | 56.11 | 17.82 |
| LightGlue GPU k512 | 20 | 416.60 | 412.00 | 106.22 | 53.73 | 18.61 |
| LightGlue GPU k256 | 20 | 198.75 | 195.00 | 162.32 | 71.65 | 13.96 |
| ORB CPU k1024 | 20 | 550.80 | 519.50 | 30.84 | 30.84 | 32.43 |

Output CSVs:

- `runs/public_sample_euroc_mh05_lightglue/matches.csv`
- `runs/public_sample_euroc_mh05_lightglue_gpu/matches.csv`
- `runs/public_sample_euroc_mh05_lightglue_gpu_k512/matches.csv`
- `runs/public_sample_euroc_mh05_lightglue_gpu_k256/matches.csv`
- `runs/public_sample_euroc_mh05_orb/matches.csv`

## Interpretation

- LightGlue produced about 43% more pairwise matches than ORB on this small EuRoC sample.
- CPU-only runtime is not suitable for real-time VINS: about 2.36 seconds per image pair.
- GPU runtime with 1024 keypoints is much better: stable runtime is about 56.1 ms/pair, roughly 17.8 Hz.
- ORB is still faster on CPU at about 30.8 ms/pair, but it has fewer matches.
- Reducing LightGlue to 512 keypoints roughly halves matches but does not materially improve stable runtime on this small test; fixed model/extractor overhead dominates.
- The 256-keypoint run was slower than 512 in this short sample, likely due to run variance/warm-up/overhead rather than matching complexity alone. Treat it as inconclusive until repeated on a longer sequence.
- The next meaningful test is not more pairwise matching; it is a ROS/VINS trajectory test where persistent track IDs, normalized coordinates, IMU timing, and backend acceptance determine whether extra matches improve ATE/RPE.

## Next Test

Run a Linux/ROS1 VINS-Fusion baseline on full EuRoC `MH_01_easy` and `MH_05_difficult`, then replace VINS `feature_tracker` with the LightGlue ROS node and compare:

- ATE/RPE via `evo_ape` / `evo_rpe`
- estimator initialization success
- accepted feature count
- feature track length
- frontend + backend runtime
- failure/reset events
