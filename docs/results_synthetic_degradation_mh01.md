# Synthetic Degradation MH_01 Results

Date: 2026-06-20

## Scope

The official EuRoC download server timed out while downloading `MH_04_difficult`, so this run uses the existing `MH_01_easy.bag` to create 60 s synthetic degradation bags:

- `MH_01_easy_blur60.bag`: horizontal motion blur, kernel length 13.
- `MH_01_easy_exposure60.bag`: brightness scale 1.7 and gamma 0.8 after 10 s.
- `MH_01_easy_frameskip60.bag`: keep every second stereo image frame as a high-apparent-motion / low-frame-rate proxy.

These are not replacements for real difficult EuRoC sequences. They are first checks that degradation-aware request triggers fire and that the recovery path remains measurable.

## Commands

Create corrupted bags:

```bash
source /opt/ros/noetic/setup.bash
python3 scripts/wsl20/create_corrupted_euroc_bag.py \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy_blur60.bag \
  --mode motion_blur --duration 60 --blur-length 13 --blur-angle 0

python3 scripts/wsl20/create_corrupted_euroc_bag.py \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy_exposure60.bag \
  --mode exposure_jump --duration 60 --jump-after 10 --brightness-scale 1.7 --gamma 0.8

python3 scripts/wsl20/create_corrupted_euroc_bag.py \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy_frameskip60.bag \
  --mode frame_skip --duration 60 --keep-every 2
```

Run tests:

```bash
bash scripts/wsl20/run_synthetic_degradation_tests.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags
```

## Results

| Scenario | Mode | VIO samples | RMSE | P95 | Accepted total | Trigger reasons |
|---|---:|---:|---:|---:|---:|---|
| blur60 | baseline | 590 | `0.047551 m` | `0.092107 m` | 0 | none |
| blur60 | degradation-aware recovery | 590 | `0.048092 m` | `0.094852 m` | 733 | lost_ratio 270, track_count 72, blur 15, high_speed 6 |
| exposure60 | baseline | 590 | `0.049715 m` | `0.096848 m` | 0 | none |
| exposure60 | degradation-aware recovery | 590 | `0.050730 m` | `0.098914 m` | 645 | lost_ratio 236, track_count 79, brightness 1 |
| frameskip60 | baseline | 290 | `0.048581 m` | `0.096044 m` | 0 | none |
| frameskip60 | degradation-aware recovery | 290 | `0.048442 m` | `0.094146 m` | 564 | lost_ratio 205, track_count 89, high_speed 29 |

ONNX runtime summary for recovery runs:

| Scenario | Logged ONNX publish lines | Logged candidates | Median inference | Mean inference | Max inference |
|---|---:|---:|---:|---:|---:|
| blur60 | 15 | 209 | `27.5 ms` | `127.8 ms` | `1389.1 ms` |
| exposure60 | 13 | 232 | `27.6 ms` | `146.7 ms` | `1158.0 ms` |
| frameskip60 | 15 | 245 | `26.2 ms` | `116.3 ms` | `1178.0 ms` |

## Interpretation

- The new degradation triggers are active: blur and high-speed reasons appear in logs, and brightness jump is detected at the exposure transition.
- Current conservative-looking settings are still too permissive for blur/exposure on `MH_01_easy`: many recovery tracks are accepted, but trajectory RMSE and P95 slightly worsen.
- The frame-skip proxy shows a small improvement, which supports the idea that recovery may help when apparent inter-frame motion rises.
- `MH_01_easy` remains too easy for a strong robustness claim. The method needs harder real sequences and a geometry gate before being promoted.

## Next Changes

1. Add a candidate geometry gate before acceptance, starting with local flow consistency or F-matrix/RANSAC over candidate batches.
2. Reduce blur/exposure acceptance aggressiveness:
   - `RECOVERY_MAX_CNT=1-2`
   - larger `REQUEST_DISTANCE_PENALTY`
   - tighter `REQUEST_PREV_RADIUS`
3. Download or mirror real difficult EuRoC bags and rerun `MH_04`, `MH_05`, `V1_03`, and `V2_03`.
4. Treat accepted track count as a diagnostic only; trajectory metrics decide whether a setting is useful.
