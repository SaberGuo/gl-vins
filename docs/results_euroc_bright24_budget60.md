# EuRoC Bright24 Exposure Budget60 Recovery Evaluation

Date: 2026-06-21

Generated bags:

```text
/mnt/f/datasets/EuRoC_bags/exposure_eval/V1_03_difficult_bright24.bag
/mnt/f/datasets/EuRoC_bags/exposure_eval/MH_04_difficult_bright24.bag
/mnt/f/datasets/EuRoC_bags/exposure_eval/MH_05_difficult_bright24.bag
```

Corruption:

```bash
--mode exposure_jump
--duration 0
--jump-after 10
--brightness-scale 2.4
--brightness-offset 0
--gamma 1.0
```

Recovery policy:

```bash
RECOVERY_MAX_CNT=2
RECOVERY_REQUEST_MIN_TRACKS=80
RECOVERY_REQUEST_LOST_RATIO=0.25
RECOVERY_REQUEST_TIMEOUT_MS=30
RECOVERY_REQUEST_COOLDOWN_MS=200
RECOVERY_REQUEST_MIN_SIGNAL_COUNT=2
RECOVERY_REQUEST_REQUIRE_DEGRADATION_SIGNAL=true
RECOVERY_REQUEST_MAX_MEAN_FLOW=40
RECOVERY_REQUEST_MIN_BLUR_SCORE=15
RECOVERY_REQUEST_BRIGHTNESS_DELTA=50
RECOVERY_MAX_FLOW_ERROR=20
RECOVERY_MIN_FLOW_TRACKS=6
MAX_INFERENCE_MS=60
DEPLETION_GRID_COLS=0
```

## Results

| sequence | mode | run dir | GT topic | VIO samples | aligned samples | requests | accepted tracks | ATE RMSE m | ATE p95 m | interpretation |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|---|
| V1_03 bright24 | baseline | `/home/gx/catkin_ws_vins/results/V1_03_difficult_bright24_20260621_090252` | `/vicon/firefly_sbx/firefly_sbx` | 1064 | 1056 | 0 | 0 | 0.124362 | 0.196020 | strong brightening worsens baseline relative to bright16 and original |
| V1_03 bright24 | budget60 recovery | `/home/gx/catkin_ws_vins/results/V1_03_difficult_bright24_cpp_recovery_onnx_20260621_090506` | `/vicon/firefly_sbx/firefly_sbx` | 1063 | 1056 | 27 | 4 | 0.121188 | 0.191660 | recovery improves RMSE by 2.55% vs bright24 baseline |
| MH_04 bright24 | baseline | `/home/gx/catkin_ws_vins/results/MH_04_difficult_bright24_20260621_090717` | `/leica/position` | 1006 | 998 | 0 | 0 | 0.371403 | 0.662565 | brightening still improves original MH_04 baseline |
| MH_04 bright24 | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_04_difficult_bright24_cpp_recovery_onnx_20260621_090926` | `/leica/position` | 1005 | 997 | 1 | 0 | 0.405235 | 0.661912 | no accepted recovery; trigger/wait perturbation worsens RMSE |
| MH_05 bright24 | baseline | `/home/gx/catkin_ws_vins/results/MH_05_difficult_bright24_20260621_091128` | `/leica/position` | 1126 | 1120 | 0 | 0 | 0.337771 | 0.599860 | strong brightening slightly worsens original MH_05 baseline |
| MH_05 bright24 | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_05_difficult_bright24_cpp_recovery_onnx_20260621_091348` | `/leica/position` | 1125 | 1120 | 0 | 0 | 0.352116 | 0.598405 | no accepted recovery; worse RMSE likely due run-to-run/timing perturbation |

## Findings

- `brightness_scale=2.4` is a better over-brightness stress test than `1.6` for V1_03: baseline RMSE increases from the original 0.121651 m to 0.124362 m.
- On V1_03 bright24, LightGlue recovery finally shows a positive effect: 27 requests, 4 accepted tracks, and RMSE improves to 0.121188 m.
- On MH_04 and MH_05, budget60 remains ineffective. It admits no recovery tracks and does not improve the trajectory.
- The Machine Hall behavior suggests that the current health trigger and candidate admission are not aligned with the dominant Machine Hall error mode. A sequence-specific trigger sweep is still needed before using LightGlue recovery there.

## Next Step

Keep the bright24 V1_03 case as a useful positive regression test for recovery. For Machine Hall, test a looser but cheaper policy:

```bash
RECOVERY_REQUEST_MIN_SIGNAL_COUNT=1
RECOVERY_REQUEST_REQUIRE_DEGRADATION_SIGNAL=false
RECOVERY_REQUEST_LOST_RATIO=0.15
RECOVERY_REQUEST_MIN_TRACKS=100
RECOVERY_MAX_CNT=1
MAX_INFERENCE_MS=60
```

If this still accepts zero useful tracks, Machine Hall should be treated as a case for IMU-predicted flow prefiltering or estimator-level robustness, not LightGlue recovery alone.
