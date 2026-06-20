# EuRoC Exposure-Jump Budget60 Recovery Evaluation

Date: 2026-06-20

Generated bags:

```text
/mnt/f/datasets/EuRoC_bags/exposure_eval/V1_03_difficult_exposure_jump.bag
/mnt/f/datasets/EuRoC_bags/exposure_eval/MH_04_difficult_exposure_jump.bag
/mnt/f/datasets/EuRoC_bags/exposure_eval/MH_05_difficult_exposure_jump.bag
```

Corruption:

```bash
--mode exposure_jump
--duration 0
--jump-after 10
--brightness-scale 1.6
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
| V1_03 exposure | baseline | `/home/gx/catkin_ws_vins/results/V1_03_difficult_exposure_jump_20260620_230706` | `/vicon/firefly_sbx/firefly_sbx` | 1063 | 1056 | 0 | 0 | 0.110095 | 0.190533 | exposure scale improves this run relative to original V1_03 baseline |
| V1_03 exposure | budget60 recovery | `/home/gx/catkin_ws_vins/results/V1_03_difficult_exposure_jump_cpp_recovery_onnx_20260620_230916` | `/vicon/firefly_sbx/firefly_sbx` | 1064 | 1056 | 31 | 1 | 0.111636 | 0.192113 | recovery triggers but slightly worsens ATE |
| MH_04 exposure | baseline | `/home/gx/catkin_ws_vins/results/MH_04_difficult_exposure_jump_20260620_231130` | `/leica/position` | 1005 | 997 | 0 | 0 | 0.393548 | 0.683850 | exposure scale improves this run relative to original MH_04 baseline |
| MH_04 exposure | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_04_difficult_exposure_jump_cpp_recovery_onnx_20260620_231334` | `/leica/position` | 1005 | 997 | 0 | 0 | 0.393548 | 0.683851 | no recovery triggered; identical to baseline |
| MH_05 exposure | baseline | `/home/gx/catkin_ws_vins/results/MH_05_difficult_exposure_jump_20260620_231542` | `/leica/position` | 1125 | 1120 | 0 | 0 | 0.335679 | 0.573916 | exposure scale slightly worsens original MH_05 baseline |
| MH_05 exposure | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_05_difficult_exposure_jump_cpp_recovery_onnx_20260620_231757` | `/leica/position` | 1125 | 1120 | 0 | 0 | 0.338912 | 0.575644 | no accepted recovery; slightly worse than exposure baseline |

## Findings

- This `brightness_scale=1.6` exposure jump is not a uniformly destructive corruption. It improves V1_03 and MH_04 baseline ATE relative to the original bags, likely because the simple brightness amplification makes some image regions easier for KLT.
- The current budget60 recovery policy remains safe in the sense that it avoids large over-injection, but it does not improve exposure-jump performance.
- V1_03 exposure produces 31 recovery requests and 1 accepted track, but ATE worsens from 0.110095 m to 0.111636 m.
- MH_04 and MH_05 exposure runs do not admit recovery tracks under the conservative policy.

## Next Exposure Test

For a stronger adverse exposure benchmark, use one or both of:

```bash
--brightness-scale 0.45 --brightness-offset 0 --gamma 1.0
--brightness-scale 1.0 --brightness-offset -80 --gamma 1.6
```

The current brightening test is useful as an illumination-change sanity check, but it is not enough to prove robustness under harsh underexposure or saturation.
