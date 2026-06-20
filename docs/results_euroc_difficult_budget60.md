# EuRoC Difficult Budget60 Recovery Evaluation

Date: 2026-06-20

Policy under test:

```bash
BACKEND=onnx USE_CUDA=1
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

This is the current conservative policy: keep original VINS KLT as the primary frontend, request LightGlue only when multiple health signals agree, reject geometry-inconsistent candidates, and drop over-budget ONNX results.

## Results

| sequence | mode | run dir | GT topic | VIO samples | aligned samples | requests | accepted tracks | ATE RMSE m | ATE p95 m | interpretation |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|---|
| V1_03_difficult | baseline | `/home/gx/catkin_ws_vins/results/V1_03_difficult_20260620_175553` | `/vicon/firefly_sbx/firefly_sbx` | 1064 | 1056 | 0 | 0 | 0.121651 | 0.223654 | baseline |
| V1_03_difficult | budget60 recovery | `/home/gx/catkin_ws_vins/results/V1_03_difficult_cpp_recovery_onnx_20260620_182702` | `/vicon/firefly_sbx/firefly_sbx` | 1064 | 1056 | 23 | 1 | 0.121652 | 0.223755 | parity with baseline; request volume controlled |
| MH_04_difficult | baseline | `/home/gx/catkin_ws_vins/results/MH_04_difficult_20260620_224521` | `/leica/position` | 1006 | 998 | 0 | 0 | 0.430734 | 0.730659 | baseline |
| MH_04_difficult | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_04_difficult_cpp_recovery_onnx_20260620_224728` | `/leica/position` | 1005 | 998 | 0 | 0 | 0.427874 | 0.739500 | no recovery triggered; essentially baseline-level |
| MH_05_difficult | baseline | `/home/gx/catkin_ws_vins/results/MH_05_difficult_20260620_224932` | `/leica/position` | 1126 | 1120 | 0 | 0 | 0.330279 | 0.560998 | baseline |
| MH_05_difficult | budget60 recovery | `/home/gx/catkin_ws_vins/results/MH_05_difficult_cpp_recovery_onnx_20260620_225149` | `/leica/position` | 1125 | 1120 | 1 | 0 | 0.330253 | 0.564649 | one request, no accepted recovery; baseline-level |

## Findings

- On MH_04 and MH_05, the conservative health trigger is mostly inactive. This means the policy is safe but not yet useful on these Machine Hall difficult bags.
- The recovery path does not degrade MH_04/MH_05, but it also does not materially improve them because almost no LightGlue candidates are admitted.
- V1_03 remains the only current sequence where the bridge actively requests nontrivial recovery under budget60, and even there the result is parity rather than improvement.
- The depletion prior should stay disabled by default. V1_03 ablations showed worse ATE for both hard filtering and rank-only boosting.

## Next Experiment

The next useful test is a less conservative Machine Hall trigger sweep, still keeping geometry gate and inference budget:

```bash
RECOVERY_REQUEST_MIN_SIGNAL_COUNT=1
RECOVERY_REQUEST_REQUIRE_DEGRADATION_SIGNAL=false
RECOVERY_REQUEST_LOST_RATIO=0.15
RECOVERY_REQUEST_MIN_TRACKS=100
RECOVERY_MAX_CNT=1
MAX_INFERENCE_MS=60
```

This should answer whether MH_04/MH_05 have recoverable short track-loss pockets, or whether LightGlue recovery is simply not aligned with their dominant error mode.
