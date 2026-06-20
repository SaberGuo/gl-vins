# Experiment Protocol

## Baselines

- VINS-Fusion default frontend.
- VINS + LightGlue frontend.
- Optional: VINS + SuperPoint-only matching / LightGlue with lower keypoint budget.

## Dataset Matrix

| Dataset | Sequence | Purpose |
|---|---|---|
| EuRoC | MH_01_easy | Smoke test |
| EuRoC | MH_04_difficult | aggressive motion / visual difficulty |
| EuRoC | MH_05_difficult | difficult VIO stress |
| TUM-VI | room1 | fisheye indoor baseline |
| TUM-VI | corridor1 | corridor / low texture |

## Metrics

- ATE RMSE.
- RPE translational and rotational drift.
- Tracking failure count.
- Mean / median frontend runtime.
- Mean matched feature count.
- Median feature track length.
- Backend accepted feature count.

## Reporting Template

```text
Run:
Dataset:
Sequence:
Frontend:
Keypoint budget:
Resize:
GPU:
ATE RMSE:
RPE:
Failure count:
Mean frontend ms:
Median track length:
Notes:
```

## Interpretation Rules

- If LightGlue improves ATE but halves runtime below real-time, report it as offline robustness, not onboard readiness.
- If feature counts rise but track length drops, inspect ID propagation and outlier filtering.
- If EuRoC improves but TUM-VI degrades, check fisheye undistortion and normalized coordinate conversion first.
