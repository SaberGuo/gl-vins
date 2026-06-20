# V1_03 Difficult LightGlue Recovery Geometry Gate

Date: 2026-06-20

Dataset:
- Bag: `/mnt/f/datasets/EuRoC_bags/vicon_room1/vicon_room1/V1_03_difficult/V1_03_difficult.bag`
- GT topic: `/vicon/firefly_sbx/firefly_sbx`
- Duration: about 109.9 s

## Summary

V1_03 confirms the earlier synthetic-degradation observation: accepting many LightGlue recovery tracks is harmful unless the candidates are tightly gated by local geometry.

The new local KLT-flow geometry gate is useful as a safety filter. It reduced the no-gate over-injection problem, but the current ONNX runtime path is still too expensive if requests are frequent. The best accuracy run below improved RMSE, but only produced 811 VIO samples, so it should not yet be treated as a real win.

## Runs

| mode | run dir | accepted tracks | VIO samples | aligned samples | ATE RMSE m | ATE p95 m | notes |
|---|---:|---:|---:|---:|---:|---:|---|
| baseline | `/home/gx/catkin_ws_vins/results/V1_03_difficult_20260620_175553` | 0 | 1064 | 1056 | 0.121651 | 0.223654 | Original VINS frontend |
| recovery no gate | `/home/gx/catkin_ws_vins/results/V1_03_difficult_cpp_recovery_onnx_20260620_175804` | 4464 | 1064 | 1056 | 0.126118 | 0.238023 | Too many candidates accepted; degrades trajectory |
| gate tight | `/home/gx/catkin_ws_vins/results/V1_03_difficult_cpp_recovery_onnx_20260620_180556` | 15 | 811 | 803 | 0.112533 | 0.188296 | Better RMSE, but lower frame coverage due frequent recovery requests / ONNX latency |
| gate sparse | `/home/gx/catkin_ws_vins/results/V1_03_difficult_cpp_recovery_onnx_20260620_180834` | 0 | 1064 | 1056 | 0.123794 | 0.230789 | Preserves coverage but rejects every candidate |
| gate relaxed | `/home/gx/catkin_ws_vins/results/V1_03_difficult_cpp_recovery_onnx_20260620_181059` | 33 | 1064 | 1056 | 0.122095 | 0.224878 | Preserves coverage and nearly matches baseline |

## Geometry Gate Parameters

The implemented gate is disabled by default:

```bash
RECOVERY_MAX_FLOW_ERROR=0
```

When enabled, each LightGlue candidate is accepted only if its image-space flow is consistent with nearby successfully tracked KLT flows:

```bash
RECOVERY_MAX_FLOW_ERROR=20
RECOVERY_MIN_FLOW_TRACKS=6
```

The currently most practical V1_03 setting is the relaxed gate:

```bash
BACKEND=onnx USE_CUDA=1 \
RECOVERY_MAX_CNT=2 \
RECOVERY_REQUEST_MIN_TRACKS=80 \
RECOVERY_REQUEST_LOST_RATIO=0.25 \
RECOVERY_REQUEST_TIMEOUT_MS=30 \
RECOVERY_REQUEST_MAX_MEAN_FLOW=40 \
RECOVERY_REQUEST_MIN_BLUR_SCORE=15 \
RECOVERY_REQUEST_BRIGHTNESS_DELTA=50 \
RECOVERY_MAX_FLOW_ERROR=20 \
RECOVERY_MIN_FLOW_TRACKS=6 \
bash scripts/wsl20/run_vins_cpp_recovery_bridge_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/vicon_room1/vicon_room1/V1_03_difficult/V1_03_difficult.bag
```

## Interpretation

- No-gate recovery is worse than baseline because it injects thousands of visually plausible but VIO-inconsistent tracks.
- A tight flow gate can improve RMSE, but current request frequency and ONNX inference latency reduce output coverage. This makes the result not yet comparable to full-rate VINS.
- The relaxed flow gate is the best current engineering compromise: it preserves full output coverage and avoids the clear no-gate degradation, but it is only parity-level on V1_03.

## Next Tuning Step

The next useful change is not to further loosen candidate acceptance. It is to reduce request volume before ONNX inference:

1. Add request cooldown, e.g. do not request recovery more often than 5-10 Hz.
2. Request only when multiple health signals agree, e.g. `(lost_ratio || track_count) && (high_speed || blur || brightness)`.
3. Add a hard inference budget: if the candidate node exceeds 30-40 ms median latency, skip publishing for that frame instead of blocking VINS.
4. After that, rerun V1_03 and the synthetic blur/exposure/frame-skip bags with the same evaluator.
