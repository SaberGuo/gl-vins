# Degradation-Aware LightGlue Recovery Plan

Date: 2026-06-20

## Goal

当前主线不是用 LightGlue 替代 VINS 的 KLT frontend，而是在真实退化场景中让 LightGlue 成为低频、受约束的 lost-track recovery provider。

要覆盖的现实问题：

- 高速运动：相邻帧像素位移大，KLT 搜索窗口不足，parallax 和重投影残差容易异常。
- 运动模糊：纹理梯度下降，KLT 和 learned matcher 都会变差，但 LightGlue 可能在仍有结构的区域恢复少量 track。
- 光强突变：亮度一致性假设失效，KLT 容易丢 track，learned descriptor 和归一化预处理更有价值。

## Confirmed Current Problem

已有结果说明：

- 直接替换 VINS frontend 的外部 LightGlue 路线明显差于原始 VINS。
- 固定频率 ONNX candidates 在 full real-time `MH_01_easy` 上 accepted total 为 `0`，因为候选没有对齐 KLT 实际丢失的 tracks。
- `FeatureTracker` health-triggered request + lost-track proximity reranking 后，full real-time `MH_01_easy` accepted total `995`，RMSE `0.183657 m`，略好于原始 baseline `0.193380 m`。

因此接下来的优化点是：让 request trigger 更懂退化类型，并在接纳阶段更保守。

## New Health Signals

本分支在 C++ overlay 中新增低成本健康指标：

| Signal | Measurement | Why it matters |
|---|---|---|
| `lost_ratio` | KLT 后丢失 tracks / 原 tracks | 已有主触发指标 |
| `active_num` | KLT 后仍活跃 tracks | 已有主触发指标 |
| `mean_flow` | 成功 KLT tracks 的平均像素位移 | 高速运动、低帧率、快速转动的 proxy |
| `blur_score` | Laplacian variance | 运动模糊或失焦导致梯度下降 |
| `brightness_delta` | 当前帧与上一帧灰度均值差 | 曝光/光强突变 proxy |

新增 ROS params：

```bash
_recovery_request_max_mean_flow:=0.0
_recovery_request_min_blur_score:=0.0
_recovery_request_brightness_delta:=0.0
```

默认 `0.0` 表示关闭对应触发，保持原有行为。开启后：

- `mean_flow >= max_mean_flow` 触发 high-speed recovery request。
- `blur_score <= min_blur_score` 触发 blur-aware recovery request。
- `brightness_delta >= brightness_delta_threshold` 触发 illumination-aware recovery request。

触发日志会输出：

```text
recovery request health active=... lost=... ratio=... flow=... blur=... brightness_delta=... reasons=...
```

## Candidate Acceptance Policy

当前仍保持小步优化：

1. LightGlue 只提供 candidates。
2. 原始 `FeatureTracker` 仍负责接纳。
3. 只恢复 KLT 丢失的旧 track，不创建任意新 track。
4. 候选必须接近 lost previous-frame point。
5. 候选必须远离当前活跃点，避免密集错误注入。

下一轮需要加入几何 gate：

- Fundamental matrix / RANSAC gate for candidate batch.
- IMU-predicted displacement consistency gate.
- Local flow direction consistency gate.
- Lower `RECOVERY_MAX_CNT` under blur, because blur increases false positives.

## Experiment Matrix

### Real Sequences

先跑较难 EuRoC：

- `MH_04_difficult`
- `MH_05_difficult`
- `V1_03_difficult`
- `V2_03_difficult`

比较：

1. Original VINS baseline.
2. Current request-driven ONNX recovery.
3. Degradation-aware trigger recovery.
4. Degradation-aware trigger + geometry gate.

### Synthetic Stress

在 rosbag image stream 或离线 image stream 上注入：

| Stress | Sweep |
|---|---|
| Motion blur | kernel length 5, 9, 13, 17; horizontal/vertical/diagonal |
| Exposure jump | brightness scale 0.5, 0.7, 1.3, 1.6; gamma 0.6, 1.6 |
| Frame skipping | play rate / frame drop to mimic high-speed apparent flow |

Metrics：

- ATE RMSE / median / p95.
- accepted recovery total and accepted per trigger reason.
- recovery request count.
- frontend track count, lost ratio, mean flow, blur score, brightness delta.
- estimator failure or reset.
- runtime median/p95/max for ONNX inference.

## Suggested Initial Parameters

For EuRoC `MH_01_easy`, keep conservative settings:

```bash
RECOVERY_MAX_CNT=3
RECOVERY_REQUEST_MIN_TRACKS=120
RECOVERY_REQUEST_LOST_RATIO=0.10
RECOVERY_REQUEST_TIMEOUT_MS=80
RECOVERY_REQUEST_MAX_MEAN_FLOW=35
RECOVERY_REQUEST_MIN_BLUR_SCORE=20
RECOVERY_REQUEST_BRIGHTNESS_DELTA=30
REQUEST_PREV_RADIUS=14
REQUEST_DISTANCE_PENALTY=0.04
```

These are not final values. They are meant to produce logs and bounded recovery, then tune from full-bag metrics.

## Research Decision Rule

Do not claim LightGlue improves VINS only because accepted tracks increase.

Promote the method only if it improves or preserves:

- full-bag RMSE,
- p95 error,
- failure rate,
- recovery under synthetic/real degradation,
- and runtime tail latency.

If LightGlue improves track count but worsens trajectory, the acceptance gate is too loose.
