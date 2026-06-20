# Hybrid KLT + LightGlue Recovery on EuRoC MH_01

Date: 2026-06-20

## Goal

Test the idea:

> Keep high-frequency KLT tracking, and call LightGlue only when KLT track count drops, parallax is abnormal, or recovery is needed.

This is different from the previous pure LightGlue frontend replacement.

## Implementation Added

- Hybrid ROS node:
  - `scripts/ros1/hybrid_klt_lightglue_feature_tracker_node.py`
- Runner:
  - `scripts/wsl20/run_vins_hybrid_lightglue_euroc.sh`

The node:

- Uses OpenCV pyramidal LK optical flow for temporal tracking.
- Uses OpenCV LK for stereo left-right matching.
- Uses `goodFeaturesToTrack` for normal feature refill.
- Calls SuperPoint + LightGlue only when:
  - active KLT tracks drop below `KLT_MIN_TRACKS`, or
  - mean temporal flow is below `LOW_PARALLAX_PX` while the frontend is under target count.
- Publishes VINS-compatible `/feature_tracker/feature`.
- Tracks every image but publishes every second image by default, matching VINS-Fusion `inputImageCnt % 2 == 0` behavior.

## Commands

30 second smoke test:

```bash
BAG_DURATION=30 BAG_RATE=1.0 MAX_CNT=180 KLT_MIN_TRACKS=120 RECOVERY_INTERVAL=5 \
PUBLISH_EVERY=2 MAX_KEYPOINTS=512 DEVICE=cuda PUBLISH_STEREO=1 \
LIGHTGLUE_PYTHON=/home/gx/venvs/lightglue-cu128-py39/bin/python \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_hybrid_lightglue_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Full bag:

```bash
BAG_RATE=1.0 MAX_CNT=180 KLT_MIN_TRACKS=120 RECOVERY_INTERVAL=5 \
PUBLISH_EVERY=2 MAX_KEYPOINTS=512 DEVICE=cuda PUBLISH_STEREO=1 \
LIGHTGLUE_PYTHON=/home/gx/venvs/lightglue-cu128-py39/bin/python \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_hybrid_lightglue_euroc.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

## Results

### 30 Second Smoke Test

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_hybrid_lg_k512_m180_20260620_093446`
- VIO rows: `269`
- Sim(3) scale: `0.909109`
- RMSE: `0.051580 m`
- Mean: `0.043175 m`
- Median: `0.034197 m`
- P95: `0.091550 m`
- Max: `0.127200 m`

This confirms the hybrid frontend can initialize and produce stable short-segment output.

### Full Bag

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_hybrid_lg_k512_m180_20260620_093558`
- VIO rows: `1831`
- Sim(3) scale: `0.802206`
- RMSE: `0.763128 m`
- Mean: `0.659522 m`
- Median: `0.598430 m`
- P95: `1.437964 m`
- Max: `1.923562 m`

Baseline reference:

- Original VINS-Fusion frontend rows: `1831`
- Sim(3) scale: `1.046527`
- RMSE: `0.193380 m`
- P95: `0.316011 m`

## Interpretation

The hybrid idea is directionally better than pure LightGlue replacement, but this first implementation still does not beat the original VINS frontend on `MH_01_easy`.

The important finding is that an external Python KLT implementation is not equivalent to "the original VINS KLT frontend":

- It still uses the external feature-message path, not the exact internal `FeatureTracker` state path.
- It uses OpenCV/pinhole normalization rather than the original camodocal/MEI camera model.
- Its stereo tracks and temporal velocities are approximations of the VINS internal frontend semantics.
- LightGlue recovery rarely triggers on `MH_01_easy`, which is already easy for KLT.

Therefore this result should not be treated as a final verdict on KLT + LightGlue recovery. It shows that the next useful implementation must place recovery inside or immediately adjacent to the original C++ `FeatureTracker`.

## Next Engineering Step

Implement a C++-side recovery bridge:

1. Keep original `FeatureTracker::trackImage()` as the authoritative KLT frontend.
2. Add an optional recovery candidate topic, for example `/feature_tracker/recovery_matches`.
3. Let a Python LightGlue node publish candidate `(prev_uv, cur_uv, score)` only on triggered frames.
4. In C++ `FeatureTracker`, accept only candidates near lost previous tracks and normalize with camodocal.
5. Preserve original `ids`, `track_cnt`, stereo LK behavior, velocity computation, and publish cadence.

That is the version that will actually test "original VINS KLT + LightGlue recovery".
