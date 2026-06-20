# C++ FeatureTracker LightGlue Recovery Bridge

Date: 2026-06-20

## Purpose

This bridge keeps the original VINS-Fusion C++ `FeatureTracker` as the authoritative frontend and lets a C++ LightGlue runner provide only recovery candidates.

The intended architecture is:

```text
EuRoC images
  -> original VINS C++ FeatureTracker
       - KLT temporal tracking
       - GFTT refill
       - camodocal / MEI normalization
       - stereo LK
       - ids / track_cnt / velocity semantics
  -> optional C++ LightGlue candidate topic
       - SuperPoint + LightGlue inference in C++ via ONNX Runtime or TensorRT
       - publishes candidate matches only
  -> FeatureTracker accepts a small number of candidates for tracks KLT lost
  -> original VINS estimator
```

This is different from the earlier external Python frontend. The external prototype replaced too much of the VINS frontend and produced worse full-bag accuracy.

## Implemented VINS Changes

Patched files:

- `vins_estimator/src/featureTracker/feature_tracker.h`
- `vins_estimator/src/featureTracker/feature_tracker.cpp`
- `vins_estimator/src/estimator/estimator.h`
- `vins_estimator/src/estimator/estimator.cpp`
- `vins_estimator/src/rosNodeTest.cpp`

New runtime params on `vins_node`:

```bash
_enable_lightglue_recovery_bridge:=true
_recovery_max_cnt:=30
_recovery_time_tolerance:=0.01
_recovery_prev_match_radius:=4.0
_recovery_min_dist_ratio:=0.5
_recovery_request_min_tracks:=120
_recovery_request_lost_ratio:=0.10
_recovery_request_timeout_ms:=80
```

New subscribed topic when enabled:

```text
/feature_tracker/recovery_candidates
```

New published topic when request mode is enabled:

```text
/feature_tracker/recovery_request
```

Message type:

```text
sensor_msgs/PointCloud
```

Topic contract:

- `header.stamp`: timestamp of the current left image.
- `points[i].x`: current-frame `u`.
- `points[i].y`: current-frame `v`.
- `points[i].z`: unused.
- `channels[0].name`: recommended `prev_u`.
- `channels[0].values[i]`: previous-frame `u`.
- `channels[1].name`: recommended `prev_v`.
- `channels[1].values[i]`: previous-frame `v`.
- `channels[2].name`: recommended `score`.
- `channels[2].values[i]`: LightGlue confidence or matcher score; optional, defaults to `1.0`.

Acceptance rule inside `FeatureTracker`:

1. Candidate timestamp must be within `_recovery_time_tolerance` of the current image time.
2. Candidate previous point must be near a previous KLT track that failed this frame.
3. Candidate current point must be inside image bounds.
4. Candidate current point must not be too close to an already active current point.
5. Accepted candidates reuse the original lost track's `id` and increment its `track_cnt`.
6. Accepted points then continue through the original camodocal normalization, velocity computation, stereo LK, and VINS publish path.

This preserves the original VINS frontend semantics.

Request rule inside `FeatureTracker`:

1. Run the original KLT step first.
2. Compute active track count and lost-track ratio after KLT status filtering.
3. If active tracks fall below `_recovery_request_min_tracks` or lost ratio exceeds `_recovery_request_lost_ratio`, publish a recovery request containing raw-pixel locations of lost previous-frame tracks.
4. Wait up to `_recovery_request_timeout_ms` for candidates.
5. Apply accepted candidates once and clear them so stale recovery matches cannot be reused by later frames.

## Build Verification

WSL Ubuntu 20.04 / ROS Noetic build:

```bash
source /opt/ros/noetic/setup.bash
cd /home/gx/catkin_ws_vins
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
```

Result: `vins_node` rebuilt successfully.

No-candidate smoke test with the bridge enabled:

```bash
BAG_DURATION=30 \
VINS_EXTRA_ARGS="_enable_lightglue_recovery_bridge:=true" \
bash /mnt/f/research/research/engineering/vins-lightglue-vio/scripts/wsl20/run_vins_fusion_euroc_baseline.sh \
  /home/gx/catkin_ws_vins \
  /mnt/f/datasets/EuRoC_bags/MH_01_easy.bag
```

Result:

- Run dir: `/home/gx/catkin_ws_vins/results/MH_01_easy_20260620_094849`
- VIO rows: `281`
- Sim(3) scale: `0.946793`
- RMSE: `0.050014 m`
- P95: `0.090742 m`

This verifies that enabling the bridge without candidate input does not break the original VINS frontend path.

## C++ LightGlue Backend Choice

Recommended next backend: ONNX Runtime C++ first, TensorRT later.

Rationale:

- ONNX Runtime gives a C++ API and can run ONNX-exported LightGlue/SuperPoint models.
- TensorRT can be faster but adds engine conversion and CUDA/TensorRT version coupling.
- For this project, the first goal is correct candidate semantics inside VINS, not maximum inference speed.

Candidate backend options to evaluate:

- `OroChippw/LightGlue-OnnxRunner`: C++ ONNX inference for LightGlue, with SuperPoint/DISK + LightGlue support.
- `fabio-sim/LightGlue-ONNX`: ONNX export/inference tooling, useful for generating compatible models.
- TensorRT pipelines such as `SuperPoint-LightGlue-TensorRT` after the ONNX path is validated.

## Current ONNX Request Mode Status

Status update: a C++ ROS node now exists:

```text
lightglue_recovery_candidate_node
```

Implemented:

1. Subscribe to `/cam0/image_raw`.
2. Subscribe to `/feature_tracker/recovery_request`.
3. Publish raw-pixel candidate matches to `/feature_tracker/recovery_candidates`.
4. Provide `fake_grid` backend for bridge dry-run and stress testing.
5. Provide `onnx` backend through ONNX Runtime C++ with CUDA12 support.
6. In request mode, re-rank LightGlue matches by proximity to the KLT-lost previous-frame points before publishing candidates.

Useful node params:

```bash
_backend:=onnx
_use_cuda:=1
_request_only:=1
_request_prev_radius:=16.0
_request_distance_penalty:=0.03
_pipeline_onnx:=/mnt/f/research/research/engineering/vins-lightglue-vio/third_party/models/superpoint_lightglue_pipeline.onnx
```

The node should not publish VINS feature tracks. It should only suggest candidates.

Full-bag result on EuRoC `MH_01_easy`:

```text
run=/home/gx/catkin_ws_vins/results/MH_01_easy_cpp_recovery_onnx_20260620_150120
accepted_total=995
ate_rmse_m=0.183657
baseline_rmse_m=0.193380
```

This is a small improvement on an easy sequence, not a broad robustness claim.

Detailed results are recorded in:

```text
docs/results_cpp_recovery_bridge_euroc_mh01_wsl20.md
```

Remaining work:

1. Rerun request-driven ONNX recovery on harder EuRoC sequences.
2. Add geometric sanity checks before candidate acceptance.
3. Evaluate a cached `superpoint.onnx` + `superpoint_lightglue_fused.onnx` route to reduce request latency.
