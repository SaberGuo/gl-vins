# VINS-Fusion Integration Plan

## Integration Choice

The current mainline is a C++ internal recovery bridge, not a full LightGlue feature-tracker replacement.

Baseline VINS flow:

```text
camera image -> VINS feature_tracker -> /feature_tracker/feature -> VINS estimator
```

Current hybrid flow:

```text
camera image -> original VINS C++ FeatureTracker -> VINS estimator
             -> C++ LightGlue candidate node -> /feature_tracker/recovery_candidates
             -> FeatureTracker accepts only a few lost-track recovery candidates
```

Keep `_external_feature_only` at the default `false` for this route. The original C++ frontend remains authoritative for KLT tracking, GFTT refill, camodocal/MEI normalization, stereo LK, track ids, track counts, and velocity semantics.

The earlier route that published `/feature_tracker/feature` from Python LightGlue is retained only as a prototype and negative baseline. On full EuRoC `MH_01_easy`, it underperformed the original VINS frontend.

## Recovery Candidate Message Shape

The C++ LightGlue candidate node should publish `sensor_msgs/PointCloud` to:

```text
/feature_tracker/recovery_candidates
```

Expected fields:

- `header.stamp`: current left image timestamp.
- `points[i].x`: current-frame raw pixel `u`.
- `points[i].y`: current-frame raw pixel `v`.
- `channels[0].values[i]`: previous-frame raw pixel `u`.
- `channels[1].values[i]`: previous-frame raw pixel `v`.
- `channels[2].values[i]`: optional confidence or matcher score.

Do not publish normalized coordinates here. The VINS C++ side applies camodocal normalization after accepting a candidate.

## Required ROS Topics

Inputs:

- VINS estimator: `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`.
- C++ LightGlue candidate node: `/cam0/image_raw` first; stereo can be added later if needed.

Output:

- C++ LightGlue candidate node: `/feature_tracker/recovery_candidates`

VINS estimator still publishes/consumes its original feature flow internally. Do not replace `/feature_tracker/feature` for the bridge experiment.

## VINS Runtime Params

Enable the bridge:

```bash
rosrun vins vins_node <config.yaml> \
  _enable_lightglue_recovery_bridge:=true \
  _recovery_max_cnt:=30 \
  _recovery_time_tolerance:=0.01 \
  _recovery_prev_match_radius:=4.0 \
  _recovery_min_dist_ratio:=0.5
```

Keep `_external_feature_only:=false` unless deliberately running the old replacement frontend.

## Key Engineering Risks

- Timestamp mismatch: candidate `header.stamp` must match the image frame processed by `FeatureTracker`.
- Candidate over-acceptance: keep `recovery_max_cnt` small and reject points close to active KLT tracks.
- Coordinate convention mismatch: recovery candidates are raw pixels; VINS normalizes accepted candidates internally.
- Runtime: LightGlue should run at reduced rate or under health triggers, not every frame unless profiling proves it is cheap.
- Fisheye support: TUM-VI requires explicit camera model handling.

## First Milestone

1. Run original VINS-Fusion on EuRoC `MH_01_easy` as baseline.
2. Enable `_enable_lightglue_recovery_bridge:=true` with no candidate publisher and verify trajectory parity.
3. Run a fake-candidate publisher to verify that VINS logs `LightGlue recovery bridge accepted X tracks`.
4. Use `lightglue_recovery_candidate_node` with `_backend:=fake_grid` to verify the candidate topic and acceptance log.
5. Install or vendor ONNX Runtime C++ plus exported SuperPoint/LightGlue ONNX models.
6. Switch `lightglue_recovery_candidate_node` to a real ONNX backend and compare full EuRoC trajectory and frontend recovery statistics against the original baseline.
