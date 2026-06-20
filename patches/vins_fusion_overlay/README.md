# VINS-Fusion Overlay

These files are the modified VINS-Fusion `vins_estimator` files used by the LightGlue recovery bridge experiments.

Apply by copying this `vins_estimator` overlay on top of a VINS-Fusion checkout, then build with the scripts in `scripts/wsl20/`.

The overlay adds:

- `FeatureTracker` health-triggered `/feature_tracker/recovery_request`.
- `/feature_tracker/recovery_candidates` acceptance for lost KLT tracks.
- C++ `lightglue_recovery_candidate_node` with fake-grid and ONNX Runtime backends.
- Optional ONNX Runtime C++/CUDA build wiring.
