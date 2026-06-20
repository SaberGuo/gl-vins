# Public Dataset Notes

## EuRoC MAV

Use EuRoC first because VINS-Fusion commonly ships example configs for it.

Expected extracted structure:

```text
MH_01_easy/
└── mav0/
    ├── cam0/
    │   ├── data.csv
    │   └── data/
    ├── cam1/
    ├── imu0/
    └── state_groundtruth_estimate0/
```

Recommended first pass:

- `MH_01_easy`: smoke test.
- `MH_04_difficult`, `MH_05_difficult`: robustness stress.
- `V1_03_difficult`: motion/visual difficulty.

## TUM-VI

Use after EuRoC because fisheye camera handling and calibration alignment need more care.

Expected extracted structure:

```text
dataset-room1_512_16/
└── mav0/
    ├── cam0/
    ├── cam1/
    ├── imu0/
    └── mocap0/
```

Important:

- TUM-VI images are fisheye. For VINS-Fusion experiments, document whether images are undistorted before LightGlue or whether VINS receives original fisheye-normalized coordinates.
- Keep camera calibration versioned with each run.

## Suggested Robustness Subsets

- Low texture: corridors / white walls.
- Motion blur: fast turns and aggressive yaw.
- Lighting change: room/corridor transitions.
- Repeated texture: shelves, floor tiles, windows.

Each subset should be recorded as timestamp ranges in future `configs/subsets/*.yaml`.
