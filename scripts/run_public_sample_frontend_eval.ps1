param(
    [string]$SequenceRoot = "data/public_samples/sample_euroc_MH05/mav0",
    [int]$MaxPairs = 20,
    [int]$ResizeLongEdge = 752,
    [int]$MaxNumKeypoints = 1024
)

$ErrorActionPreference = "Stop"
$env:PYTHONPATH = (Resolve-Path "src").Path

python -m vins_lightglue.cli match `
    --dataset-config configs/datasets/euroc.yaml `
    --sequence-root $SequenceRoot `
    --camera cam0 `
    --max-pairs $MaxPairs `
    --resize-long-edge $ResizeLongEdge `
    --max-num-keypoints $MaxNumKeypoints `
    --output runs/public_sample_euroc_mh05_lightglue/matches.csv

python -m vins_lightglue.cli orb-match `
    --dataset-config configs/datasets/euroc.yaml `
    --sequence-root $SequenceRoot `
    --camera cam0 `
    --max-pairs $MaxPairs `
    --resize-long-edge $ResizeLongEdge `
    --max-num-keypoints $MaxNumKeypoints `
    --output runs/public_sample_euroc_mh05_orb/matches.csv
