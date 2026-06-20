param(
    [string]$SequenceRoot = "data/public_samples/sample_euroc_MH05/mav0",
    [int]$MaxPairs = 20,
    [int]$ResizeLongEdge = 752
)

$ErrorActionPreference = "Stop"
$env:PYTHONPATH = (Resolve-Path "src").Path

foreach ($keypoints in @(1024, 512, 256)) {
    python -m vins_lightglue.cli match `
        --dataset-config configs/datasets/euroc.yaml `
        --sequence-root $SequenceRoot `
        --camera cam0 `
        --max-pairs $MaxPairs `
        --resize-long-edge $ResizeLongEdge `
        --max-num-keypoints $keypoints `
        --device cuda `
        --output "runs/public_sample_euroc_mh05_lightglue_gpu_k$keypoints/matches.csv"
}
