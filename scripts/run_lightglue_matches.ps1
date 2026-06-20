param(
    [Parameter(Mandatory=$true)]
    [string]$SequenceRoot,

    [string]$DatasetConfig = "configs/datasets/euroc.yaml",
    [string]$Camera = "cam0",
    [int]$MaxPairs = 50,
    [string]$Output = "runs/euroc_mh01_lightglue/matches.csv"
)

$ErrorActionPreference = "Stop"

python -m vins_lightglue.cli match `
    --dataset-config $DatasetConfig `
    --sequence-root $SequenceRoot `
    --camera $Camera `
    --max-pairs $MaxPairs `
    --output $Output
