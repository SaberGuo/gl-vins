param(
    [int]$ImageCount = 30,
    [string]$OutputRoot = "data/public_samples/sample_euroc_MH05/mav0"
)

$ErrorActionPreference = "Stop"

$pageUrl = "https://github.com/Realle0815/ORB_SLAM3-ROS2/tree/main/TEST_DATASET/sample_euroc_MH05/mav0/cam0/data"
$rawBase = "https://raw.githubusercontent.com/Realle0815/ORB_SLAM3-ROS2/main/TEST_DATASET/sample_euroc_MH05/mav0/cam0"

$html = (Invoke-WebRequest -UseBasicParsing $pageUrl -Headers @{"User-Agent" = "Mozilla/5.0"}).Content
$names = [regex]::Matches($html, "[0-9]{19}\.png") |
    ForEach-Object { $_.Value } |
    Sort-Object -Unique |
    Select-Object -First $ImageCount

if ($names.Count -lt 2) {
    throw "No image names parsed from $pageUrl"
}

$camRoot = Join-Path $OutputRoot "cam0"
$imageDir = Join-Path $camRoot "data"
New-Item -ItemType Directory -Force -Path $imageDir | Out-Null

Invoke-WebRequest -UseBasicParsing "$rawBase/data.csv" -OutFile (Join-Path $camRoot "data.csv")

foreach ($name in $names) {
    $dest = Join-Path $imageDir $name
    if (-not (Test-Path $dest)) {
        Invoke-WebRequest -UseBasicParsing "$rawBase/data/$name" -OutFile $dest
    }
}

Write-Output "Downloaded $($names.Count) images to $imageDir"
