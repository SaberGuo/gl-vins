from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def summarize(path: Path) -> dict[str, float | str]:
    df = pd.read_csv(path)
    stable = df.iloc[1:] if "lightglue_gpu" in str(path) and len(df) > 1 else df
    mean_stable_ms = float(stable["elapsed_ms"].mean())
    return {
        "run": str(path.parent),
        "pairs": float(len(df)),
        "mean_matches": float(df["matches"].mean()),
        "median_matches": float(df["matches"].median()),
        "mean_ms_all": float(df["elapsed_ms"].mean()),
        "stable_mean_ms": mean_stable_ms,
        "stable_hz": 1000.0 / mean_stable_ms if mean_stable_ms > 0 else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    args = parser.parse_args()

    rows = [summarize(path) for path in args.csv]
    print("| Run | Pairs | Mean matches | Median matches | Mean ms all | Stable mean ms | Stable Hz |")
    print("|---|---:|---:|---:|---:|---:|---:|")
    for row in rows:
        print(
            f"| {row['run']} | {row['pairs']:.0f} | {row['mean_matches']:.2f} | "
            f"{row['median_matches']:.2f} | {row['mean_ms_all']:.2f} | "
            f"{row['stable_mean_ms']:.2f} | {row['stable_hz']:.2f} |"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
