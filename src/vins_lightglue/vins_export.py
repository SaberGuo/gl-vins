from __future__ import annotations

from dataclasses import asdict
from pathlib import Path

import pandas as pd

from .lightglue_frontend import MatchResult


def write_match_results(results: list[MatchResult], output_path: str | Path) -> None:
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for result in results:
        row = asdict(result)
        row["image0"] = str(result.image0)
        row["image1"] = str(result.image1)
        rows.append(row)
    pd.DataFrame(rows).to_csv(path, index=False)


def summarize_match_results(results: list[MatchResult]) -> dict[str, float]:
    if not results:
        return {
            "pairs": 0,
            "mean_matches": 0.0,
            "median_matches": 0.0,
            "mean_elapsed_ms": 0.0,
        }
    matches = pd.Series([r.matches for r in results], dtype="float64")
    elapsed = pd.Series([r.elapsed_ms for r in results], dtype="float64")
    return {
        "pairs": float(len(results)),
        "mean_matches": float(matches.mean()),
        "median_matches": float(matches.median()),
        "mean_elapsed_ms": float(elapsed.mean()),
    }
