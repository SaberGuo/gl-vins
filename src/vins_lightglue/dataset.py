from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ImageFrame:
    timestamp_ns: int
    image_path: Path


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _format_template(template: str, sequence_root: Path, camera: str) -> Path:
    return Path(template.format(sequence_root=str(sequence_root), camera=camera))


def load_image_sequence(
    dataset_config_path: str | Path,
    sequence_root: str | Path,
    camera: str,
) -> list[ImageFrame]:
    config = load_yaml(dataset_config_path)
    layout = config["layout"]
    sequence_root_path = Path(sequence_root)
    timestamp_csv = _format_template(layout["timestamp_csv_template"], sequence_root_path, camera)
    image_dir = _format_template(layout["camera_dir_template"], sequence_root_path, camera)

    if not timestamp_csv.exists():
        raise FileNotFoundError(f"timestamp csv not found: {timestamp_csv}")
    if not image_dir.exists():
        raise FileNotFoundError(f"image directory not found: {image_dir}")

    frames: list[ImageFrame] = []
    with timestamp_csv.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = [p.strip() for p in stripped.split(",")]
            if len(parts) < 2:
                continue
            try:
                timestamp_ns = int(parts[0])
            except ValueError:
                continue
            image_path = image_dir / parts[1]
            if image_path.exists():
                frames.append(ImageFrame(timestamp_ns=timestamp_ns, image_path=image_path))

    frames.sort(key=lambda frame: frame.timestamp_ns)
    if len(frames) < 2:
        raise ValueError(f"need at least two images, got {len(frames)} from {timestamp_csv}")
    return frames


def adjacent_pairs(frames: list[ImageFrame], max_pairs: int | None = None) -> list[tuple[ImageFrame, ImageFrame]]:
    pairs = list(zip(frames[:-1], frames[1:]))
    if max_pairs is not None:
        return pairs[:max_pairs]
    return pairs
