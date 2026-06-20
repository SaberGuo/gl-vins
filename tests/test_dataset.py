from pathlib import Path

import pytest
import yaml

from vins_lightglue.dataset import adjacent_pairs, load_image_sequence


def test_load_euroc_like_sequence(tmp_path: Path) -> None:
    sequence_root = tmp_path / "mav0"
    image_dir = sequence_root / "cam0" / "data"
    image_dir.mkdir(parents=True)
    (image_dir / "000.png").write_bytes(b"fake")
    (image_dir / "001.png").write_bytes(b"fake")
    (sequence_root / "cam0" / "data.csv").write_text(
        "#timestamp [ns],filename\n100,000.png\n200,001.png\n",
        encoding="utf-8",
    )

    config = {
        "layout": {
            "camera_dir_template": "{sequence_root}/{camera}/data",
            "timestamp_csv_template": "{sequence_root}/{camera}/data.csv",
        }
    }
    config_path = tmp_path / "dataset.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    frames = load_image_sequence(config_path, sequence_root, "cam0")
    assert [frame.timestamp_ns for frame in frames] == [100, 200]
    assert adjacent_pairs(frames) == [(frames[0], frames[1])]


def test_load_sequence_rejects_too_few_images(tmp_path: Path) -> None:
    sequence_root = tmp_path / "mav0"
    image_dir = sequence_root / "cam0" / "data"
    image_dir.mkdir(parents=True)
    (image_dir / "000.png").write_bytes(b"fake")
    (sequence_root / "cam0" / "data.csv").write_text("100,000.png\n", encoding="utf-8")
    config = {
        "layout": {
            "camera_dir_template": "{sequence_root}/{camera}/data",
            "timestamp_csv_template": "{sequence_root}/{camera}/data.csv",
        }
    }
    config_path = tmp_path / "dataset.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError):
        load_image_sequence(config_path, sequence_root, "cam0")
