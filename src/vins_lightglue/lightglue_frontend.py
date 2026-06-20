from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np
import torch


@dataclass(frozen=True)
class MatchResult:
    image0: Path
    image1: Path
    timestamp0_ns: int
    timestamp1_ns: int
    keypoints0: int
    keypoints1: int
    matches: int
    elapsed_ms: float


def resolve_device(device: str) -> str:
    if device == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    return device


def read_gray(path: Path, resize_long_edge: int | None = None) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"failed to read image: {path}")
    if resize_long_edge is not None and resize_long_edge > 0:
        h, w = image.shape[:2]
        scale = resize_long_edge / max(h, w)
        if scale < 1.0:
            image = cv2.resize(image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return image


def image_to_tensor(image: np.ndarray, device: str) -> torch.Tensor:
    tensor = torch.from_numpy(image).float() / 255.0
    return tensor[None, None].to(device)


class LightGlueFrontend:
    def __init__(
        self,
        extractor: str = "superpoint",
        max_num_keypoints: int = 2048,
        match_confidence: float = 0.1,
        resize_long_edge: int | None = 1024,
        device: str = "auto",
    ) -> None:
        self.device = resolve_device(device)
        self.resize_long_edge = resize_long_edge

        try:
            from lightglue import LightGlue, SuperPoint  # type: ignore
        except ImportError as exc:
            raise ImportError(
                "LightGlue is not installed. Install it with: "
                "pip install git+https://github.com/cvg/LightGlue.git"
            ) from exc

        if extractor != "superpoint":
            raise ValueError("only extractor='superpoint' is wired in this scaffold")

        self.extractor = SuperPoint(max_num_keypoints=max_num_keypoints).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint", filter_threshold=match_confidence).eval().to(self.device)

    @torch.inference_mode()
    def match_pair(
        self,
        image0_path: Path,
        image1_path: Path,
        timestamp0_ns: int,
        timestamp1_ns: int,
    ) -> MatchResult:
        image0 = read_gray(image0_path, self.resize_long_edge)
        image1 = read_gray(image1_path, self.resize_long_edge)
        tensor0 = image_to_tensor(image0, self.device)
        tensor1 = image_to_tensor(image1, self.device)

        start = perf_counter()
        feats0 = self.extractor.extract(tensor0)
        feats1 = self.extractor.extract(tensor1)
        matches01 = self.matcher({"image0": feats0, "image1": feats1})
        elapsed_ms = (perf_counter() - start) * 1000.0

        keypoints0 = int(feats0["keypoints"].shape[-2])
        keypoints1 = int(feats1["keypoints"].shape[-2])
        matches = matches01.get("matches", None)
        if matches is None:
            matches_count = int(matches01["matches0"].ge(0).sum().item())
        elif isinstance(matches, list):
            matches_count = int(matches[0].shape[0]) if matches else 0
        else:
            matches_count = int(matches.shape[-2])

        return MatchResult(
            image0=image0_path,
            image1=image1_path,
            timestamp0_ns=timestamp0_ns,
            timestamp1_ns=timestamp1_ns,
            keypoints0=keypoints0,
            keypoints1=keypoints1,
            matches=matches_count,
            elapsed_ms=elapsed_ms,
        )
