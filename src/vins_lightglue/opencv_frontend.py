from __future__ import annotations

from pathlib import Path
from time import perf_counter

import cv2

from .lightglue_frontend import MatchResult, read_gray


class OrbFrontend:
    def __init__(
        self,
        max_num_keypoints: int = 1024,
        resize_long_edge: int | None = 752,
        ratio: float = 0.75,
    ) -> None:
        self.resize_long_edge = resize_long_edge
        self.ratio = ratio
        self.orb = cv2.ORB_create(nfeatures=max_num_keypoints)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def match_pair(
        self,
        image0_path: Path,
        image1_path: Path,
        timestamp0_ns: int,
        timestamp1_ns: int,
    ) -> MatchResult:
        image0 = read_gray(image0_path, self.resize_long_edge)
        image1 = read_gray(image1_path, self.resize_long_edge)

        start = perf_counter()
        kp0, desc0 = self.orb.detectAndCompute(image0, None)
        kp1, desc1 = self.orb.detectAndCompute(image1, None)
        if desc0 is None or desc1 is None:
            good_matches = []
        else:
            knn_matches = self.matcher.knnMatch(desc0, desc1, k=2)
            good_matches = []
            for pair in knn_matches:
                if len(pair) != 2:
                    continue
                m, n = pair
                if m.distance < self.ratio * n.distance:
                    good_matches.append(m)
        elapsed_ms = (perf_counter() - start) * 1000.0

        return MatchResult(
            image0=image0_path,
            image1=image1_path,
            timestamp0_ns=timestamp0_ns,
            timestamp1_ns=timestamp1_ns,
            keypoints0=len(kp0),
            keypoints1=len(kp1),
            matches=len(good_matches),
            elapsed_ms=elapsed_ms,
        )
