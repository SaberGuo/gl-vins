from __future__ import annotations

import argparse
import json
from pathlib import Path

from tqdm import tqdm

from .dataset import adjacent_pairs, load_image_sequence
from .lightglue_frontend import LightGlueFrontend
from .opencv_frontend import OrbFrontend
from .vins_export import summarize_match_results, write_match_results


def _run_pairwise(args: argparse.Namespace, frontend: object) -> int:
    frames = load_image_sequence(args.dataset_config, args.sequence_root, args.camera)
    pairs = adjacent_pairs(frames, args.max_pairs)

    results = []
    for frame0, frame1 in tqdm(pairs, desc=f"{args.frontend_name} pairs"):
        results.append(
            frontend.match_pair(
                frame0.image_path,
                frame1.image_path,
                frame0.timestamp_ns,
                frame1.timestamp_ns,
            )
        )

    write_match_results(results, args.output)
    summary = summarize_match_results(results)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


def run_lightglue(args: argparse.Namespace) -> int:
    args.frontend_name = "LightGlue"
    frontend = LightGlueFrontend(
        extractor=args.extractor,
        max_num_keypoints=args.max_num_keypoints,
        match_confidence=args.match_confidence,
        resize_long_edge=args.resize_long_edge,
        device=args.device,
    )
    return _run_pairwise(args, frontend)


def run_orb(args: argparse.Namespace) -> int:
    args.frontend_name = "ORB"
    frontend = OrbFrontend(
        max_num_keypoints=args.max_num_keypoints,
        resize_long_edge=args.resize_long_edge,
        ratio=args.ratio,
    )
    return _run_pairwise(args, frontend)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="VINS + LightGlue experiment utilities")
    subparsers = parser.add_subparsers(dest="command", required=True)

    match = subparsers.add_parser("match", help="run LightGlue on adjacent dataset frames")
    match.add_argument("--dataset-config", required=True, type=Path)
    match.add_argument("--sequence-root", required=True, type=Path)
    match.add_argument("--camera", default="cam0")
    match.add_argument("--output", required=True, type=Path)
    match.add_argument("--max-pairs", type=int, default=None)
    match.add_argument("--device", default="auto")
    match.add_argument("--extractor", default="superpoint")
    match.add_argument("--max-num-keypoints", type=int, default=2048)
    match.add_argument("--match-confidence", type=float, default=0.1)
    match.add_argument("--resize-long-edge", type=int, default=1024)
    match.set_defaults(func=run_lightglue)

    orb = subparsers.add_parser("orb-match", help="run ORB baseline on adjacent dataset frames")
    orb.add_argument("--dataset-config", required=True, type=Path)
    orb.add_argument("--sequence-root", required=True, type=Path)
    orb.add_argument("--camera", default="cam0")
    orb.add_argument("--output", required=True, type=Path)
    orb.add_argument("--max-pairs", type=int, default=None)
    orb.add_argument("--max-num-keypoints", type=int, default=1024)
    orb.add_argument("--resize-long-edge", type=int, default=752)
    orb.add_argument("--ratio", type=float, default=0.75)
    orb.set_defaults(func=run_orb)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
