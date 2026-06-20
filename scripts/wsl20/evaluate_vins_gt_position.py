#!/usr/bin/env python3
"""Evaluate VINS position against EuRoC rosbag position ground truth.

Supports Machine Hall `/leica/position` and Vicon Room
`/vicon/firefly_sbx/firefly_sbx` topics with Sim(3) alignment.
"""

import argparse
import csv
import math

import numpy as np
import rosbag


def load_groundtruth_positions(bag_path):
    leica_topic = "/leica/position"
    vicon_topic = "/vicon/firefly_sbx/firefly_sbx"
    samples = []
    topic_used = None
    with rosbag.Bag(bag_path) as bag:
        topics = bag.get_type_and_topic_info().topics
        if leica_topic in topics:
            topic_used = leica_topic
            for _, msg, stamp in bag.read_messages(topics=[leica_topic]):
                ts = msg.header.stamp.to_sec() if msg.header.stamp else stamp.to_sec()
                samples.append((ts, msg.point.x, msg.point.y, msg.point.z))
        elif vicon_topic in topics:
            topic_used = vicon_topic
            for _, msg, stamp in bag.read_messages(topics=[vicon_topic]):
                ts = msg.header.stamp.to_sec() if msg.header.stamp else stamp.to_sec()
                tr = msg.transform.translation
                samples.append((ts, tr.x, tr.y, tr.z))
        else:
            raise RuntimeError("No supported EuRoC ground-truth position topic found")
    return topic_used, np.asarray(samples, dtype=float)


def load_vins_positions(vio_csv):
    samples = []
    with open(vio_csv, newline="") as handle:
        for row in csv.reader(handle):
            if len(row) >= 4:
                samples.append((float(row[0]) * 1e-9, float(row[1]), float(row[2]), float(row[3])))
    return np.asarray(samples, dtype=float)


def umeyama_align(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = (target_centered.T @ source_centered) / len(source)
    u_mat, singular_values, vt_mat = np.linalg.svd(covariance)
    sign = np.eye(3)
    if np.linalg.det(u_mat @ vt_mat) < 0:
        sign[-1, -1] = -1
    rotation = u_mat @ sign @ vt_mat
    source_variance = (source_centered * source_centered).sum() / len(source)
    scale = np.trace(np.diag(singular_values) @ sign) / source_variance
    translation = target_mean - scale * rotation @ source_mean
    aligned = (scale * (rotation @ source.T)).T + translation
    return aligned, scale


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="EuRoC rosbag path")
    parser.add_argument("vio_csv", help="VINS output vio.csv")
    args = parser.parse_args()

    gt_topic, gt = load_groundtruth_positions(args.bag)
    vins = load_vins_positions(args.vio_csv)
    if gt.size == 0:
        raise RuntimeError("Ground-truth topic has no samples")
    if vins.size == 0:
        raise RuntimeError("VINS CSV has no samples")

    times = vins[:, 0]
    valid = (times >= gt[0, 0]) & (times <= gt[-1, 0])
    times = times[valid]
    predicted = vins[valid, 1:4]
    target = np.stack([np.interp(times, gt[:, 0], gt[:, axis]) for axis in range(1, 4)], axis=1)
    if len(predicted) < 3:
        raise RuntimeError("Not enough overlapping samples for alignment")

    aligned, scale = umeyama_align(predicted, target)
    error = np.linalg.norm(aligned - target, axis=1)

    print(f"gt_topic={gt_topic}")
    print(f"gt_samples={len(gt)}")
    print(f"vio_samples={len(vins)}")
    print(f"aligned_samples={len(error)}")
    print(f"scale={scale:.6f}")
    print(f"ate_rmse_m={math.sqrt(np.mean(error ** 2)):.6f}")
    print(f"ate_mean_m={np.mean(error):.6f}")
    print(f"ate_median_m={np.median(error):.6f}")
    print(f"ate_p95_m={np.percentile(error, 95):.6f}")
    print(f"ate_max_m={np.max(error):.6f}")


if __name__ == "__main__":
    main()
