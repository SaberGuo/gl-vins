#!/usr/bin/env python3
"""Evaluate VINS position against EuRoC bag /leica/position with Sim(3) alignment.

This is a smoke-test metric for EuRoC rosbag runs. It is not a replacement for
full EuRoC ground-truth ATE/RPE because the rosbag topic provides Leica position
only, not full body pose.
"""

import argparse
import csv
import math

import numpy as np
import rosbag


def load_leica_positions(bag_path):
    samples = []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, stamp in bag.read_messages(topics=["/leica/position"]):
            ts = msg.header.stamp.to_sec() if msg.header.stamp else stamp.to_sec()
            samples.append((ts, msg.point.x, msg.point.y, msg.point.z))
    return np.asarray(samples, dtype=float)


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

    leica = load_leica_positions(args.bag)
    vins = load_vins_positions(args.vio_csv)
    times = vins[:, 0]
    valid = (times >= leica[0, 0]) & (times <= leica[-1, 0])
    times = times[valid]
    predicted = vins[valid, 1:4]
    target = np.stack([np.interp(times, leica[:, 0], leica[:, axis]) for axis in range(1, 4)], axis=1)

    aligned, scale = umeyama_align(predicted, target)
    error = np.linalg.norm(aligned - target, axis=1)

    print(f"leica_samples={len(leica)}")
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
