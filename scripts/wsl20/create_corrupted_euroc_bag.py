#!/usr/bin/env python3
"""Create short EuRoC rosbag variants with image-only degradations."""

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge


IMAGE_TOPICS = {"/cam0/image_raw", "/cam1/image_raw"}


def motion_blur_kernel(length: int, angle_deg: float) -> np.ndarray:
    length = max(1, int(length))
    kernel = np.zeros((length, length), dtype=np.float32)
    kernel[length // 2, :] = 1.0
    center = (length / 2.0 - 0.5, length / 2.0 - 0.5)
    rot = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
    kernel = cv2.warpAffine(kernel, rot, (length, length))
    s = float(kernel.sum())
    if s <= 0:
        kernel[length // 2, :] = 1.0
        s = float(kernel.sum())
    return kernel / s


def apply_gamma(img: np.ndarray, gamma: float) -> np.ndarray:
    gamma = max(0.05, float(gamma))
    lut = np.arange(256, dtype=np.float32)
    lut = np.power(lut / 255.0, gamma) * 255.0
    return cv2.LUT(img, np.clip(lut, 0, 255).astype(np.uint8))


def corrupt_image(img: np.ndarray, mode: str, elapsed: float, args) -> np.ndarray:
    if mode == "motion_blur":
        kernel = motion_blur_kernel(args.blur_length, args.blur_angle)
        return cv2.filter2D(img, -1, kernel)
    if mode == "exposure_jump":
        if elapsed < args.jump_after:
            return img
        out = img.astype(np.float32) * args.brightness_scale + args.brightness_offset
        out = np.clip(out, 0, 255).astype(np.uint8)
        if abs(args.gamma - 1.0) > 1e-6:
            out = apply_gamma(out, args.gamma)
        return out
    return img


def should_write_image(topic: str, image_index: dict, mode: str, args) -> bool:
    if mode != "frame_skip":
        return True
    image_index[topic] = image_index.get(topic, 0) + 1
    return (image_index[topic] - 1) % max(1, args.keep_every) == 0


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag")
    parser.add_argument("output_bag")
    parser.add_argument("--mode", choices=["motion_blur", "exposure_jump", "frame_skip"], required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--blur-length", type=int, default=13)
    parser.add_argument("--blur-angle", type=float, default=0.0)
    parser.add_argument("--jump-after", type=float, default=10.0)
    parser.add_argument("--brightness-scale", type=float, default=1.6)
    parser.add_argument("--brightness-offset", type=float, default=0.0)
    parser.add_argument("--gamma", type=float, default=1.0)
    parser.add_argument("--keep-every", type=int, default=2)
    args = parser.parse_args()

    output = Path(args.output_bag)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()

    bridge = CvBridge()
    image_index = {}
    written = 0
    image_written = 0
    image_dropped = 0

    with rosbag.Bag(args.input_bag, "r") as src:
        start = src.get_start_time()
        end = start + args.duration if args.duration > 0 else math.inf
        with rosbag.Bag(str(output), "w") as dst:
            for topic, msg, t in src.read_messages():
                ts = t.to_sec()
                if ts > end:
                    break
                if topic in IMAGE_TOPICS:
                    if not should_write_image(topic, image_index, args.mode, args):
                        image_dropped += 1
                        continue
                    original_header = msg.header
                    img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                    img = corrupt_image(img, args.mode, ts - start, args)
                    msg = bridge.cv2_to_imgmsg(img, encoding=msg.encoding)
                    msg.header = original_header
                    image_written += 1
                dst.write(topic, msg, t)
                written += 1

    print(f"wrote={written} image_written={image_written} image_dropped={image_dropped} output={output}")


if __name__ == "__main__":
    main()
