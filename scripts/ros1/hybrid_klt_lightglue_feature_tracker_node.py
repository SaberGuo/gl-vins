#!/usr/bin/env python3
"""Hybrid KLT + LightGlue recovery frontend for VINS-Fusion feature messages."""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
import torch
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, Image, PointCloud

from lightglue_feature_tracker_node import CameraModel, image_msg_to_mono8


@dataclass
class HybridState:
    timestamp: rospy.Time
    image_left: np.ndarray
    points_left: np.ndarray
    track_ids: np.ndarray
    track_counts: np.ndarray
    features_left: Optional[dict] = None
    keypoints_left: Optional[np.ndarray] = None


class HybridKltLightGlueTrackerNode:
    def __init__(self) -> None:
        self.image0_topic = rospy.get_param("~image0_topic", "/cam0/image_raw")
        self.image1_topic = rospy.get_param("~image1_topic", "/cam1/image_raw")
        self.feature_topic = rospy.get_param("~feature_topic", "/feature_tracker/feature")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.max_cnt = int(rospy.get_param("~max_cnt", 180))
        self.min_dist = int(rospy.get_param("~min_dist", 30))
        self.klt_min_tracks = int(rospy.get_param("~klt_min_tracks", 90))
        self.low_parallax_px = float(rospy.get_param("~low_parallax_px", 1.0))
        self.recovery_interval = int(rospy.get_param("~recovery_interval", 10))
        self.max_recoveries_per_frame = int(rospy.get_param("~max_recoveries_per_frame", 80))
        self.recovery_match_radius = float(rospy.get_param("~recovery_match_radius", 4.0))
        self.publish_every = max(1, int(rospy.get_param("~publish_every", 2)))
        self.match_confidence = float(rospy.get_param("~match_confidence", 0.1))
        self.max_num_keypoints = int(rospy.get_param("~max_num_keypoints", 512))
        self.publish_stereo = bool(rospy.get_param("~publish_stereo", True))
        self.device = rospy.get_param("~device", "auto")
        if self.device == "auto":
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.device = self.select_device(self.device)

        cam0_yaml = rospy.get_param("~cam0_calib")
        cam1_yaml = rospy.get_param("~cam1_calib", cam0_yaml)
        self.cam0 = CameraModel.from_yaml(cam0_yaml)
        self.cam1 = CameraModel.from_yaml(cam1_yaml)

        from lightglue import LightGlue, SuperPoint  # type: ignore

        self.extractor = SuperPoint(max_num_keypoints=self.max_num_keypoints).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint", filter_threshold=self.match_confidence).eval().to(self.device)

        self.previous: Optional[HybridState] = None
        self.next_track_id = itertools.count()
        self.frame_index = 0
        self.last_recovery_frame = -10**9
        self.publisher = rospy.Publisher(self.feature_topic, PointCloud, queue_size=20)

        self.image0_sub = message_filters.Subscriber(self.image0_topic, Image)
        if self.publish_stereo:
            self.image1_sub = message_filters.Subscriber(self.image1_topic, Image)
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.image0_sub, self.image1_sub], queue_size=30, slop=0.003
            )
            sync.registerCallback(self.on_stereo)
            self.sync = sync
        else:
            self.image0_sub.registerCallback(self.on_mono)

        rospy.logwarn(
            "Hybrid KLT+LightGlue tracker ready: device=%s max_cnt=%d klt_min=%d recovery_interval=%d",
            self.device,
            self.max_cnt,
            self.klt_min_tracks,
            self.recovery_interval,
        )

    @staticmethod
    def select_device(requested_device: str) -> str:
        if requested_device != "cuda":
            return requested_device
        try:
            torch.zeros(1, device="cuda") + 1
            torch.cuda.synchronize()
            return "cuda"
        except Exception as exc:
            rospy.logwarn("CUDA requested but unusable, falling back to CPU: %s", exc)
            return "cpu"

    def on_mono(self, msg0: Image) -> None:
        self.process(msg0, None)

    def on_stereo(self, msg0: Image, msg1: Image) -> None:
        self.process(msg0, msg1)

    def process(self, msg0: Image, msg1: Optional[Image]) -> None:
        start_time = time.perf_counter()
        image0 = image_msg_to_mono8(msg0)
        image1 = image_msg_to_mono8(msg1) if msg1 is not None else None

        if self.previous is None:
            points, ids, counts = self.detect_new_tracks(image0, self.max_cnt)
            state = HybridState(msg0.header.stamp, image0, points, ids, counts)
            published = self.should_publish_frame()
            if published:
                stereo_points, stereo_ids = self.track_stereo(image0, image1, points, ids)
                self.publish_features(msg0.header.stamp, state, stereo_points, stereo_ids)
            self.previous = state
            self.frame_index += 1
            return

        prev = self.previous
        klt_points, klt_ids, klt_counts, mean_flow = self.track_temporal(prev, image0)
        recovered = 0
        should_recover = (
            len(klt_points) < self.klt_min_tracks
            or (0 < mean_flow < self.low_parallax_px and len(klt_points) < self.max_cnt)
        )
        if should_recover and self.frame_index - self.last_recovery_frame >= self.recovery_interval:
            klt_points, klt_ids, klt_counts, recovered = self.recover_tracks_with_lightglue(
                prev, image0, klt_points, klt_ids, klt_counts
            )
            self.last_recovery_frame = self.frame_index

        points, ids, counts, new_count = self.add_gftt_tracks(image0, klt_points, klt_ids, klt_counts)
        state = HybridState(msg0.header.stamp, image0, points, ids, counts)
        published = self.should_publish_frame()
        stereo_points, stereo_ids = self.track_stereo(image0, image1, points, ids) if published else (empty_points(), empty_ints())
        if published:
            self.publish_features(msg0.header.stamp, state, stereo_points, stereo_ids)
        self.previous = state
        self.frame_index += 1

        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        rospy.loginfo_throttle(
            2.0,
            "Hybrid frame stamp=%.3f published=%d tracks=%d new=%d recovered=%d stereo=%d mean_flow=%.2f elapsed_ms=%.1f",
            msg0.header.stamp.to_sec(),
            int(published),
            len(points),
            new_count,
            recovered,
            len(stereo_ids),
            mean_flow,
            elapsed_ms,
        )

    def should_publish_frame(self) -> bool:
        return (self.frame_index + 1) % self.publish_every == 0

    def track_temporal(self, prev: HybridState, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        if len(prev.points_left) == 0:
            return empty_points(), empty_ints(), empty_ints(), 0.0

        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            prev.image_left,
            image,
            prev.points_left.astype(np.float32),
            None,
            winSize=(21, 21),
            maxLevel=3,
        )
        if next_points is None or status is None:
            return empty_points(), empty_ints(), empty_ints(), 0.0

        reverse_points, reverse_status, _ = cv2.calcOpticalFlowPyrLK(
            image,
            prev.image_left,
            next_points,
            None,
            winSize=(21, 21),
            maxLevel=1,
        )
        status = status.reshape(-1).astype(bool)
        reverse_status = reverse_status.reshape(-1).astype(bool) if reverse_status is not None else np.zeros_like(status)
        fb_error = np.linalg.norm(reverse_points.reshape(-1, 2) - prev.points_left, axis=1) if reverse_points is not None else np.inf
        border = np.array([in_border(p, image.shape[1], image.shape[0]) for p in next_points.reshape(-1, 2)])
        valid = status & reverse_status & (fb_error <= 0.5) & border

        tracked = next_points.reshape(-1, 2)[valid].astype(np.float32)
        ids = prev.track_ids[valid].astype(np.int64)
        counts = (prev.track_counts[valid] + 1).astype(np.int64)
        flow = np.linalg.norm(tracked - prev.points_left[valid], axis=1) if len(tracked) else np.array([], dtype=np.float32)
        mean_flow = float(np.mean(flow)) if len(flow) else 0.0
        return tracked, ids, counts, mean_flow

    def add_gftt_tracks(
        self,
        image: np.ndarray,
        points: np.ndarray,
        ids: np.ndarray,
        counts: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        target = self.max_cnt - len(points)
        if target <= 0:
            return points, ids, counts, 0

        mask = np.full(image.shape[:2], 255, dtype=np.uint8)
        for p in sort_tracks(points, counts):
            cv2.circle(mask, tuple(np.round(p).astype(int)), self.min_dist, 0, -1)
        detected = cv2.goodFeaturesToTrack(image, target, 0.01, self.min_dist, mask=mask)
        if detected is None:
            return points, ids, counts, 0

        new_points = detected.reshape(-1, 2).astype(np.float32)
        new_ids = np.array([next(self.next_track_id) for _ in range(len(new_points))], dtype=np.int64)
        new_counts = np.ones((len(new_points),), dtype=np.int64)
        return (
            np.concatenate([points, new_points], axis=0),
            np.concatenate([ids, new_ids], axis=0),
            np.concatenate([counts, new_counts], axis=0),
            len(new_points),
        )

    def detect_new_tracks(self, image: np.ndarray, max_count: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        detected = cv2.goodFeaturesToTrack(image, max_count, 0.01, self.min_dist)
        if detected is None:
            return empty_points(), empty_ints(), empty_ints()
        points = detected.reshape(-1, 2).astype(np.float32)
        ids = np.array([next(self.next_track_id) for _ in range(len(points))], dtype=np.int64)
        counts = np.ones((len(points),), dtype=np.int64)
        return points, ids, counts

    def recover_tracks_with_lightglue(
        self,
        prev: HybridState,
        image: np.ndarray,
        points: np.ndarray,
        ids: np.ndarray,
        counts: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, int]:
        feats_prev, keypoints_prev = self.features_for_state(prev)
        feats_cur, keypoints_cur = self.extract(image)
        if len(keypoints_prev) == 0 or len(keypoints_cur) == 0:
            return points, ids, counts, 0

        matches = self.match(feats_prev, feats_cur)
        if len(matches) == 0:
            return points, ids, counts, 0

        existing_ids = set(int(x) for x in ids.tolist())
        accepted_points: List[np.ndarray] = []
        accepted_ids: List[int] = []
        accepted_counts: List[int] = []

        prev_points = prev.points_left
        for prev_kp_index, cur_kp_index in matches:
            prev_kp = keypoints_prev[prev_kp_index]
            cur_kp = keypoints_cur[cur_kp_index]
            nearest = nearest_point_index(prev_points, prev_kp)
            if nearest < 0:
                continue
            if np.linalg.norm(prev_points[nearest] - prev_kp) > self.recovery_match_radius:
                continue
            track_id = int(prev.track_ids[nearest])
            if track_id in existing_ids:
                continue
            if not in_border(cur_kp, image.shape[1], image.shape[0]):
                continue
            if min_distance(cur_kp, points, accepted_points) < self.min_dist * 0.5:
                continue
            accepted_points.append(cur_kp.astype(np.float32))
            accepted_ids.append(track_id)
            accepted_counts.append(int(prev.track_counts[nearest]) + 1)
            existing_ids.add(track_id)
            if len(accepted_points) >= self.max_recoveries_per_frame:
                break

        if not accepted_points:
            return points, ids, counts, 0

        return (
            np.concatenate([points, np.vstack(accepted_points).astype(np.float32)], axis=0),
            np.concatenate([ids, np.array(accepted_ids, dtype=np.int64)], axis=0),
            np.concatenate([counts, np.array(accepted_counts, dtype=np.int64)], axis=0),
            len(accepted_points),
        )

    def features_for_state(self, state: HybridState) -> Tuple[dict, np.ndarray]:
        if state.features_left is None or state.keypoints_left is None:
            state.features_left, state.keypoints_left = self.extract(state.image_left)
        return state.features_left, state.keypoints_left

    @torch.inference_mode()
    def extract(self, image: np.ndarray) -> Tuple[dict, np.ndarray]:
        tensor = torch.from_numpy(np.ascontiguousarray(image).copy()).float()[None, None].to(self.device) / 255.0
        feats = self.extractor.extract(tensor)
        keypoints = feats["keypoints"][0].detach().cpu().numpy().astype(np.float32)
        return feats, keypoints

    @torch.inference_mode()
    def match(self, feats0: dict, feats1: dict) -> np.ndarray:
        matches01 = self.matcher({"image0": feats0, "image1": feats1})
        matches0 = matches01["matches0"][0].detach().cpu().numpy()
        pairs = [(i, int(j)) for i, j in enumerate(matches0) if j >= 0]
        return np.asarray(pairs, dtype=np.int64)

    def track_stereo(
        self,
        left: np.ndarray,
        right: Optional[np.ndarray],
        left_points: np.ndarray,
        left_ids: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if right is None or len(left_points) == 0:
            return empty_points(), empty_ints()
        right_points, status, _ = cv2.calcOpticalFlowPyrLK(
            left, right, left_points.astype(np.float32), None, winSize=(21, 21), maxLevel=3
        )
        if right_points is None or status is None:
            return empty_points(), empty_ints()
        reverse_left, reverse_status, _ = cv2.calcOpticalFlowPyrLK(
            right, left, right_points, None, winSize=(21, 21), maxLevel=3
        )
        status = status.reshape(-1).astype(bool)
        reverse_status = reverse_status.reshape(-1).astype(bool) if reverse_status is not None else np.zeros_like(status)
        fb_error = np.linalg.norm(reverse_left.reshape(-1, 2) - left_points, axis=1) if reverse_left is not None else np.inf
        border = np.array([in_border(p, right.shape[1], right.shape[0]) for p in right_points.reshape(-1, 2)])
        valid = status & reverse_status & (fb_error <= 0.5) & border
        return right_points.reshape(-1, 2)[valid].astype(np.float32), left_ids[valid].astype(np.int64)

    def publish_features(
        self,
        stamp: rospy.Time,
        state: HybridState,
        right_points: np.ndarray,
        right_ids: np.ndarray,
    ) -> None:
        cloud = PointCloud()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frame_id
        channels = [
            ChannelFloat32(name="id"),
            ChannelFloat32(name="camera_id"),
            ChannelFloat32(name="u"),
            ChannelFloat32(name="v"),
            ChannelFloat32(name="velocity_x"),
            ChannelFloat32(name="velocity_y"),
        ]

        previous_by_id: Dict[int, np.ndarray] = {}
        dt = 1.0
        if self.previous is not None:
            previous_by_id = {
                int(track_id): point for track_id, point in zip(self.previous.track_ids, self.previous.points_left)
            }
            dt = max((state.timestamp - self.previous.timestamp).to_sec(), 1e-6)

        for track_id, point in zip(state.track_ids, state.points_left):
            append_feature(cloud, channels, int(track_id), 0, point, self.cam0, previous_by_id, dt)

        right_by_id = {int(track_id): point for track_id, point in zip(right_ids, right_points)}
        for track_id in state.track_ids:
            right_point = right_by_id.get(int(track_id))
            if right_point is not None:
                append_feature(cloud, channels, int(track_id), 1, right_point, self.cam1, None, dt)

        cloud.channels = channels
        self.publisher.publish(cloud)


def append_feature(
    cloud: PointCloud,
    channels: list,
    track_id: int,
    camera_id: int,
    point: np.ndarray,
    camera: CameraModel,
    previous_by_id: Optional[Dict[int, np.ndarray]],
    dt: float,
) -> None:
    u = float(point[0])
    v = float(point[1])
    x, y = camera.normalize(point)
    cloud.points.append(Point32(x=x, y=y, z=1.0))
    channels[0].values.append(float(track_id))
    channels[1].values.append(float(camera_id))
    channels[2].values.append(u)
    channels[3].values.append(v)
    if previous_by_id is None or track_id not in previous_by_id:
        channels[4].values.append(0.0)
        channels[5].values.append(0.0)
    else:
        previous = previous_by_id[track_id]
        prev_x, prev_y = camera.normalize(previous)
        channels[4].values.append(float((x - prev_x) / dt))
        channels[5].values.append(float((y - prev_y) / dt))


def empty_points() -> np.ndarray:
    return np.empty((0, 2), dtype=np.float32)


def empty_ints() -> np.ndarray:
    return np.empty((0,), dtype=np.int64)


def in_border(point: np.ndarray, width: int, height: int) -> bool:
    x, y = float(point[0]), float(point[1])
    return 1.0 <= x < width - 1.0 and 1.0 <= y < height - 1.0


def nearest_point_index(points: np.ndarray, query: np.ndarray) -> int:
    if len(points) == 0:
        return -1
    return int(np.argmin(np.linalg.norm(points - query, axis=1)))


def min_distance(query: np.ndarray, points: np.ndarray, extra_points: List[np.ndarray]) -> float:
    values = []
    if len(points):
        values.append(float(np.min(np.linalg.norm(points - query, axis=1))))
    if extra_points:
        values.append(float(np.min(np.linalg.norm(np.vstack(extra_points) - query, axis=1))))
    return min(values) if values else float("inf")


def sort_tracks(points: np.ndarray, counts: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    order = np.argsort(-counts)
    return points[order]


def main() -> None:
    rospy.init_node("hybrid_klt_lightglue_feature_tracker")
    HybridKltLightGlueTrackerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
