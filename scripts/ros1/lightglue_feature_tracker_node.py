#!/usr/bin/env python3
"""ROS1 LightGlue feature tracker compatible with VINS-Fusion feature messages."""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import message_filters
import numpy as np
import rospy
import torch
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32, Image, PointCloud


@dataclass
class CameraModel:
    camera_matrix: np.ndarray
    distortion: np.ndarray

    @classmethod
    def from_yaml(cls, path: str) -> "CameraModel":
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(f"failed to open camera yaml: {path}")
        model_type = fs.getNode("model_type").string()
        projection = fs.getNode("projection_parameters")
        distortion = fs.getNode("distortion_parameters")

        if model_type == "PINHOLE":
            fx = projection.getNode("fx").real()
            fy = projection.getNode("fy").real()
            cx = projection.getNode("cx").real()
            cy = projection.getNode("cy").real()
        else:
            fx = projection.getNode("gamma1").real()
            fy = projection.getNode("gamma2").real()
            cx = projection.getNode("u0").real()
            cy = projection.getNode("v0").real()

        k1 = distortion.getNode("k1").real()
        k2 = distortion.getNode("k2").real()
        p1 = distortion.getNode("p1").real()
        p2 = distortion.getNode("p2").real()
        fs.release()

        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        distortion_vector = np.array([k1, k2, p1, p2], dtype=np.float64)
        return cls(camera_matrix=camera_matrix, distortion=distortion_vector)

    def normalize(self, keypoint: np.ndarray) -> Tuple[float, float]:
        point = np.asarray(keypoint, dtype=np.float64).reshape(1, 1, 2)
        undistorted = cv2.undistortPoints(point, self.camera_matrix, self.distortion)
        return float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])


@dataclass
class TrackedFrame:
    timestamp: rospy.Time
    keypoints_left: np.ndarray
    track_ids: np.ndarray
    features_left: dict


class LightGlueTrackerNode:
    def __init__(self) -> None:
        self.image0_topic = rospy.get_param("~image0_topic", "/cam0/image_raw")
        self.image1_topic = rospy.get_param("~image1_topic", "/cam1/image_raw")
        self.feature_topic = rospy.get_param("~feature_topic", "/feature_tracker/feature")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.max_num_keypoints = int(rospy.get_param("~max_num_keypoints", 512))
        self.match_confidence = float(rospy.get_param("~match_confidence", 0.1))
        self.resize_long_edge = int(rospy.get_param("~resize_long_edge", 0))
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
        self.previous: Optional[TrackedFrame] = None
        self.next_track_id = itertools.count()
        self.publisher = rospy.Publisher(self.feature_topic, PointCloud, queue_size=20)

        self.image0_sub = message_filters.Subscriber(self.image0_topic, Image)
        if self.publish_stereo:
            self.image1_sub = message_filters.Subscriber(self.image1_topic, Image)
            sync = message_filters.ApproximateTimeSynchronizer(
                [self.image0_sub, self.image1_sub], queue_size=20, slop=0.003
            )
            sync.registerCallback(self.on_stereo)
            self.sync = sync
        else:
            self.image0_sub.registerCallback(self.on_mono)

        rospy.logwarn(
            "LightGlue tracker ready: device=%s max_keypoints=%d stereo=%s",
            self.device,
            self.max_num_keypoints,
            self.publish_stereo,
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
        feats0, keypoints0 = self.extract(image0)
        current_to_previous = self.match_current_to_previous(feats0, len(keypoints0))

        if self.previous is None:
            track_ids = np.array([next(self.next_track_id) for _ in range(len(keypoints0))], dtype=np.int64)
        else:
            track_ids = np.array([next(self.next_track_id) for _ in range(len(keypoints0))], dtype=np.int64)
            valid = current_to_previous >= 0
            track_ids[valid] = self.previous.track_ids[current_to_previous[valid]]

        stereo_current_to_right = None
        keypoints1 = None
        if self.publish_stereo and msg1 is not None:
            image1 = image_msg_to_mono8(msg1)
            feats1, keypoints1 = self.extract(image1)
            stereo_current_to_right = self.match_left_to_right(feats0, feats1, len(keypoints0))

        tracked = TrackedFrame(
            timestamp=msg0.header.stamp,
            keypoints_left=keypoints0,
            track_ids=track_ids,
            features_left=feats0,
        )
        self.publish_features(msg0.header.stamp, tracked, keypoints1, stereo_current_to_right)
        self.previous = tracked
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0
        stereo_matches = int(np.sum(stereo_current_to_right >= 0)) if stereo_current_to_right is not None else 0
        rospy.loginfo_throttle(
            2.0,
            "LightGlue frame stamp=%.3f left_kp=%d stereo_matches=%d elapsed_ms=%.1f",
            msg0.header.stamp.to_sec(),
            len(keypoints0),
            stereo_matches,
            elapsed_ms,
        )

    @torch.inference_mode()
    def extract(self, image: np.ndarray) -> Tuple[dict, np.ndarray]:
        if self.resize_long_edge > 0:
            h, w = image.shape[:2]
            scale = self.resize_long_edge / max(h, w)
            if scale < 1.0:
                image = cv2.resize(image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
            else:
                scale = 1.0
        else:
            scale = 1.0

        tensor = torch.from_numpy(np.ascontiguousarray(image)).float()[None, None].to(self.device) / 255.0
        feats = self.extractor.extract(tensor)
        keypoints = feats["keypoints"][0].detach().cpu().numpy()
        if scale != 1.0:
            keypoints = keypoints / scale
            feats["keypoints"] = feats["keypoints"] / scale
        return feats, keypoints

    @torch.inference_mode()
    def match_current_to_previous(self, feats0: dict, current_count: int) -> np.ndarray:
        if self.previous is None:
            return np.full((current_count,), -1, dtype=np.int64)
        matches01 = self.matcher({"image0": self.previous.features_left, "image1": feats0})
        previous_to_current = matches01["matches0"][0].detach().cpu().numpy()
        current_to_previous = np.full((current_count,), -1, dtype=np.int64)
        for previous_index, current_index in enumerate(previous_to_current):
            if current_index >= 0:
                current_to_previous[int(current_index)] = int(previous_index)
        return current_to_previous

    @torch.inference_mode()
    def match_left_to_right(self, feats0: dict, feats1: dict, left_count: int) -> np.ndarray:
        matches01 = self.matcher({"image0": feats0, "image1": feats1})
        left_to_right = matches01["matches0"][0].detach().cpu().numpy()
        if len(left_to_right) != left_count:
            fixed = np.full((left_count,), -1, dtype=np.int64)
            fixed[: min(left_count, len(left_to_right))] = left_to_right[: min(left_count, len(left_to_right))]
            left_to_right = fixed
        return left_to_right

    def publish_features(
        self,
        stamp: rospy.Time,
        current: TrackedFrame,
        keypoints_right: Optional[np.ndarray],
        left_to_right: Optional[np.ndarray],
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
                int(track_id): kp
                for track_id, kp in zip(self.previous.track_ids, self.previous.keypoints_left)
            }
            dt = max((current.timestamp - self.previous.timestamp).to_sec(), 1e-6)

        for left_index, (track_id, kp) in enumerate(zip(current.track_ids, current.keypoints_left)):
            self.append_feature(cloud, channels, int(track_id), 0, kp, self.cam0, previous_by_id, dt)
            if keypoints_right is not None and left_to_right is not None:
                right_index = int(left_to_right[left_index])
                if right_index >= 0 and right_index < len(keypoints_right):
                    self.append_feature(
                        cloud,
                        channels,
                        int(track_id),
                        1,
                        keypoints_right[right_index],
                        self.cam1,
                        None,
                        dt,
                    )

        cloud.channels = channels
        self.publisher.publish(cloud)

    @staticmethod
    def append_feature(
        cloud: PointCloud,
        channels: list,
        track_id: int,
        camera_id: int,
        kp: np.ndarray,
        camera: CameraModel,
        previous_by_id: Optional[Dict[int, np.ndarray]],
        dt: float,
    ) -> None:
        u = float(kp[0])
        v = float(kp[1])
        x, y = camera.normalize(kp)
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
            channels[4].values.append(float((x - camera.normalize(previous)[0]) / dt))
            channels[5].values.append(float((y - camera.normalize(previous)[1]) / dt))


def image_msg_to_mono8(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in ("mono8", "8uc1"):
        row_width = msg.step
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, row_width)
        image = data[:, : msg.width]
        return np.ascontiguousarray(image)

    if encoding in ("bgr8", "rgb8"):
        row_width = msg.step // 3
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, row_width, 3)
        image = data[:, : msg.width, :]
        code = cv2.COLOR_BGR2GRAY if encoding == "bgr8" else cv2.COLOR_RGB2GRAY
        return np.ascontiguousarray(cv2.cvtColor(image, code))

    if encoding in ("mono16", "16uc1"):
        dtype = ">u2" if msg.is_bigendian else "<u2"
        row_width = msg.step // 2
        data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)
        image16 = data[:, : msg.width]
        image8 = np.clip(image16 / 256, 0, 255).astype(np.uint8)
        return np.ascontiguousarray(image8)

    raise ValueError(f"unsupported image encoding: {msg.encoding}")


def main() -> None:
    rospy.init_node("lightglue_feature_tracker")
    LightGlueTrackerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
