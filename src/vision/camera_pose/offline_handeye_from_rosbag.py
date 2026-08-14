"""Compute eye-in-hand calibration offline from /synced_msg in a ROS 2 bag.

Each SyncedMsg is expected to contain:
  * image_raw: sensor_msgs/CompressedImage
  * fk_pose: geometry_msgs/PoseStamped describing gripper -> base

The script first finds stable, sufficiently different robot poses.  AprilTag
detection is then run only on one image from each stable pose.  The detector's
tag -> camera pose and the FK gripper -> base pose are passed to
cv2.calibrateHandEye(), which returns camera -> gripper.
"""

import argparse
from collections import deque
import json
import math
from pathlib import Path

import cv2
import numpy as np
from pupil_apriltags import Detector
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline AprilTag detection and eye-in-hand calibration"
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--calib-file",
        type=Path,
        required=True,
        help="camera_calibration.npz containing camera_matrix/dist_coeffs",
    )
    parser.add_argument("--topic", default="/synced_msg")
    parser.add_argument("--tag-family", default="tag25h9")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.09, help="metres")
    parser.add_argument("--detector-threads", type=int, default=4)
    parser.add_argument(
        "--quad-decimate",
        type=float,
        default=1.0,
        help="1.0 favours calibration accuracy; values >1 improve speed",
    )
    parser.add_argument("--method", choices=METHODS, default="park")
    parser.add_argument(
        "--stable-window-sec",
        type=float,
        default=0.35,
        help="time over which FK must remain still; use 0 for preselected samples",
    )
    parser.add_argument("--stable-translation-mm", type=float, default=0.5)
    parser.add_argument("--stable-rotation-deg", type=float, default=0.25)
    parser.add_argument(
        "--tag-retry-sec",
        type=float,
        default=0.5,
        help="minimum interval between failed Tag detection attempts at one pose",
    )
    parser.add_argument(
        "--min-pose-translation-mm",
        type=float,
        default=10.0,
        help="minimum translation from the last accepted pose",
    )
    parser.add_argument(
        "--min-pose-rotation-deg",
        type=float,
        default=5.0,
        help="minimum rotation from the last accepted pose",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("handeye_result.json"),
    )
    return parser.parse_args()


def quaternion_to_rotation(x, y, z, w):
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("FK pose contains a zero-length quaternion")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_to_quaternion(rotation):
    # OpenCV Rodrigues gives a compact and numerically stable axis-angle form.
    rotation_vector, _ = cv2.Rodrigues(rotation)
    angle = float(np.linalg.norm(rotation_vector))
    if angle < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    axis = rotation_vector.reshape(3) / angle
    scale = math.sin(angle / 2.0)
    return [
        float(axis[0] * scale),
        float(axis[1] * scale),
        float(axis[2] * scale),
        float(math.cos(angle / 2.0)),
    ]


def pose_to_transform(pose):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_rotation(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return transform


def pose_distance(transform_a, transform_b):
    translation_m = float(
        np.linalg.norm(transform_a[:3, 3] - transform_b[:3, 3])
    )
    relative_rotation = transform_a[:3, :3].T @ transform_b[:3, :3]
    cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    rotation_deg = math.degrees(math.acos(float(cosine)))
    return translation_m, rotation_deg


def message_stamp_ns(message, bag_stamp_ns):
    stamp = message.synced_header.stamp
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return stamp_ns if stamp_ns > 0 else int(bag_stamp_ns)


def infer_storage_id(bag_path):
    search_root = bag_path if bag_path.is_dir() else bag_path.parent
    if any(search_root.glob("*.mcap")):
        return "mcap"
    if any(search_root.glob("*.db3")):
        return "sqlite3"
    raise RuntimeError(
        f"Could not infer rosbag storage type in {bag_path}; expected .mcap or .db3"
    )


def open_bag(bag_path, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=infer_storage_id(bag_path),
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types))
        raise RuntimeError(
            f"Topic {topic!r} is not present in the bag. Available: {available}"
        )
    return reader, get_message(topic_types[topic])


class OfflineHandEye:
    def __init__(self, args):
        self.args = args

        with np.load(args.calib_file) as calibration:
            self.camera_matrix = np.asarray(
                calibration["camera_matrix"], dtype=np.float64
            )
            self.dist_coeffs = np.asarray(
                calibration["dist_coeffs"], dtype=np.float64
            )
            self.calibration_image_size = (
                tuple(int(value) for value in calibration["image_size"])
                if "image_size" in calibration
                else None
            )

        self.detector = Detector(
            families=args.tag_family,
            nthreads=args.detector_threads,
            quad_decimate=args.quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        self.rectification_size = None
        self.map1 = None
        self.map2 = None
        self.window = deque()
        self.accepted_gripper_to_base = []
        self.accepted_target_to_camera = []
        self.last_accepted_gripper_to_base = None
        self.locked_at_current_pose = False
        self.last_detection_attempt_ns = None
        self.examined_messages = 0
        self.stable_candidates = 0
        self.no_tag_count = 0

    def initialize_rectification(self, image_size):
        if (
            self.calibration_image_size is not None
            and self.calibration_image_size != image_size
        ):
            raise RuntimeError(
                "Recorded image resolution "
                f"{image_size} differs from calibration resolution "
                f"{self.calibration_image_size}. Recalibrate at the recorded resolution."
            )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.camera_matrix,
            image_size,
            cv2.CV_32FC1,
        )
        self.rectification_size = image_size

    def detect_target(self, compressed_image):
        encoded = np.frombuffer(bytes(compressed_image.data), dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("OpenCV could not decode a CompressedImage")

        height, width = frame.shape[:2]
        image_size = (width, height)
        if self.rectification_size != image_size:
            self.initialize_rectification(image_size)

        rectified = cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
        )
        gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                float(self.camera_matrix[0, 0]),
                float(self.camera_matrix[1, 1]),
                float(self.camera_matrix[0, 2]),
                float(self.camera_matrix[1, 2]),
            ],
            tag_size=self.args.tag_size,
        )

        for detection in detections:
            if int(detection.tag_id) == self.args.tag_id:
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :3] = np.asarray(
                    detection.pose_R, dtype=np.float64
                )
                transform[:3, 3] = np.asarray(
                    detection.pose_t, dtype=np.float64
                ).reshape(3)
                return transform
        return None

    def is_stable(self):
        if not self.window:
            return False
        if self.args.stable_window_sec <= 0.0:
            return True

        required_ns = int(self.args.stable_window_sec * 1_000_000_000)
        if self.window[-1][0] - self.window[0][0] < required_ns:
            return False

        reference = self.window[0][1]
        max_translation_m = self.args.stable_translation_mm / 1000.0
        for _, transform, _ in self.window:
            translation_m, rotation_deg = pose_distance(reference, transform)
            if (
                translation_m > max_translation_m
                or rotation_deg > self.args.stable_rotation_deg
            ):
                return False
        return True

    def pose_is_new(self, transform):
        if self.last_accepted_gripper_to_base is None:
            return True
        translation_m, rotation_deg = pose_distance(
            self.last_accepted_gripper_to_base, transform
        )
        return (
            translation_m >= self.args.min_pose_translation_mm / 1000.0
            or rotation_deg >= self.args.min_pose_rotation_deg
        )

    def process(self, message, bag_stamp_ns):
        self.examined_messages += 1
        timestamp_ns = message_stamp_ns(message, bag_stamp_ns)
        gripper_to_base = pose_to_transform(message.fk_pose.pose)
        self.window.append((timestamp_ns, gripper_to_base, message.image_raw))

        required_window_ns = int(
            max(self.args.stable_window_sec, 0.0) * 1_000_000_000
        )
        if required_window_ns <= 0:
            while len(self.window) > 1:
                self.window.popleft()
        else:
            # Keep the shortest suffix that still spans the requested window.
            # A simple "delete everything older than the window" rule would
            # leave slightly less than 0.35 s at discrete frame intervals and
            # could therefore prevent stability from ever becoming true.
            while (
                len(self.window) > 1
                and timestamp_ns - self.window[1][0] >= required_window_ns
            ):
                self.window.popleft()

        if self.locked_at_current_pose:
            if self.pose_is_new(gripper_to_base):
                self.locked_at_current_pose = False
            else:
                return

        if not self.is_stable():
            return

        self.stable_candidates += 1
        if not self.pose_is_new(gripper_to_base):
            self.locked_at_current_pose = True
            return

        retry_ns = int(max(self.args.tag_retry_sec, 0.0) * 1_000_000_000)
        if (
            self.last_detection_attempt_ns is not None
            and timestamp_ns - self.last_detection_attempt_ns < retry_ns
        ):
            return
        self.last_detection_attempt_ns = timestamp_ns

        target_to_camera = self.detect_target(message.image_raw)
        if target_to_camera is None:
            self.no_tag_count += 1
            return

        self.accepted_gripper_to_base.append(gripper_to_base)
        self.accepted_target_to_camera.append(target_to_camera)
        self.last_accepted_gripper_to_base = gripper_to_base
        self.locked_at_current_pose = True
        print(
            f"Accepted sample {len(self.accepted_gripper_to_base):02d}: "
            f"stamp={timestamp_ns}, tag={self.args.tag_id}"
        )

    def solve(self):
        sample_count = len(self.accepted_gripper_to_base)
        if sample_count < 5:
            raise RuntimeError(
                f"Only {sample_count} valid poses were found; at least 5 are required "
                "and 15-30 diverse poses are recommended"
            )

        rotations_gripper_to_base = [
            transform[:3, :3] for transform in self.accepted_gripper_to_base
        ]
        translations_gripper_to_base = [
            transform[:3, 3].reshape(3, 1)
            for transform in self.accepted_gripper_to_base
        ]
        rotations_target_to_camera = [
            transform[:3, :3] for transform in self.accepted_target_to_camera
        ]
        translations_target_to_camera = [
            transform[:3, 3].reshape(3, 1)
            for transform in self.accepted_target_to_camera
        ]

        rotation_camera_to_gripper, translation_camera_to_gripper = (
            cv2.calibrateHandEye(
                rotations_gripper_to_base,
                translations_gripper_to_base,
                rotations_target_to_camera,
                translations_target_to_camera,
                method=METHODS[self.args.method],
            )
        )

        camera_to_gripper = np.eye(4, dtype=np.float64)
        camera_to_gripper[:3, :3] = rotation_camera_to_gripper
        camera_to_gripper[:3, 3] = np.asarray(
            translation_camera_to_gripper
        ).reshape(3)

        # For a fixed calibration target, base -> target should be constant.
        base_to_targets = [
            gripper_to_base @ camera_to_gripper @ target_to_camera
            for gripper_to_base, target_to_camera in zip(
                self.accepted_gripper_to_base,
                self.accepted_target_to_camera,
            )
        ]
        target_positions = np.asarray(
            [transform[:3, 3] for transform in base_to_targets]
        )
        translation_residual_mm = np.linalg.norm(
            target_positions - target_positions.mean(axis=0), axis=1
        ) * 1000.0

        reference_rotation = base_to_targets[0][:3, :3]
        rotation_residual_deg = np.asarray(
            [
                pose_distance(
                    np.block(
                        [
                            [reference_rotation, np.zeros((3, 1))],
                            [np.zeros((1, 3)), np.ones((1, 1))],
                        ]
                    ),
                    np.block(
                        [
                            [transform[:3, :3], np.zeros((3, 1))],
                            [np.zeros((1, 3)), np.ones((1, 1))],
                        ]
                    ),
                )[1]
                for transform in base_to_targets
            ]
        )

        result = {
            "calibration_type": "eye_in_hand",
            "method": self.args.method,
            "sample_count": sample_count,
            "transform": "camera_to_gripper",
            "camera_to_gripper_matrix": camera_to_gripper.tolist(),
            "translation_m": camera_to_gripper[:3, 3].tolist(),
            "quaternion_xyzw": rotation_to_quaternion(
                camera_to_gripper[:3, :3]
            ),
            "fixed_target_consistency": {
                "translation_residual_mean_mm": float(
                    translation_residual_mm.mean()
                ),
                "translation_residual_max_mm": float(
                    translation_residual_mm.max()
                ),
                "rotation_residual_mean_deg_to_first": float(
                    rotation_residual_deg.mean()
                ),
                "rotation_residual_max_deg_to_first": float(
                    rotation_residual_deg.max()
                ),
            },
        }
        self.args.output.parent.mkdir(parents=True, exist_ok=True)
        self.args.output.write_text(
            json.dumps(result, indent=2), encoding="utf-8"
        )
        return result


def main():
    args = parse_args()
    if args.tag_size <= 0.0:
        raise ValueError("--tag-size must be positive")

    reader, message_type = open_bag(args.bag, args.topic)
    calibration = OfflineHandEye(args)

    while reader.has_next():
        topic, serialized_data, bag_stamp_ns = reader.read_next()
        if topic != args.topic:
            continue
        message = deserialize_message(serialized_data, message_type)
        calibration.process(message, bag_stamp_ns)

    result = calibration.solve()
    consistency = result["fixed_target_consistency"]
    print(f"\nSaved hand-eye result to: {args.output}")
    print("camera -> gripper:")
    print(np.asarray(result["camera_to_gripper_matrix"]))
    print(
        "fixed-target residual: "
        f"mean {consistency['translation_residual_mean_mm']:.3f} mm, "
        f"max {consistency['translation_residual_max_mm']:.3f} mm"
    )


if __name__ == "__main__":
    main()
