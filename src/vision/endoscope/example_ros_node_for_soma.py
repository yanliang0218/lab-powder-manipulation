#!/usr/bin/env python3

"""
Project: Lab Powder Manipulation
File: collect_vision_dataset.py
Author: Lang Yun

Description:
    This script collects a vision dataset using a camera and AprilTag detection.
    It captures images, detects AprilTags, estimates the camera pose relative to the tags,
    and saves the images and pose information to a dataset folder.
Required: 
    - OpenCV (cv2)
    - NumPy (np)
    - pupil_apriltags (Detector)

"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from arm_v2_description.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
from pupil_apriltags import Detector

from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class WebcamAprilTagPublisher(Node):
    def __init__(self):
        super().__init__("webcam_apriltag_publisher")

        # =========================
        # Parameters
        # =========================
        self.camera_id = self.declare_parameter("camera_id", 0).value
        self.image_width = self.declare_parameter("image_width", 1280).value
        self.image_height = self.declare_parameter("image_height", 720).value
        self.target_fps = self.declare_parameter("target_fps", 150).value
        self.tag_family = self.declare_parameter("tag_family", "tag25h9").value
        self.tag_size = self.declare_parameter("tag_size", 0.09).value

        # local path to camera calibration file (npz format) produced by camera_calibration.py
        #self.calib_file = self.declare_parameter("calib_file","/home/stephen/ros2_ws/src/arm_v2_description/config/camera_calibration.npz").value

        default_calib_file = str(
            Path(get_package_share_directory("arm_v2_description"))
            / "config"
            / "camera_calibration.npz"
        )

        self.calib_file = self.declare_parameter(
            "calib_file",
            default_calib_file
        ).value

        # Boolean parameters that control whether to publish debug image and show images used in AprilTag detection
        self.if_publish_debug_image = self.declare_parameter("if_publish_debug_image",False).value
        self.if_show_image = self.declare_parameter("if_show_image", False).value

        # =========================
        # Publishers
        # =========================

        # Initialize CvBridge for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Create publishers for raw webcam images and AprilTag pose information
        self.image_pub = self.create_publisher(Image,"/webcam/image_raw",10)
        self.tag_pose_pub = self.create_publisher(AprilTagDetectionArray,"/webcam/tag_pose",10)

        # Create a publisher for debug images if the parameter is set to True
        if self.if_publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image,"/webcam/image_debug",10)
        else:
            self.debug_image_pub = None

        # =========================
        # Load camera calibration
        # =========================

        # Load camera calibration data from the specified file
        calib = np.load(self.calib_file)

        self.camera_matrix = calib["camera_matrix"].astype(np.float64)
        self.dist_coeffs = calib["dist_coeffs"].astype(np.float64)

        self.get_logger().info("Loaded camera calibration.")
        self.get_logger().info(f"camera_matrix:\n{self.camera_matrix}")
        self.get_logger().info(f"dist_coeffs:\n{self.dist_coeffs}")

        # extract camera intrinsic parameters from the camera matrix
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

        # =========================
        # AprilTag detector
        # =========================

        # Initialize the AprilTag detector with the specified tag family and parameters
        self.detector = Detector(
            families=self.tag_family,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # =========================
        # Camera setup
        # =========================

        # Open the webcam using OpenCV's VideoCapture with the specified camera ID and V4L2 backend
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("Cannot open webcam.")

        self.get_logger().info("Webcam opened.")
        self.get_logger().info(
            f"Actual width: {self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}"
        )
        self.get_logger().info(
            f"Actual height: {self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}"
        )
        self.get_logger().info(
            f"Actual fps: {self.cap.get(cv2.CAP_PROP_FPS)}"
        )

        # =========================
        # Timer
        # =========================

        # create a timer at the specified target FPS to call the timer_callback function
        timer_period = 1.0 / float(self.target_fps)

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        # store a count to keep track of how many frames have been captured and processed
        self.frame_count = 0

    def timer_callback(self):

        # capture a frame from the webcam
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Cannot read webcam frame.")
            return

        self.frame_count += 1

        # record the current ROS time for timestamping the AprilTag pose message and images
        now = self.get_clock().now()
        now_msg = now.to_msg()

        # ============================================================
        # 1. Publish raw webcam image
        # ============================================================
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = now_msg
        image_msg.header.frame_id = "webcam_frame"

        self.image_pub.publish(image_msg)

        # ============================================================
        # 2. AprilTag detection
        # ============================================================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size
        )

        debug_image = frame.copy()

        #create an AprilTagDetectionArray message to hold the pose information for all detected tags from this frame
        pose_msg = AprilTagDetectionArray()

        # since all the detections are from the same frame, we can use the same timestamp and frame_id for the entire array
        pose_msg.header.stamp = now_msg
        pose_msg.header.frame_id = "webcam_frame"
   
        #iterate through the detected AprilTags and populate each AprilTagDetection message with the tag ID, size, and pose information
        for tag in results:

            R_camera_tag = tag.pose_R
            t_camera_tag = tag.pose_t

            # Create a new AprilTagDetection message for each detected tag
            tag_detection = AprilTagDetection()
            tag_detection.tag_id = int(tag.tag_id)
            tag_detection.tag_size = float(self.tag_size)
            tag_detection.decision_margin = float(tag.decision_margin)
            tag_detection.hamming = int(tag.hamming)

            tag_detection.pose.position.x = float(t_camera_tag[0, 0])
            tag_detection.pose.position.y = float(t_camera_tag[1, 0])
            tag_detection.pose.position.z = float(t_camera_tag[2, 0])

            # Convert the rotation matrix to a quaternion for the orientation
            quat = Rotation.from_matrix(R_camera_tag).as_quat()

            tag_detection.pose.orientation.x = float(quat[0])
            tag_detection.pose.orientation.y = float(quat[1])
            tag_detection.pose.orientation.z = float(quat[2])
            tag_detection.pose.orientation.w = float(quat[3])

            # Append the the current tag detection information pose_msg.detections
            pose_msg.detections.append(tag_detection)

            # Draw debug overlay
            corners = tag.corners.astype(int)

            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(debug_image, pt1, pt2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(debug_image, center, 5, (0, 255, 0), -1)

            cv2.putText(
                debug_image,
                f"ID {tag.tag_id}",
                (center[0] + 10, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        #count number of detections made and publish the AprilTag pose message for all detected tags from the current frame
        pose_msg.number_of_tags_detected = len(pose_msg.detections)
        #publish the AprilTag pose message for all detected tags from the current frame
        self.tag_pose_pub.publish(pose_msg)

        # ============================================================
        # 3. Optional debug image publish / display
        # ============================================================
        if self.if_publish_debug_image and self.debug_image_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
            debug_msg.header.stamp = now_msg
            debug_msg.header.frame_id = "webcam_frame"
            self.debug_image_pub.publish(debug_msg)

        if self.if_show_image:
            display = cv2.resize(
                debug_image,
                None,
                fx=0.5,
                fy=0.5,
                interpolation=cv2.INTER_AREA
            )
            cv2.imshow("webcam apriltag publisher", display)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WebcamAprilTagPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()