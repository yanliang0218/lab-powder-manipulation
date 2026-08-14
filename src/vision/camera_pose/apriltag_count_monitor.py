#!/usr/bin/env python3

"""
Project: Lab Powder Manipulation
File: apriltag_count_monitor.py
Author: Liang Yan

Purpose:
    subscribe to usb_cam's raw image stream and print how many AprilTags are currently visible.
    this is to ensure that the user always keeps at least one tag in view

Important:
    - It is intended only as a lightweight visibility/alignment check, and it
       does not perform pose estimation.
"""

import threading
import time

import cv2

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from pupil_apriltags import Detector
from sensor_msgs.msg import Image


class AprilTagCountMonitor(Node):
    def __init__(self):
        super().__init__("apriltag_count_monitor")

        self.image_topic = self.declare_parameter(
            "image_topic",
            "/webcam/image_raw"
        ).value

        self.tag_family = self.declare_parameter(
            "tag_family",
            "tag25h9"
        ).value

        # Limit the terminal output to a readable rate.
        self.print_period_sec = self.declare_parameter(
            "print_period_sec",
            0.25
        ).value

        self.bridge = CvBridge()

        # Pose estimation is intentionally disabled; only tag count is needed
        # The tag size is only needed for pose estimation, so it does not need
        # to be specified here
        self.detector = Detector(
            families=self.tag_family,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        #when this thread runs, no other thread can access the latest_frame variable,
        #and the detection thread will wait for a new frame to be available before processing it
        self.latest_frame_lock = threading.Lock()
        self.latest_frame_event = threading.Event()

        #this variable will hold the latest frame received from the image topic, and it will be processed by the detection thread
        self.latest_frame = None
        self.running = True

        self.last_print_time = 0.0

        # everytime a new image is received, the image_callback function will be called
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detection_thread = threading.Thread(
            target=self.detection_worker,
            daemon=True,
        )
        self.detection_thread.start()

        self.get_logger().info(
            f"AprilTag preview monitor subscribed to {self.image_topic}"
        )

    def image_callback(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(
                f"Failed to convert image: {error}"
            )
            return

        with self.latest_frame_lock:
            #assign the current frame to the latest_frame variable,
            #which will be processed by the detection thread
            self.latest_frame = frame

        self.latest_frame_event.set()

    def detection_worker(self):
        while self.running and rclpy.ok():
            self.latest_frame_event.wait(timeout=0.1)

            if not self.running:
                break

            self.latest_frame_event.clear()

            with self.latest_frame_lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(
                gray,
                estimate_tag_pose=False,
                camera_params=None,
                tag_size=None,
            )

            now = time.monotonic()

            if now - self.last_print_time >= self.print_period_sec:
                tag_count = len(detections)

                if tag_count > 0:
                    self.get_logger().info(
                        f"APRILTAGS DETECTED: {tag_count}"
                    )
                else:
                    self.get_logger().info(
                        "APRILTAGS DETECTED: 0"
                    )

                self.last_print_time = now

    def destroy_node(self):
        self.running = False
        self.latest_frame_event.set()

        if self.detection_thread.is_alive():
            self.detection_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCountMonitor()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
