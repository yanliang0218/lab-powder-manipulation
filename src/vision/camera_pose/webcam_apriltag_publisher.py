#!/usr/bin/env python3

import os
import time
import threading
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage  
from arm_v2_description.msg import AprilTagDetection, AprilTagDetectionArray

from pupil_apriltags import Detector


# ============================================================
# class for 
# ============================================================
class ThreadedCamera:

    # constructor
    def __init__(self, camera_id, width, height, fps):
        self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.ret = False
        self.frame = None
        self.frame_time_ns = 0

        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        #continuously read and store the latest frame, not a queue of recent frames
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                capture_time = time.time_ns()  # record time of capture in nanoseconds
                with self.lock:
                    self.frame = frame
                    self.frame_time_ns = capture_time
                    self.ret = ret

    def read(self):
        with self.lock:
            if self.ret and self.frame is not None:
                return True, self.frame.copy(), self.frame_time_ns
            return False, None, 0

    def release(self):
    
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.cap.release()


# ============================================================
# 2. ROS2 node that publish compressed images and AprilTag poses in two separate threads
# ============================================================
class AsyncAprilTagPublisher(Node):
    def __init__(self):
        super().__init__("webcam_apriltag_publisher")

        # ----------------------------------------------------
        # parameters
        # ----------------------------------------------------
        self.camera_id = self.declare_parameter("camera_id", 0).value
        self.image_width = self.declare_parameter("image_width", 1280).value
        self.image_height = self.declare_parameter("image_height", 720).value
        self.target_fps = self.declare_parameter("target_fps", 150).value

        self.tag_family = self.declare_parameter("tag_family", "tag25h9").value
        self.tag_size = self.declare_parameter("tag_size", 0.09).value

        # ！! I have this in config, look into what needs to change
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_calib = os.path.join(script_dir, "camera_calibration.npz")
        self.calib_file = self.declare_parameter("calib_file", default_calib).value

        # ----------------------------------------------------
        # publishers
        # ----------------------------------------------------
        self.image_pub = self.create_publisher(
            CompressedImage, 
            "/webcam/image_raw/compressed", 
            10
        )

        self.tag_pose_pub = self.create_publisher(
            Float64MultiArray, 
            "/webcam/tag_pose", 
            10
        )

        # ----------------------------------------------------
        # load camera intrinsics from calibration file
        # ----------------------------------------------------
        if not os.path.exists(self.calib_file):
            self.get_logger().error(f"cannot find calihb file: {self.calib_file}")
            raise FileNotFoundError(f"Missing calibration file: {self.calib_file}")

        calib = np.load(self.calib_file)
        self.camera_matrix = calib["camera_matrix"].astype(np.float64)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        self.get_logger().info(f"loaded calibration file: {self.calib_file}")

        # ----------------------------------------------------
        # initialize AprilTag Detector
        # ----------------------------------------------------
        self.detector = Detector(
            families=self.tag_family,
            nthreads=4,            
            quad_decimate=2.0,    
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ----------------------------------------------------
        #  threaded camera 
        # ----------------------------------------------------
        self.cam = ThreadedCamera(
            #also started a camera thread that keeps running update
            #thread_0：only responsible for publishing compressed images, no tag detection here

            self.camera_id, 
            self.image_width, 
            self.image_height, 
            self.target_fps
        )

        # ----------------------------------------------------
        # ROS Main Thread/ Thread_1: timed publisher for compressed images
        # ----------------------------------------------------
        # Main ROS thread
        timer_period = 1.0 / float(self.target_fps)
        self.image_timer = self.create_timer(timer_period, self.publish_image_callback)

        # ----------------------------------------------------
        # Thread 2: AprilTag detection worker thread
        # ----------------------------------------------------
        self.running = True
        self.detection_thread = threading.Thread(target=self._tag_detection_worker, daemon=True)
        self.detection_thread.start()

        self.get_logger().info("ROS main thread and AprilTag detection worker thread started.")
   
    def publish_image_callback(self):
        #Thread_1：only responsible for publishing compressed images, no tag detection here
        ret, frame, frame_time_ns = self.cam.read()
        if not ret or frame is None:
            return

        _, compressed_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

        msg = CompressedImage()
        msg.header.stamp.sec = int(frame_time_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(frame_time_ns % 1_000_000_000)
        msg.header.frame_id = "webcam_frame"
        msg.format = "jpeg"
        msg.data = compressed_data.tobytes()

        self.image_pub.publish(msg)

    def _tag_detection_worker(self):
        """【thread_2】：only responsible for AprilTag detection and publishing tag poses, no image publishing here"""
        while rclpy.ok() and self.running:
            ret, frame, frame_time_ns = self.cam.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            elapsed_time_ns = frame_time_ns

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            results = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=[self.fx, self.fy, self.cx, self.cy],
                tag_size=self.tag_size
            )

            if len(results) == 0:
                msg = Float64MultiArray()
                msg.data = [
                    float(elapsed_time_ns), 0.0, 0.0, -1.0,
                    *([np.nan] * 9), np.nan, np.nan, np.nan
                ]
                self.tag_pose_pub.publish(msg)
            else:
                for tag in results:
                    R_camera_tag = tag.pose_R
                    t_camera_tag = tag.pose_t

                    msg = Float64MultiArray()
                    msg.data = [
                        float(elapsed_time_ns), 1.0, float(len(results)), float(tag.tag_id),
                        float(R_camera_tag[0, 0]), float(R_camera_tag[0, 1]), float(R_camera_tag[0, 2]),
                        float(R_camera_tag[1, 0]), float(R_camera_tag[1, 1]), float(R_camera_tag[1, 2]),
                        float(R_camera_tag[2, 0]), float(R_camera_tag[2, 1]), float(R_camera_tag[2, 2]),
                        float(t_camera_tag[0, 0]), float(t_camera_tag[1, 0]), float(t_camera_tag[2, 0])
                    ]
                    self.tag_pose_pub.publish(msg)

            time.sleep(0.001)

    def destroy_node(self):
        self.running = False
        self.cam.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AsyncAprilTagPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()