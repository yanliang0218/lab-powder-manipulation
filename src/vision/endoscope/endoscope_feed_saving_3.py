#!/usr/bin/env python3
import os
import time  # <-- 1. Imported the standard time module

# Clear environment variables to prevent Snap/OpenCV crashes
if 'GTK_PATH' in os.environ:
    del os.environ['GTK_PATH']
if 'LD_LIBRARY_PATH' in os.environ:
    del os.environ['LD_LIBRARY_PATH']

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import h5py
import numpy as np

class CameraH5Publisher(Node):
    def __init__(self):
        super().__init__('camera_h5_publisher')
        
        # Setup Camera (Added CAP_V4L2 back to bypass GStreamer hang!)
        self.cap = cv2.VideoCapture(3, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video device /dev/video2")
            raise SystemExit
            
        ret, frame = self.cap.read()
        h, w, c = frame.shape

        # Setup ROS2 Publisher
        # 'camera/image_raw' is the standard ROS topic name for raw video
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Setup HDF5 File (Local Storage)
        desktop_path = os.path.expanduser("/home/ubuntu/Desktop")
        self.file_path = os.path.join(desktop_path, 'camera_data_ros.h5')
        self.h5_file = h5py.File(self.file_path, 'w')
        
        # Create expandable datasets
        self.img_dataset = self.h5_file.create_dataset('live_feed', shape=(1, h, w, c), maxshape=(None, h, w, c), dtype=np.uint8, chunks=True, compression="gzip")
        self.ts_dataset = self.h5_file.create_dataset('timestamps', shape=(1,), maxshape=(None,), dtype=np.float64)
        
        self.frame_idx = 0
        
        # Create a timer that triggers ~30 times a second (30 FPS)
        self.timer = self.create_timer(1.0 / 30.0, self.capture_and_publish)
        
        self.get_logger().info("Recording locally & publishing to ROS2...")
        self.get_logger().info("Press 'q' in the video window, or Ctrl+C in terminal to stop.")

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Dropped frame, retrying...")
            return
            
        # 2. GET UNIX TIME
        # Get raw system Unix time in nanoseconds to prevent floating-point precision loss
        unix_time_ns = time.time_ns()
        
        # Convert to float64 (seconds.nanoseconds) for your HDF5 file
        unix_time_sec_float = unix_time_ns / 1e9 

        # 3. SAVE LOCALLY TO HDF5
        if self.frame_idx > 0:
            self.img_dataset.resize((self.frame_idx + 1, frame.shape[0], frame.shape[1], frame.shape[2]))
            self.ts_dataset.resize((self.frame_idx + 1,))
        
        self.img_dataset[self.frame_idx] = frame
        self.ts_dataset[self.frame_idx] = unix_time_sec_float
        
        # 4. PUBLISH TO ROS2
        # Convert OpenCV numpy array to ROS2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Stamp the message directly with Unix time separated into sec and nanosec
        img_msg.header.stamp.sec = int(unix_time_ns // 1_000_000_000)
        img_msg.header.stamp.nanosec = int(unix_time_ns % 1_000_000_000)
        img_msg.header.frame_id = "usb_camera_link" 
        
        self.publisher_.publish(img_msg)
        
        self.frame_idx += 1

        # 5. DISPLAY 
        cv2.imshow('ROS2 Camera Publisher', frame)
        
        # If 'q' is pressed, trigger a KeyboardInterrupt to shut down cleanly
        if cv2.waitKey(1) & 0xFF == ord('q'):
            raise KeyboardInterrupt 

    def cleanup(self):
        # Always release hardware and close files safely
        self.cap.release()
        self.h5_file.close()
        cv2.destroyAllWindows()
        self.get_logger().info(f"Successfully saved {self.frame_idx} frames to {self.file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraH5Publisher()
    
    try:
        # spin() keeps the node running and triggering the timer loop
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()