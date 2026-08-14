import cv2
import time
import rclpy
import multiprocessing as mp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from supercamera import Camera
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class EndoscopePublisher(Node):
    def __init__(self):
        super().__init__('endoscope_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(CompressedImage, 'endoscope/compressed', qos_profile) 
        
        # Use a multiprocessing Queue instead of a threading Queue
        self.frame_queue = mp.Queue(maxsize=1)
        self.stop_event = mp.Event()
        
        # Move the hardware capture to a completely separate CPU process
        self.capture_process = mp.Process(
            target=self.hardware_capture_loop, 
            args=(self.frame_queue, self.stop_event)
        )
        self.capture_process.start()

        # We can keep the publish loop as a ROS timer to keep it in the main ROS thread
        # This checks the queue for a new frame 60 times a second
        self.timer = self.create_timer(1.0 / 60.0, self.publish_loop)

    @staticmethod
    def hardware_capture_loop(queue, stop_event):
        """
        This runs in a completely separate CPU core. 
        It initializes the camera here so the C++ memory isn't shared across processes.
        """
        with Camera() as cam:
            while not stop_event.is_set():
                ret, frame = cam.read() 
                if ret:
                    capture_time = time.time_ns()
                    if queue.full():
                        try:
                            queue.get_nowait()
                        except Exception:
                            pass
                    queue.put((capture_time, frame))

    def publish_loop(self):
        try:
            # Check if a fresh frame is ready from the other process
            capture_time, frame = self.frame_queue.get_nowait()
        except Exception:
            return # No new frame, just return and wait for the next timer tick
            
        _, compressed_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        msg = CompressedImage() 
        msg.header.stamp.sec = int(capture_time // 1_000_000_000)
        msg.header.stamp.nanosec = int(capture_time % 1_000_000_000)
        msg.header.frame_id = "usb_camera_link" 
        msg.format = "jpeg"
        msg.data = compressed_data.tobytes()             
        self.pub.publish(msg)

    def cleanup(self):
        self.stop_event.set()
        self.capture_process.join()

def main(args=None):
    rclpy.init(args=args)
    node = EndoscopePublisher()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()