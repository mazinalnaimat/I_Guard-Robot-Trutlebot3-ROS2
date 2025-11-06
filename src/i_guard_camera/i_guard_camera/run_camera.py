import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from picamera2 import Picamera2
import cv2
import numpy as np
import threading
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class CameraCompressedPublisher(Node):
    def __init__(self):
        super().__init__('picamera2_main_compressed_publisher')
        
        # Configure QoS for low latency.
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        # Publisher for compressed images.
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', qos_profile)
        
        # Configure Picamera2 for full sensor capture (main stream) at 1920x1280 in RGB888.
        self.picam2 = Picamera2()
        video_config = self.picam2.create_video_configuration(
            main={"size": (1920, 1280), "format": "RGB888"},
            controls={"FrameRate": 20}
        )
        self.picam2.configure(video_config)
        self.picam2.start()
        
        # Set up one thread that handles capture, processing, and publishing.
        self.running = True
        self.thread = threading.Thread(target=self.pipeline_loop, daemon=True)
        self.thread.start()
        
    def pipeline_loop(self):
        target_fps = 15  # target frame rate: 20 FPS
        frame_interval = 1.0 / target_fps
        
        while self.running:
            start_time = time.time()
            try:
                # Capture full sensor image from the main stream.
                frame = self.picam2.capture_array("main")
            except Exception as e:
                self.get_logger().error(f"Frame capture error: {e}")
                continue
                
            try:
                # Resize full captured frame to 1280x720 (scaling, no cropping)
                resized_frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
                # Compress the resized frame using JPEG compression.
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 25]  # Adjust quality as needed.
                ret, buffer = cv2.imencode('.jpg', resized_frame, encode_param)
                if not ret:
                    self.get_logger().error("JPEG compression failed.")
                    continue

                # Create and fill the CompressedImage message.
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = np.array(buffer).tobytes()
                
                # Publish the compressed message.
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Processing/publishing error: {e}")
            
            # Sleep to achieve the target frame rate.
            elapsed = time.time() - start_time
            sleep_duration = frame_interval - elapsed
            if sleep_duration > 0:
                time.sleep(sleep_duration)
                
    def destroy_node(self):
        self.running = False
        self.thread.join()
        self.picam2.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCompressedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
