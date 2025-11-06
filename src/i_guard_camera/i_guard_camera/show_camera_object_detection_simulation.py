import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class CameraYoloViewer(Node):
    def __init__(self):
        super().__init__('camera_yolo_viewer')

        # QoS for simulation camera
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/world/default/model/i_guard/link/base_footprint/sensor/camera_sensor/image',
            self.listener_callback,
            qos_profile
        )

        self.model = YOLO('yolov8x.pt')

        self.bridge = CvBridge()

        # Setup OpenCV window
        self.window_name = "Camera Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

        self.get_logger().info("YOLO model loaded and subscribed to simulation image topic")

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if image_np is None:
                self.get_logger().warn("Failed to convert image")
                return

            # Run YOLO detection
            results = self.model(image_np)
            result = results[0]
            annotated_image = result.plot()

            # Resize keeping aspect ratio inside 800x600
            h, w = annotated_image.shape[:2]
            scale = min(800 / w, 600 / h)
            resized = cv2.resize(annotated_image, (int(w * scale), int(h * scale)))

            # Show annotated image
            cv2.imshow(self.window_name, resized)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Exception: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
