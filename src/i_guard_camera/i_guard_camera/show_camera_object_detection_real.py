import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from ultralytics import YOLO

class CameraYoloViewer(Node):
    def __init__(self):
        super().__init__('camera_yolo_viewer')

        # QoS for camera stream
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            qos_profile
        )

        #self.model = YOLO('yolov8n.pt')
        #self.model = YOLO('yolov8s.pt')
        #self.model = YOLO('yolov8m.pt')
        #self.model = YOLO('yolov8l.pt')
        self.model = YOLO('yolov8x.pt')
   
        #self.model = YOLO('yolov8n-seg.pt')

        #self.model = YOLO('yolov8n-cls.pt')

        #self.model = YOLO('yolov8n-pose.pt')

  

        # Setup OpenCV window
        self.window_name = "Camera Detection"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

        self.get_logger().info("YOLO model loaded and subscribed to /image_raw/compressed")

    def listener_callback(self, msg):
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image_np is None:
                self.get_logger().warn("Failed to decode image")
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
