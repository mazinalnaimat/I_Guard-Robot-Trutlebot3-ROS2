import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        # Define QoS profile (best effort for camera stream)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            qos_profile)

        # Create a named window and set its size
        self.window_name = "Camera View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image_np is not None:
                # Resize the image while keeping aspect ratio inside 800x600
                h, w = image_np.shape[:2]
                scale = min(800 / w, 600 / h)
                resized_image = cv2.resize(image_np, (int(w * scale), int(h * scale)))

                cv2.imshow(self.window_name, resized_image)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("Failed to decode image")
        except Exception as e:
            self.get_logger().error(f"Exception: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    camera_viewer = CameraViewer()

    try:
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        camera_viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

