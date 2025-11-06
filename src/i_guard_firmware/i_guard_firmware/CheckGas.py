#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice

class GasDetector(Node):
    def __init__(self):
        super().__init__('gas_detector')
        self.pin = DigitalInputDevice(18, pull_up=False)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Gas detector started on GPIO18")

    def timer_callback(self):
        if self.pin.value == 0:
            self.get_logger().info("no gas")
        else:
            self.get_logger().info("there is gas")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GasDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
