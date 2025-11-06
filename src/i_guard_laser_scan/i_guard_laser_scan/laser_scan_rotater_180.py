#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanRotator(Node):
    def __init__(self):
        super().__init__('laser_scan_rotator')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(LaserScan, '/rotated_scan', 10)
    
    def listener_callback(self, msg):
        rotated_scan = LaserScan()
        rotated_scan.header = msg.header
        
        # Rotate the ranges by 180 degrees (swap front and back)
        rotated_scan.ranges = np.roll(msg.ranges, len(msg.ranges) // 2).tolist()
        
        # Keep angles the same (since we rotated the data, not the frame)
        rotated_scan.angle_min = msg.angle_min
        rotated_scan.angle_max = msg.angle_max
        
        # Copy other parameters
        rotated_scan.angle_increment = msg.angle_increment
        rotated_scan.time_increment = msg.time_increment
        rotated_scan.scan_time = msg.scan_time
        rotated_scan.range_min = msg.range_min
        rotated_scan.range_max = msg.range_max
        
        # Optionally rotate intensities if needed
        if len(msg.intensities) > 0:
            rotated_scan.intensities = np.roll(msg.intensities, len(msg.intensities) // 2).tolist()
        else:
            rotated_scan.intensities = []
        
        self.publisher.publish(rotated_scan)
        self.get_logger().info('Publishing rotated laser scan')

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()