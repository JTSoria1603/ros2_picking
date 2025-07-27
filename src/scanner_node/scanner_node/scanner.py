#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import random

class ScannerNode(Node):
    def __init__(self):
        super().__init__('scanner_node')
        self.pub = self.create_publisher(String, 'barcode', 10)
        self.srv = self.create_service(Trigger, 'get_barcode', self.handle_get_barcode)
        self.timer = self.create_timer(1.0, self.publish_barcode)
        self.current_barcode = ''

    def publish_barcode(self):
        code = f"{random.randint(0, 99999):05d}"
        self.current_barcode = code
        self.pub.publish(String(data=code))
        self.get_logger().info(f'Published barcode: {code}')

    def handle_get_barcode(self, request, response):
        response.success = True
        response.message = self.current_barcode
        self.get_logger().info(f'Service call: returning {self.current_barcode}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
