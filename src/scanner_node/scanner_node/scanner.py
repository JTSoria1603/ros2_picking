import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import random


class ScannerNode(Node):
    """
    ROS2 node that simulates a barcode scanner.

    Publishes random barcode strings to the 'barcode' topic every second and provides a service
    'get_barcode' to retrieve the most recently published barcode.
    """
    def __init__(self):
        """
        Initialize the ScannerNode, setting up the publisher, service, timer, and barcode state.
        """
        super().__init__('scanner_node')
        self.pub = self.create_publisher(String, 'barcode', 10)
        self.srv = self.create_service(Trigger, 'get_barcode', self.handle_get_barcode)
        self.timer = self.create_timer(1.0, self.publish_barcode)
        self.current_barcode = ''

    def publish_barcode(self):
        """
        Generate a random 5-digit barcode, publish it to the 'barcode' topic, and store it as the current barcode.
        """
        code = f"{random.randint(0, 99999):05d}"
        self.current_barcode = code
        self.pub.publish(String(data=code))
        self.get_logger().info(f'Published barcode: {code}')

    def handle_get_barcode(self, response):
        """
        Handle the 'get_barcode' service call, returning the most recently published barcode.

        Args:
            response: The service response object to populate.

        Returns:
            The populated response with the latest barcode and success status.
        """
        response.success = True
        response.message = self.current_barcode
        self.get_logger().info(f'Service call: returning {self.current_barcode}')
        return response


def main(args=None):
    """
    Entry point for the scanner_node. Initializes ROS, starts the node, and handles shutdown.
    """
    rclpy.init(args=args)
    node = ScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
