#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

class DoorHandleNode(Node):
    def __init__(self):
        super().__init__('door_handle_node')
        self.pub = self.create_publisher(Bool, 'door_closed', 10)
        self.state = True
        self.timer = self.create_timer(1.0, self.publish_state)
        self.get_logger().info('Door Handle Node has been started.')
        self.srv = self.create_service(Trigger, 'toggle_door', self.handle_toggle)

    def publish_state(self):
        msg = Bool(data=self.state)
        self.pub.publish(msg)
        self.get_logger().info(f'Published door_closed: {self.state}')

    def handle_toggle(self, request, response):
        self.state = not self.state
        response.success = True
        response.message = f'door_closed toggled to {self.state}'
        self.get_logger().info(f'Door state toggled: {self.state}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoorHandleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
