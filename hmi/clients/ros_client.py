from rclpy.node import Node
from std_srvs.srv import Trigger
import rclpy

class ROSClient(Node):
    def __init__(self):
        super().__init__('ros_bridge_hmi')
        self.cli_door = self.create_client(Trigger, 'get_door_state')
        self.cli_emergency_button = self.create_client(Trigger, 'get_emergency_button_state')
        self.cli_stack_light = self.create_client(Trigger, 'get_stack_light_state')

        while not self.cli_stack_light.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stack_light service...')
        while not self.cli_door.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for door service...')
        while not self.cli_emergency_button.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency service...')

    def is_door_closed(self):
        req = Trigger.Request()
        future = self.cli_door.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message.lower() == "true"

    def is_emergency_pressed(self):
        req = Trigger.Request()
        future = self.cli_emergency_button.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message.lower() == "true"

    def get_stack_light(self):
        req = Trigger.Request()
        future = self.cli_stack_light.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message
