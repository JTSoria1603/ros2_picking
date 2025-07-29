import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RosClient(Node):
    def __init__(self):
        super().__init__('ros_client')
        
        self.get_logger().info('ROS Client Node has been started.')
        
        self.cli_barcode = self.create_client(Trigger, 'get_barcode')
        while not self.cli_barcode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_barcode service...')
        

        self.cli_stack_light = self.create_client(Trigger, 'get_stack_light_state')
        while not self.cli_stack_light.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stack_light service...')


    def get_barcode(self):
        req = Trigger.Request()
        future = self.cli_barcode.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message
    
    def get_stack_light(self):
        req = Trigger.Request()
        future = self.cli_stack_light.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().message

