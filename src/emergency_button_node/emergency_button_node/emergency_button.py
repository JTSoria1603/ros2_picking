import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class EmergencyButtonNode(Node):
    """
    ROS2 node that simulates an emergency button.

    Publishes the emergency button state to the 'emergency_button' topic every 0.5 seconds, and provides
    services to press, release, and get the current state of the emergency button.
    """
    def __init__(self):
        """
        Initialize the EmergencyButtonNode, setting up publisher, services, timer, and initial state.
        """
        super().__init__('emergency_button_node')
        self.pub = self.create_publisher(Bool, 'emergency_button', 10)
        self.state = False 
        self.timer = self.create_timer(0.5, self.publish_state)
        self.get_logger().info('Emergency Button Node has been started')
        self.press_srv   = self.create_service(Trigger, 'press_emergency_button',   self.handle_press)
        self.release_srv = self.create_service(Trigger, 'release_emergency_button', self.handle_release)
        self.get_emergency_button_status = self.create_service(Trigger, 'get_emergency_button_state', self.handle_get_emergency_button_status)

    def publish_state(self):
        """
        Publish the current emergency button state to the 'emergency_button' topic.
        """
        msg = Bool(data=self.state)
        self.pub.publish(msg)
        self.get_logger().info(f'Published emergency_button: {self.state}')

    def handle_get_emergency_button_status(self, response):
        """
        Handle the 'get_emergency_button_state' service call, returning the current emergency button state.

        Args:
            response: The service response object to populate.

        Returns:
            The populated response with the current emergency button state and success status.
        """
        response.success = True
        response.message = str(self.state)
        self.get_logger().info(f'Current emergency button state: {self.state}')
        return response

    def handle_press(self, response):
        """
        Handle the 'press_emergency_button' service call, setting the emergency button state to pressed.

        Args:
            response: The service response object to populate.

        Returns:
            The populated response with the result of the press action and success status.
        """
        if not self.state:
            self.state = True
            response.success = True
            response.message = 'Emergency button PRESSED'
            self.get_logger().info('Emergency button pressed → robot must stop')
        else:
            response.success = False
            response.message = 'Already PRESSED'
        return response

    def handle_release(self, response):
        """
        Handle the 'release_emergency_button' service call, setting the emergency button state to released.

        Args:
            response: The service response object to populate.

        Returns:
            The populated response with the result of the release action and success status.
        """
        if self.state:
            self.state = False
            response.success = True
            response.message = 'Emergency button RELEASED'
            self.get_logger().info('Emergency button released → robot may resume')
        else:
            response.success = False
            response.message = 'Already RELEASED'
        return response


def main(args=None):
    """
    Entry point for the emergency_button_node. Initializes ROS, starts the node, and handles shutdown.
    """
    rclpy.init(args=args)
    node = EmergencyButtonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
