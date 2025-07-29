import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DoorHandleNode(Node):
    """
    ROS2 node that simulates a door handle sensor.

    Publishes the door's closed state to the 'door_closed' topic every second, and provides services
    to toggle the door state to simulate that the door was open('toggle_door') and to get the current door state ('get_door_state').
    """
    def __init__(self):
        """
        Initialize the DoorHandleNode, setting up publisher, services, timer, and initial state.
        """
        super().__init__('door_handle_node')
        self.pub = self.create_publisher(Bool, 'door_closed', 10)
        self.state = True
        self.timer = self.create_timer(1.0, self.publish_state)
        self.get_logger().info('Door Handle Node has been started.')
        self.srv = self.create_service(Trigger, 'toggle_door', self.handle_toggle)
        self.srv_get = self.create_service(Trigger, 'get_door_state', self.get_door_state)

    def publish_state(self):
        """
        Publish the current door state to the 'door_closed' topic.
        """
        msg = Bool(data=self.state)
        self.pub.publish(msg)
        self.get_logger().info(f'Published door_closed: {self.state}')

    def handle_toggle(self, request, response):
        """
        Handle the 'toggle_door' service call, toggling the door state and returning the new state.

        Args:
            request: The service request (unused).
            response: The service response object to populate.

        Returns:
            The populated response with the new door state and success status.
        """
        self.state = not self.state
        response.success = True
        response.message = f'door_closed toggled to {self.state}'
        self.get_logger().info(f'Door state toggled: {self.state}')
        return response

    def get_door_state(self, request, response):
        """
        Handle the 'get_door_state' service call, returning the current door state.

        Args:
            request: The service request (unused).
            response: The service response object to populate.

        Returns:
            The populated response with the current door state and success status.
        """
        response.success = True
        response.message = str(self.state)
        self.get_logger().info(f'Current door state: {self.state}')
        return response


def main(args=None):
    """
    Entry point for the door_handle_node. Initializes ROS, starts the node, and handles shutdown.
    """
    rclpy.init(args=args)
    node = DoorHandleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
