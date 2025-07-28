import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Trigger


class StackLightNode(Node):
    """
    ROS2 node that simulates a stack light indicator.

    Subscribes to door and emergency button states, determines the stack light state, and publishes it to the 'stack_light' topic.
    Provides a service to get the current stack light state.
    """
    def __init__(self):
        """
        Initialize the StackLightNode, setting up publisher, subscriptions, timer, and initial states.
        """
        super().__init__('stack_light_node')
        self.pub = self.create_publisher(Int8, 'stack_light', 10)
        self.door_closed = True
        self.emergency_pressed = False
        self.state = 0
        self.create_subscription(Bool, 'door_closed', self.cb_door, 10)
        self.create_subscription(Bool, 'emergency_button', self.cb_emergency, 10)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info('Stack Light Node has been started.')
        self.srv_get = self.create_service(Trigger, 'get_stack_light_state', self.get_stack_light_state)

    def get_stack_light_state(self, response):
        """
        Handle the 'get_stack_light_state' service call, returning the current stack light state.

        Args:
            response: The service response object to populate.

        Returns:
            The populated response with the current stack light state and success status.
        """
        response.success = True
        response.message = str(self.state)
        self.get_logger().info(f'Current stack light state: {self.state}')
        return response

    def cb_door(self, msg: Bool):
        """
        Callback for the 'door_closed' topic. Updates the door state.

        Args:
            msg (Bool): Message containing the door closed state.
        """
        self.door_closed = msg.data
        self.get_logger().debug(f'Door closed={self.door_closed}')

    def cb_emergency(self, msg: Bool):
        """
        Callback for the 'emergency_button' topic. Updates the emergency button state.

        Args:
            msg (Bool): Message containing the emergency button pressed state.
        """
        self.emergency_pressed = msg.data
        self.get_logger().debug(f'Emergency-Button pressed={self.emergency_pressed}')

    def publish_state(self):
        """
        Determine and publish the current stack light state based on door and emergency button states.
        -1: Emergency pressed
         1: Door open
         0: Operative (door closed, no emergency)
        """
        if self.emergency_pressed:
            self.state = -1
        elif not self.door_closed:
            self.state = 1
        else:
            self.state = 0

        msg = Int8(data=self.state)
        self.pub.publish(msg)
        self.get_logger().info(f'Published stack_light: {self.state}')


def main(args=None):
    """
    Entry point for the stack_light_node. Initializes ROS, starts the node, and handles shutdown.
    """
    rclpy.init(args=args)
    node = StackLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
