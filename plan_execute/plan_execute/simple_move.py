import rclpy
from rclpy.node import Node
from enum import Enum, auto

class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """
    WAIT = auto()
    FORWARD = auto()
    BACKWARD = auto()


class Test(Node):
    """
    Keep track of enviroment components and do calculations.

    This node publishes to the visualization_marker, text_marker, goal_pose, move_robot,
    and tilt. it also has parammeters for gravity and velocity.
    """

    def __init__(self):
        super().__init__('flip')
        # Start timer
        self.timer = self.create_timer(.01, self.timer_callback)


    def timer_callback(self):
        self.get_logger().info("test")


def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()