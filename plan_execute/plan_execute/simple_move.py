import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute.plan_and_execute import PlanAndExecute
from moveit_msgs.action import MoveGroup

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
        super().__init__('simple_move')
        # Start timer
        self.freq = 100.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)
        self.movegroup = None # Fill this in later lol

        self.PlanEx = PlanAndExecute(self)

    def timer_callback(self):
        self.get_logger().info("test")


def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()