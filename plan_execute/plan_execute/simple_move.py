import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere
from plan_execute.plan_and_execute import PlanAndExecute
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Point, Quaternion, Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """
    START = auto(),
    IDLE = auto(),
    CALL = auto()


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
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(1./self.freq, self.timer_callback, callback_group=self.cbgroup)
        self.movegroup = None # Fill this in later lol
        self.go_here = self.create_service(GoHere, 'go_here', self.go_here_callback)
        self.PlanEx = PlanAndExecute(self)

        self.state = State.START
        self.ct = 0
        self.goal_pose = Pose()
        self.future = None

    def go_here_callback(self, request, response):
        self.goal_pose = request.goal_pose
        self.execute = request.execute
        print(self.goal_pose)
        print(self.execute)
        return response

    async def timer_callback(self):
        if self.state == State.START:
            # add a bit of a time buffer so js can be read in
            if self.ct==100:
                self.state = State.CALL
            else:
                self.ct += 1
        if self.state == State.CALL: 
            self.state = State.IDLE
            start = Point(x=1.0, y=1.0, z=1.0)
            endpos = Point(x=0.5, y=0.5, z=0.5)
            endori = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
            self.future = await self.PlanEx.plan_to_pose(start, Pose(position=endpos, orientation=endori), True)
            print(type(self.future))
            print("MAIN LOOP:", self.future)
        # self.get_logger().info("test")


def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()