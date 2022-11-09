import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere, Place
from plan_execute.plan_and_execute import PlanAndExecute
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    START = auto(),
    IDLE = auto(),
    CALL = auto(),
    PLACE = auto()


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
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.timer_callback, callback_group=self.cbgroup)
        self.movegroup = None
        self.go_here = self.create_service(GoHere, 'go_here', self.go_here_callback)
        self.place = self.create_service(Place, 'place', self.place_callback)
        self.PlanEx = PlanAndExecute(self)
        self.state = State.START
        self.ct = 0
        self.goal_pose = Pose()
        self.block_pose = Pose()
        self.future = None

    def go_here_callback(self, request, response):
        self.start_pose = request.start_pose
        self.goal_pose = request.goal_pose
        self.execute = request.execute
        pose_len = len(self.start_pose)
        if pose_len == 0:
            self.start_pose = None
            self.state = State.CALL
            response.success = True
        elif pose_len == 1:
            self.start_pose = self.start_pose[0]
            self.state = State.CALL
            response.success = True
            self.execute = False
        else:
            self.get_logger().info('Enter either zero or one initial poses.')
            self.state = State.IDLE
            response.success = False
        return response

    def place_callback(self, request, response):
        self.block_pose = request.place
        self.state = State.PLACE
        return response

    async def timer_callback(self):
        if self.state == State.START:
            # add a bit of a time buffer so js can be read in
            if self.ct == 100:
                self.state = State.IDLE
            else:
                self.ct += 1
        if self.state == State.CALL:
            self.state = State.IDLE
            self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                         self.goal_pose,
                                                         self.execute)
            # self.future = await self.PlanEx.plan_to_position(self.start_pose,
            #                                                  self.goal_pose,
            #                                                  self.execute)
            # self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
            #                                                     self.goal_pose,
            #                                                     self.execute)
        if self.state == State.PLACE:
            self.state = State.IDLE
            await self.PlanEx.place_block(self.block_pose)


def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()
