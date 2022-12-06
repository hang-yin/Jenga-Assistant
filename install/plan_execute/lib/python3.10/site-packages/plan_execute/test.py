import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere, Place
from plan_execute.plan_and_execute import PlanAndExecute
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math
import copy


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    START = auto(),
    IDLE = auto(),
    CALL = auto(),
    # ready 
        # sends pose back to ready default position of robot after any movement or motion
    # calibrate
        # send calibrate position to plane pose function make sure it goes from ready position to the 
        # calibrate position
    # grab 
        # orients the gripper to the correct orientation 
            # if end y position < 0 then rotate y = 0.3826834
            # if end y position > 0 then rotate y = -0.3826834 
            # or if vision is really good get the gripper the orientation from the block
        # then sends the pre gripping position
            # 0.08 distance from the edge of the brick
            # if vision for orientation, find the x y coords using the the angle of the block in euler and use sin cos)
            # if hard code we expect 45 deg: abs(x) = abs(y) for coords
        # then cartesian path to gripping position 
            # then cartesian path into the grab spot of the block 
        # grasp the block
    # pull
        # use cartesian to pull the block in a straght line back following the orientation of the tower
        # if hard code we expect 45 deg
        # if vision for orientation, find the x y coords using the the angle of the block in euler and use sin cos)
        # then return to ready
    # set

class Test1(Node):
    """
    Control the robot scene.

    Calls the /place and the /go_here services to plan or execute a robot movement path
    and to place a block in the scene.
    """

    def __init__(self):
        """Create callbacks, initialize variables, start timer."""
        super().__init__('test')
        # Start timer
        self.freq = 100.
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.timer_callback, callback_group=self.cbgroup)
        self.movegroup = None
        self.oreint_here = self.create_service(GoHere, '/orient_here', self.go_here_callback)
        self.PlanEx = PlanAndExecute(self)
        self.state = State.START
        self.ct = 0
        self.goal_pose = Pose()
        self.block_pose = Pose()
        self.future = None

    def go_here_callback(self, request, response):
        """
        Call a custom service that takes one Pose of variable length, a regular Pose, and a bool.

        The user can pass a custom start postion to the service and a desired end goal. The boolean
        indicates whether to plan or execute the path.
        """
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


    async def timer_callback(self):
        """State maching that dictates which functions from the class are being called."""
        if self.state == State.START:
            # add a bit of a time buffer so js can be read in
            if self.ct == 100:
                self.state = State.IDLE
            else:
                self.ct += 1
            # await self.PlanEx.grab()
        elif self.state == State.CALL:
            self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                                self.goal_pose,
                                                                self.execute)
            self.state = State.IDLE
def test1_entry(args=None):
    rclpy.init(args=args)
    node = Test1()
    rclpy.spin(node)
    rclpy.shutdown()
