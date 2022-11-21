import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere, Place
from plan_execute.plan_and_execute import PlanAndExecute
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import math


class State(Enum):
    """
    Current state of the system.

    Determines what the main timer function should be doing on each iteration
    """

    START = auto(),
    IDLE = auto(),
    CALL = auto(),
    PLACEBLOCK = auto(),
    PLACEPLANE = auto(),
    CARTESIAN = auto(),
    GRAB = auto(),
    PULL = auto(),
    SET = auto(),
    READY = auto(),
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

class Test(Node):
    """
    Control the robot scene.

    Calls the /place and the /go_here services to plan or execute a robot movement path
    and to place a block in the scene.
    """

    def __init__(self):
        """Create callbacks, initialize variables, start timer."""
        super().__init__('simple_move')
        # Start timer
        self.freq = 100.
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.timer_callback, callback_group=self.cbgroup)
        self.movegroup = None
        self.go_here = self.create_service(GoHere, '/go_here', self.go_here_callback)
        self.cart_go_here = self.create_service(GoHere, '/cartesian_here', self.cart_callback)
        self.place = self.create_service(Place, '/place', self.place_callback)
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
    
    def cart_callback(self, request, response):
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
            self.state = State.GRAB
            response.success = True
        elif pose_len == 1:
            self.start_pose = self.start_pose[0]
            self.state = State.GRAB
            response.success = True
            self.execute = False
        else:
            self.get_logger().info('Enter either zero or one initial poses.')
            self.state = State.IDLE
            response.success = False
        return response

    def place_callback(self, request, response):
        """Call service to pass the desired Pose of a block in the scene."""
        self.block_pose = request.place
        self.state = State.PLACEBLOCK
        return response
    
    async def place_plane(self):
        plane_pose = Pose()
        plane_pose.position.x = 0.0
        plane_pose.position.y = 0.0
        plane_pose.position.z = -0.14
        plane_pose.orientation.x = 0.0
        plane_pose.orientation.y = 0.0
        plane_pose.orientation.z = 0.0
        plane_pose.orientation.w = 1.0
        await self.PlanEx.place_block(plane_pose, [10.0, 10.0, 0.1], 'plane')
    
    async def place_tower(self):
        tower_pose = Pose()
        tower_pose.position.x = 0.6
        tower_pose.position.y = 0.0
        tower_pose.position.z = 0.09
        tower_pose.orientation.x = 0.9226898
        tower_pose.orientation.y = 0.3855431
        tower_pose.orientation.z = 0.0
        tower_pose.orientation.w = 0.0
        await self.PlanEx.place_block(tower_pose, [0.15, 0.15, 0.18], 'tower')

    async def timer_callback(self):
        """State maching that dictates which functions from the class are being called."""
        if self.state == State.START:
            # add a bit of a time buffer so js can be read in
            if self.ct == 100:
                self.state = State.PLACEPLANE
            else:
                self.ct += 1
        if self.state == State.PLACEPLANE:
            self.state = State.IDLE
            await self.place_plane()
            await self.place_tower()
            # await self.PlanEx.grab()
        if self.state == State.CALL:
            self.state = State.IDLE
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                self.goal_pose,
                                                                self.execute)
        if self.state == State.CARTESIAN:
            self.state = State.IDLE
            # self.future = await self.PlanEx.plan_to_pose(self.start_pose,
            #                                              self.goal_pose,
            #                                              self.execute)
            # self.future = await self.PlanEx.plan_to_position(self.start_pose,
            #                                                  self.goal_pose,
            #                                                  self.execute)
            # self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
            #                                                     self.goal_pose,
            #                                                     self.execute)
            offset = math.sin(math.pi/2) * 0.1
            pre_grasp = self.goal_pose
            pre_grasp.position.x = self.goal_pose.position.x - offset
            if self.goal_pose.position.y > 0:
                pre_grasp.position.y  = self.goal_pose.position.y + offset
            else:
                pre_grasp.position.y = self.goal_pose.position.y - offset
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pre_grasp,
                                                                   self.execute)
            # await self.PlanEx.grab()
        if self.state == State.GRAB:
            # TODO: if y > 0, do something, else do something else
            orientation_pose = self.goal_pose
            orientation_pose.orientation.x = 0.9238795
            if self.goal_pose.position.y > 0:
                orientation_pose.orientation.y = -0.3826834
            else:
                orientation_pose.orientation.y = 0.3826834
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                self.goal_pose,
                                                                self.execute)
            # go to pre-grab pose
            offset = math.sin(math.pi/2) * 0.1
            pre_grasp = self.goal_pose
            pre_grasp.position.x = self.goal_pose.position.x - offset
            if self.goal_pose.position.y > 0:
                pre_grasp.position.y  = self.goal_pose.position.y + offset
            else:
                pre_grasp.position.y = self.goal_pose.position.y - offset
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pre_grasp,
                                                                   self.execute)
            # # go to grab pose
            # self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
            #                                                        grab_pose,
            #                                                        self.execute)
            # grab
            # await self.PlanEx.grab()
            # go to pull pose
            self.state = State.PULL
        if self.state == State.PULL:
            # TODO: pull block out straight
            self.state = State.READY
        if self.state == State.READY:
            # TODO: go to ready pose
            self.state = State.IDLE
        if self.state == State.PLACEBLOCK:
            self.state = State.IDLE
            # place block
            await self.PlanEx.place_block(self.block_pose, [0.15, 0.05, 0.03], 'block')

def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()
