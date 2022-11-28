import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere, Place
from plan_execute.plan_and_execute import PlanAndExecute
from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
# from pynput import keyboard
import math
import copy
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException


class Ready_State(Enum):
    PREPLACE = auto()


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
    ORIENT = auto(),
    ORIENT2 = auto(),
    PREGRAB = auto(),
    GRAB = auto(),
    PULL = auto(),
    SET = auto(),
    READY = auto(),
    CALIBRATE = auto(),
    RELEASE = auto(),
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
        super().__init__('cv_test')
        # Start timer
        self.freq = 100.
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        period = 1.0 / self.freq
        self.timer = self.create_timer(period, self.timer_callback, callback_group=self.cbgroup)
        self.movegroup = None
        self.go_here = self.create_service(GoHere, '/go_here', self.go_here_callback)
        self.cart_go_here = self.create_service(GoHere, '/cartesian_here', self.cart_callback)
        self.cal = self.create_service(Empty, '/calibrate', self.calibrate_callback)
        self.cal = self.create_service(Empty, '/ready', self.ready_callback)
        self.place = self.create_service(Place, '/place', self.place_callback)
        self.PlanEx = PlanAndExecute(self)
        self.prev_state = State.START
        self.state = State.START
        self.ct = 0
        self.goal_pose = Pose()
        self.block_pose = Pose()
        self.future = None
        self.pregrasp_pose = None

        self.piece_sub = self.create_subscription(Pose, 'jenga_piece', self.piece_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # added these so it won't rely on service calls to run
        self.start_pose = None
        self.execute = True
    
    def piece_cb(self, data):
        """
        Get the location of the jenga piece from cv nodes, and transform into panda_link0 frame.

        The location of the jenga piece in panda_link0 frame is then stored in self.goal_pose
        """
        self.get_logger().info(f'Piece location in camera frame:\n{data}')
        # TODO: Might need to wait a bit before looking up the transform.
        # Sometimes it is not there immediately.
        try:
            t = self.tf_buffer.lookup_transform('panda_link0', 'brick', rclpy.time.Time())
            self.get_logger().info(f'transform bw base and brick:\n{t}')
            self.get_logger().info('Go to orient')
            self.goal_pose.position.x = t.transform.translation.x
            self.goal_pose.position.y = t.transform.translation.y
            self.goal_pose.position.z = t.transform.translation.z
            self.goal_pose.orientation.x = t.transform.rotation.x
            self.goal_pose.orientation.y = t.transform.rotation.y
            self.goal_pose.orientation.z = t.transform.rotation.z
            self.goal_pose.orientation.w = t.transform.rotation.w
            self.get_logger().info(f'Goal Pose:\n{self.goal_pose}')
            self.state = State.ORIENT
        except TransformException:
            print("couldn't do panda_link0->brick transform")

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
        self.goal_pose = request.goal_pose
        self.execute = True
        self.start_pose = None
        self.state = State.ORIENT
        response.success = True
        return response

    def calibrate_callback(self, request, response):
        self.start_pose = None
        self.execute = True
        self.goal_pose = Pose()
        self.goal_pose.position.x = 0.55
        self.goal_pose.position.y = 0.0
        self.goal_pose.position.z = 0.5
        self.goal_pose.orientation.x = 0.7071068
        self.goal_pose.orientation.y = 0.0
        self.goal_pose.orientation.z = 0.7071068
        self.goal_pose.orientation.w = 0.0
        self.state = State.CALIBRATE
        return response

    def ready_callback(self, request, response):
        self.start_pose = None
        self.execute = True
        self.state = State.READY
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
        tower_pose.position.x = 0.46
        tower_pose.position.y = 0.0
        tower_pose.position.z = 0.2
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
                self.prev_state = State.START
                self.state = State.PLACEPLANE
                self.ct = 0
            else:
                self.ct += 1
        elif self.state == State.PLACEPLANE:
            self.state = State.IDLE
            await self.place_plane()
            await self.place_tower()
            self.prev_state = State.PLACEPLANE
            # await self.PlanEx.grab()
        elif self.state == State.CALL:
            self.future = await self.PlanEx.plan_to_pose(self.start_pose, self.goal_pose, 
                                                                0.001, None, self.execute)
            self.prev_state = State.CALL
        elif self.state == State.CALIBRATE:
            # self.state = State.IDLE
            joint_position = [1.2330863957058005, -1.0102056537740298, -1.0964429184557338, 
                              -2.4467336392631585, -2.661665911210206, 2.505597946846172,
                              2.6301953196046246]
            if self.ct == 0:
                self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                         self.goal_pose, joint_position, 0.001,
                                                         self.execute)
            self.prev_state = State.CALIBRATE
            self.state = State.IDLE
        elif self.state == State.ORIENT:
            self.get_logger().info('ORIENT STATE')
            orientation_pose = copy.deepcopy(self.goal_pose)
            # Turn to either +/- 45 degrees depending on piece position.
            orientation_pose.orientation.x = 0.9238795
            if self.goal_pose.position.y > 0:
                orientation_pose.orientation.y = -0.3826834
            else:
                orientation_pose.orientation.y = 0.3826834
            orientation_pose.orientation.z = 0.0
            orientation_pose.orientation.w = 0.0

            self.get_logger().info('PLAN')
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                orientation_pose, 0.02,
                                                                self.execute)
            self.get_logger().info('DONE')
            self.prev_state = State.ORIENT
            self.state = State.PREGRAB
        
        elif self.state == State.PREGRAB:
            # go to pre-grab pose
            offset = math.sin(math.pi/2) * 0.1
            pre_grasp = copy.deepcopy(self.goal_pose)
            pre_grasp.position.x = self.goal_pose.position.x - offset
            if self.goal_pose.position.y > 0:
                pre_grasp.position.y  = self.goal_pose.position.y + offset
            else:
                pre_grasp.position.y = self.goal_pose.position.y - offset
            self.pregrasp_pose = pre_grasp
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pre_grasp, 0.1,
                                                                   self.execute)
            self.prev_state = State.PREGRAB
            self.state = State.IDLE

        elif self.state == State.GRAB:
            # # go to grab pose
            self.get_logger().info('grabbing')
            self.get_logger().info(str(self.goal_pose))
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   self.goal_pose, 0.1,
                                                                   self.execute)
            # grab
            self.future = await self.PlanEx.grab()
            time.sleep(4) # maybe change to a counter rather than sleep 

            # go to pull pose
            self.prev_state = State.GRAB
            self.state = State.PULL
        elif self.state == State.PULL:
            self.prev_state = State.PLACEPLANE
            self.get_logger().info('pulling')
            # TODO: pull block out straight
            pull_pose = copy.deepcopy(self.pregrasp_pose)
            self.get_logger().info(str(pull_pose))
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pull_pose, 0.1,
                                                                   self.execute)
            self.prev_state = State.PULL
            self.get_logger().info(str(self.prev_state))
            self.state = State.READY
        elif self.state == State.READY:
            # TODO: go to ready pose
        
            ready_pose = Pose()
            ready_pose.position.x = 0.3060891
            ready_pose.position.y = 0.0
            ready_pose.position.z = 0.486882
            ready_pose.orientation.x = 1.0
            ready_pose.orientation.y = 0.0
            ready_pose.orientation.z = 0.0
            ready_pose.orientation.w = 0.0
            joint_position = [0.0, -0.7853981633974483, 0.0, 
                              -2.356194490192345, 0.0, 1.5707963267948966,
                              0.7853981633974483]
            # time.sleep(4)
            self.get_logger().info('\n\n\nReady')
            self.get_logger().info(str(self.prev_state))
            if self.prev_state == State.PULL:
                self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                    ready_pose, 0.1,
                                                                    self.execute)
                self.get_logger().info('ORIENTING')
                self.prev_state = State.READY
                self.state = State.ORIENT2
            else:
                self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                                    ready_pose, joint_position,
                                                                    0.001, self.execute)
                self.get_logger().info('IDLE')
                self.prev_state = State.READY
                self.state = State.IDLE

        elif self.state == State.ORIENT2:
            self.get_logger().info('ORIENT sencond')
            set_pose = copy.deepcopy(self.goal_pose)
            set_pose.orientation.x = 0.9238795
            set_pose.orientation.y = 0.3826834
            set_pose.orientation.z = 0.0
            set_pose.orientation.w = 0.0
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                set_pose, 0.02,
                                                                self.execute)
            self.prev_state = State.ORIENT2
            self.state = State.SET
        elif self.state == State.SET:
            set_pose = copy.deepcopy(self.goal_pose)
            set_pose.position.x = 0.474
            set_pose.position.y = -0.069
            set_pose.position.z = 0.205
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   set_pose, 0.1,
                                                                   self.execute)
            self.prev_state = State.SET            
            self.state = State.RELEASE
        elif self.state == State.RELEASE:
            self.future = await self.PlanEx.release()
            time.sleep(1)
            self.prev_state = State.RELEASE
            self.state = State.READY
        
        elif self.state == State.PLACEBLOCK:
            self.prev_state = State.PLACEBLOCK
            self.state = State.IDLE
            # place block
            await self.PlanEx.place_block(self.block_pose, [0.15, 0.05, 0.3], 'block')

def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()
