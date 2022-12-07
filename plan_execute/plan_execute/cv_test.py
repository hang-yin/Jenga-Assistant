import rclpy
from rclpy.node import Node
from enum import Enum, auto
from plan_execute_interface.srv import GoHere, Place
from plan_execute.plan_and_execute import PlanAndExecute
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int16
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Empty
# from pynput import keyboard
import math
import copy
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from math import sqrt


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
    CLOSE = auto(),
    PULL = auto(),
    POSTPULL = auto(),
    REMOVETOWER = auto(),
    SET = auto(),
    READY = auto(),
    CALIBRATE = auto(),
    RELEASE = auto(),
    FINDPIECE = auto(),
    FINDTOP = auto(),
    PREPUSH = auto(),
    PREPUSHFINGER = auto(),
    PUSH = auto(),
    POSTPUSH = auto(),
    # states for poking!!
    PREPICKUP = auto(),
    PICKUP = auto(),
    LIFT = auto(),
    ORIENT3 = auto(),
    PREPOKE = auto(),
    POKE = auto(),
    POSTPOKE = auto(),
    ORIENT4 = auto(),
    PLACEPOKER = auto(),
    POSTPLACEPOKER = auto(),
    LETGO = auto(),
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
        self.jenga = self.create_service(GoHere, '/jenga_time', self.jenga_callback)
        self.poke = self.create_service(GoHere, '/poke', self.poke_callback)
        self.cal = self.create_service(Empty, '/calibrate', self.calibrate_callback)
        self.cal = self.create_service(Empty, '/ready', self.ready_callback)
        self.cal = self.create_service(Empty, '/release', self.release_callback)
        self.place = self.create_service(Place, '/place', self.place_callback)
        self.PlanEx = PlanAndExecute(self)
        self.prev_state = State.START
        self.state = State.START
        self.ct = 0
        self.goal_pose = Pose()
        self.block_pose = Pose()
        self.future = None
        self.pregrasp_pose = None

        self.place_pose = Pose()
        self.place_pose.position.x = 0.474
        self.place_pose.position.y = -0.069
        self.place_pose.position.z = 0.380
        self.poke_pose = Pose()

        self.piece_sub = self.create_subscription(Pose, 'jenga_piece', self.piece_cb, 10)
        self.top_sub = self.create_subscription(Int16, 'top_size', self.top_cb, 10)
        self.top_ori_sub = self.create_subscription(Int16, 'top_ori', self.top_ori_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.piece_found_pub = self.create_publisher(Bool, 'piece_found', 10)
        self.motion_complete_pub = self.create_publisher(Bool, 'finished_place', 10)
        self.layer_added_pub = self.create_publisher(Bool, 'layer_added', 10)
        self.tower_top_pose = Pose()
        # added these so it won't rely on service calls to run
        self.start_pose = None
        self.execute = True
        # Pieces are 5cm wide
        self.piece_width = 0.05
        self.piece_height = 0.03
        self.top_positions = None
        self.place_locations = None
        self.place_counter = 0
        self.top_ori = None


    def piece_cb(self, data):
        """
        Get the location of the jenga piece from cv nodes, and transform into panda_link0 frame.

        The location of the jenga piece in panda_link0 frame is then stored in self.goal_pose
        """
        self.get_logger().info(f'Piece location in camera frame:\n{data}')
        self.state = State.FINDPIECE
    
    def top_cb(self, data):
        """
        Get the location of the jenga piece from cv nodes, and transform into panda_link0 frame.

        The location of the jenga piece in panda_link0 frame is then stored in self.goal_pose
        """
        self.get_logger().info(f'NUMBER OF PIECES ON TOP:\n{data}')
        self.state = State.FINDTOP

    def top_ori_cb(self, data):
        self.get_logger().info(f'ORIENTATION OF TOP:\n{data}')
        self.top_ori = data

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
        self.state = State.CARTESIAN
        response.success = True
        return response

    def jenga_callback(self, request, response):
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

    def poke_callback(self, request, response):
        """
        Call a custom service that takes one Pose of variable length, a regular Pose, and a bool.

        The user can pass a custom start postion to the service and a desired end goal. The boolean
        indicates whether to plan or execute the path.
        """
        self.poke_pose = request.goal_pose
        self.execute = True
        self.start_pose = None
        self.state = State.PREPICKUP
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

    def release_callback(self, request, response):
        self.get_logger().info("RELEASE THE GRIPPERS")
        self.state = State.LETGO
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
        # TODO update with tower location from tfs
        tower_pose = Pose()
        tower_pose.position.x = 0.46
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
                self.prev_state = State.START
                self.state = State.PLACEPLANE
                self.ct = 0
            else:
                self.ct += 1
        elif self.state == State.PLACEPLANE:
            self.state = State.IDLE
            await self.place_plane()
            # await self.place_tower()
            self.prev_state = State.PLACEPLANE
            # await self.PlanEx.grab()
        elif self.state == State.CALL:
            self.future = await self.PlanEx.plan_to_pose(self.start_pose, self.goal_pose, 
                                                                None, 0.001, self.execute)
            self.prev_state = State.CALL
            self.state = State.IDLE
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

        elif self.state == State.CARTESIAN:
            self.state = State.IDLE
            offset = math.sin(math.pi/2) * 0.1
            pre_grasp = self.goal_pose
            pre_grasp.position.x = self.goal_pose.position.x - offset
            if self.goal_pose.position.y > 0:
                pre_grasp.position.y  = self.goal_pose.position.y + offset
            else:
                pre_grasp.position.y = self.goal_pose.position.y - offset
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pre_grasp, 0.1,
                                                                   self.execute)
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
            """
            self.tilt = not (self.goal_pose.position.y > 0)
            orientation_pose.orientation.x = 0.9238795
            if not self.tilt:
                orientation_pose.orientation.y = -0.3826834
                orientation_pose.orientation.z = 0.0
                orientation_pose.orientation.w = 0.0
            else:
                orientation_pose.orientation.y = 0.3826834
                orientation_pose.orientation.z = 0.18137
                orientation_pose.orientation.w = -0.10439
            """
            self.get_logger().info('PLAN')
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                orientation_pose, 0.02,
                                                                self.execute)
            self.get_logger().info('DONE')
            time.sleep(2)
            self.prev_state = State.ORIENT
            self.state = State.PREGRAB
        
        elif self.state == State.PREGRAB:
            # go to pre-grab pose
            offset = math.sin(math.pi/2) * 0.1 # 0.125
            pre_grasp = copy.deepcopy(self.goal_pose)
            pre_grasp.position.x = self.goal_pose.position.x - offset
            if self.goal_pose.position.y > 0:
                pre_grasp.position.y  = self.goal_pose.position.y + offset
            else:
                pre_grasp.position.y = self.goal_pose.position.y - offset
            self.pregrasp_pose = pre_grasp
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pre_grasp, 1.2,
                                                                   self.execute)
            time.sleep(2)
            self.prev_state = State.PREGRAB
            self.state = State.GRAB

        elif self.state == State.GRAB:
            # # go to grab pose
            self.get_logger().info('grabbing')
            self.get_logger().info(str(self.goal_pose))
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   self.goal_pose, 1.2,
                                                                   self.execute)
            time.sleep(2)
            self.prev_state = State.GRAB
            self.state = State.CLOSE

        elif self.state == State.CLOSE:
            # Change the number in grab
            self.future = await self.PlanEx.grab(0.0495)
            time.sleep(4) # maybe change to a counter rather than sleep 
            # go to pull pose
            self.prev_state = State.CLOSE
            self.state = State.PULL

        elif self.state == State.PULL:
            self.prev_state = State.PLACEPLANE
            self.get_logger().info('pulling')
            # TODO: pull block out straight
            pull_pose = copy.deepcopy(self.pregrasp_pose)
            self.get_logger().info(str(pull_pose))
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pull_pose, 0.25,
                                                                   self.execute)
            time.sleep(2)
            d = math.sqrt(pull_pose.position.x**2+pull_pose.position.y**2)
            self.get_logger().info('\n\n\ndistance\n\n\n')
            self.get_logger().info(str(d))
            self.prev_state = State.PULL
            self.get_logger().info(str(self.prev_state))
            self.state = State.POSTPULL

        elif self.state == State.POSTPULL:
            self.get_logger().info('\n\n\nPOOOOOSTPULLLLL\n\n\n')
            postpull_pose = copy.deepcopy(self.pregrasp_pose)
            # postpull_pose.position.x = 0.5
            postpull_pose.position.z = 0.487
            self.prev_state = State.POSTPULL
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   postpull_pose, 1.0,
                                                                   self.execute)
            time.sleep(2)
            self.state = State.READY
        elif self.state == State.READY:
            self.get_logger().info('State.Ready')
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
            if self.prev_state == State.POSTPULL:
                self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                       ready_pose, 1.0,
                                                                       self.execute)
                self.get_logger().info('ORIENTING')
                self.prev_state = State.READY
                self.state = State.ORIENT2
            elif self.prev_state == State.POSTPUSH:
                # await self.place_tower()
                self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                                    ready_pose, joint_position,
                                                                    0.001, self.execute)
                self.future = await self.PlanEx.release()
                self.get_logger().info('IDLE')
                self.prev_state = State.READY
                self.state = State.IDLE
                self.motion_complete_pub.publish(Bool())
            else:
                self.future = await self.PlanEx.plan_to_pose(self.start_pose,
                                                                    ready_pose, joint_position,
                                                                    0.001, self.execute)
                self.get_logger().info('IDLE')
                self.prev_state = State.READY
                self.state = State.IDLE
            time.sleep(2)

        elif self.state == State.ORIENT2:
            # TODO update with either +45 or -45
            self.get_logger().info('ORIENT sencond')
            set_pose = copy.deepcopy(self.goal_pose)
            """
            set_pose.orientation.x = 0.9238795
            if self.place_counter < 3:
                set_pose.orientation.y = 0.3826834
            else:
                set_pose.orientation.y = -0.3826834

            if not self.tilt:
                set_pose.orientation.z = 0.0
                set_pose.orientation.w = 0.0
            else:
                if self.place_counter < 3:
                    set_pose.orientation.z = 0.18137
                    set_pose.orientation.w = -0.10439
                else:
                    set_pose.orientation.z = 0.18137
                    set_pose.orientation.w = 0.05847
            
            set_pose.orientation.x = 0.9238795
            if self.place_counter < 3:
                set_pose.orientation.y = 0.3826834
                set_pose.orientation.w = -0.10439
            else:
                set_pose.orientation.y = -0.3826834
                set_pose.orientation.w = 0.05847
            set_pose.orientation.z = 0.18137
            """
            set_pose.orientation.x = 0.9238795
            if self.place_counter < 3:
                set_pose.orientation.y = 0.3826834
            else:
                set_pose.orientation.y = -0.3826834
            set_pose.orientation.z = 0.0
            set_pose.orientation.w = 0.0
            
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                set_pose, 0.02,
                                                                self.execute)
            time.sleep(2)
            self.prev_state = State.ORIENT2
            self.state = State.REMOVETOWER
        elif self.state == State.REMOVETOWER:
            # self.future = await self.PlanEx.removeTower()
            self.prev_state = State.REMOVETOWER
            self.state = State.SET
        elif self.state == State.SET:
            # TODO need six positions for the block: left1, center1, right1, left2, center2, right2
            # for now, we will hard code this to be left1
            # we need an offset for the x and y
            set_pose = copy.deepcopy(self.goal_pose)
            self.place_pose = self.place_locations[self.place_counter]
            
            offset = math.sin(math.pi/2) * 0.03
            # Would be a different sign if on the other side
            set_pose.position.x = self.place_pose.position.x - offset
            if self.place_counter < 3:
                set_pose.position.y = self.place_pose.position.y - offset
            else:
                set_pose.position.y = self.place_pose.position.y + offset
            # TODO update this with height of tower
            set_pose.position.z = self.place_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   set_pose, 1.2,
                                                                   self.execute)
            time.sleep(2)
            self.prev_state = State.SET            
            self.state = State.RELEASE
        elif self.state == State.RELEASE:
            self.future = await self.PlanEx.release()
            time.sleep(1)
            self.prev_state = State.RELEASE
            self.state = State.PREPUSH
        
        elif self.state == State.PREPUSH:
            prepush_pose = copy.deepcopy(self.goal_pose)
            
            offset = math.sin(math.pi/2) * 0.08
            prepush_pose.position.x = self.place_pose.position.x - offset
            if self.place_counter < 3:
                prepush_pose.position.y = self.place_pose.position.y - offset
            else:
                prepush_pose.position.y = self.place_pose.position.y + offset
            # prepush_pose.position.y = self.place_pose.position.y - offset
            prepush_pose.position.z = self.place_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   prepush_pose, 1.2,
                                                                   self.execute)
            time.sleep(2)
            self.prev_state = State.PREPUSH            
            self.state = State.PREPUSHFINGER
        elif self.state == State.PREPUSHFINGER:
            self.future = await self.PlanEx.grab(0.0) # maybe TODO create a new function for this state
            time.sleep(4)
            self.prev_state = State.PREPUSHFINGER
            self.state = State.PUSH
        elif self.state == State.PUSH:
            push_pose = copy.deepcopy(self.goal_pose)
            offset = math.sin(math.pi/2) * 0.02 # 0.03
            push_pose.position.x = self.place_pose.position.x - offset
            if self.place_counter < 3:
                push_pose.position.y = self.place_pose.position.y - offset
            else:
                push_pose.position.y = self.place_pose.position.y + offset
            push_pose.position.z = self.place_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   push_pose, 0.25,
                                                                   self.execute)
            time.sleep(2)
            self.prev_state = State.PUSH
            self.state = State.POSTPUSH
        elif self.state == State.POSTPUSH:
            postpush_pose = copy.deepcopy(self.goal_pose)
            offset = math.sin(math.pi/2) * 0.08
            postpush_pose.position.x = self.place_pose.position.x - offset
            if self.place_counter < 3:
                postpush_pose.position.y = self.place_pose.position.y - offset
            else:
                postpush_pose.position.y = self.place_pose.position.y + offset
            postpush_pose.position.z = self.place_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   postpush_pose, 1.2,
                                                                   self.execute)
            time.sleep(2)
            self.place_counter += 1
            if (self.place_counter == 3) or (self.place_counter == 6):
                # Publish something
                self.layer_added_pub.publish(Bool())
                pass
            if self.place_counter>=6:
                self.place_counter = 0
                # TODO increment all of the zs in self.place_locations
                for i in range(0, len(self.place_locations)):
                    self.place_locations[i].position.z += 2.0*self.piece_height
            self.prev_state = State.POSTPUSH
            self.state = State.READY
            self.place_counter += 1
            if self.place_counter>=6:
                self.place_counter = 0
                # TODO increment all of the zs in self.place_locations
        
        elif self.state == State.PLACEBLOCK:
            self.prev_state = State.PLACEBLOCK
            self.state = State.IDLE
            # place block
            await self.PlanEx.place_block(self.block_pose, [0.15, 0.05, 0.3], 'block')
            time.sleep(3)
        elif self.state == State.PREPICKUP:
            self.prev_state = State.PREPICKUP
            self.state = State.PICKUP
            prepickup_pose = copy.deepcopy(self.goal_pose)
            prepickup_pose.position.x = 0.404
            prepickup_pose.position.y = 0.293
            prepickup_pose.position.z = 0.487
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   prepickup_pose, 1.2,
                                                                   self.execute)
            time.sleep(2)
        elif self.state == State.PICKUP:
            self.prev_state = State.PICKUP
            self.state = State.LIFT
            pickup_pose = copy.deepcopy(self.goal_pose)
            pickup_pose.position.x = 0.404
            pickup_pose.position.y = 0.293
            pickup_pose.position.z = 0.038
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   pickup_pose, 1.0,
                                                                   self.execute)
            self.future = await self.PlanEx.grab(0.025)
            time.sleep(4)
        elif self.state == State.LIFT:
            self.prev_state = State.LIFT
            self.state = State.PREPOKE
            lift_pose = copy.deepcopy(self.goal_pose)
            lift_pose.position.x = 0.404
            lift_pose.position.y = 0.293
            lift_pose.position.z = self.poke_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   lift_pose, 0.8,
                                                                   self.execute)
            time.sleep(2)
        
        elif self.state == State.PREPOKE:
            self.prev_state = State.PREPOKE
            self.state = State.ORIENT3
            prepoke_pose = copy.deepcopy(self.goal_pose)
            prepoke_pose.position.x = self.poke_pose.position.x
            prepoke_pose.position.y = self.poke_pose.position.y
            prepoke_pose.position.z = self.poke_pose.position.z
            prepoke_pose.orientation.x = 0.9238795
            prepoke_pose.orientation.y = 0.3826834
            prepoke_pose.orientation.z = 0.0
            prepoke_pose.orientation.w = 0.0
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   prepoke_pose, 0.8,
                                                                   self.execute)
            time.sleep(2)
        
        elif self.state == State.ORIENT3:
            self.prev_state = State.ORIENT3
            self.state = State.POKE
            orient_pose = copy.deepcopy(self.goal_pose)
            orient_pose.position.x = 0.404
            orient_pose.position.y = 0.293
            orient_pose.position.z = self.poke_pose.position.z
            orient_pose.orientation.x = 0.9238795
            orient_pose.orientation.y = 0.3826834
            orient_pose.orientation.z = 0.0
            orient_pose.orientation.w = 0.0
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                orient_pose, 0.02,
                                                                self.execute)
            time.sleep(3)

        elif self.state == State.POKE:
            self.prev_state = State.POKE
            self.state = State.POSTPOKE
            poke_pose = copy.deepcopy(self.goal_pose)
            offset = math.sin(math.pi/2) * 0.08
            poke_pose.position.x = self.poke_pose.position.x - offset
            poke_pose.position.y = self.poke_pose.position.y - offset
            poke_pose.position.z = self.poke_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   poke_pose, 0.25,
                                                                   self.execute)
            time.sleep(3)
        
        elif self.state == State.POSTPOKE:
            self.prev_state = State.POSTPOKE
            self.state = State.ORIENT4
            postpoke_pose = copy.deepcopy(self.goal_pose)
            postpoke_pose.position.x = self.poke_pose.position.x
            postpoke_pose.position.y = self.poke_pose.position.y
            postpoke_pose.position.z = self.poke_pose.position.z
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   postpoke_pose, 0.25,
                                                                   self.execute)
            time.sleep(3)
        
        elif self.state == State.ORIENT4:
            self.prev_state = State.ORIENT4
            self.state = State.PLACEPOKER
            orient_pose = copy.deepcopy(self.goal_pose)
            orient_pose.position.x = 0.404
            orient_pose.position.y = 0.293
            orient_pose.position.z = 0.038
            orient_pose.orientation.x = 1.0
            orient_pose.orientation.y = 0.0
            orient_pose.orientation.z = 0.0
            orient_pose.orientation.w = 0.0
            self.future = await self.PlanEx.plan_to_orientation(self.start_pose,
                                                                orient_pose, 0.02,
                                                                self.execute)
            time.sleep(2)
        
        elif self.state == State.PLACEPOKER:
            self.prev_state = State.PLACEPOKER
            self.state = State.POSTPLACEPOKER
            placepoker_pose = copy.deepcopy(self.goal_pose)
            placepoker_pose.position.x = 0.404
            placepoker_pose.position.y = 0.293
            placepoker_pose.position.z = 0.038
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   placepoker_pose, 0.25,
                                                                   self.execute)
            self.future = await self.PlanEx.release()
            time.sleep(4)

        elif self.state == State.POSTPLACEPOKER:
            self.prev_state = State.POSTPLACEPOKER
            self.state = State.READY
            postplacepoker_pose = copy.deepcopy(self.goal_pose)
            postplacepoker_pose.position.x = 0.404
            postplacepoker_pose.position.y = 0.293
            postplacepoker_pose.position.z = 0.487
            self.future = await self.PlanEx.plan_to_cartisian_pose(self.start_pose,
                                                                   postplacepoker_pose, 1.0,
                                                                   self.execute)
            time.sleep(3)

        elif self.state == State.FINDPIECE:
            try:
                t = self.tf_buffer.lookup_transform('panda_link0', 'brick', rclpy.time.Time())
                self.get_logger().info(f'transform bw base and brick:\n{t}')
                self.get_logger().info('Go to orient')
                self.goal_pose.position.x = t.transform.translation.x
                """
                if self.goal_pose.position.y > 0:
                    self.goal_pose.position.y = t.transform.translation.y + 0.008
                else:
                    self.goal_pose.position.y = t.transform.translation.y - 0.008
                """
                self.goal_pose.position.y = t.transform.translation.y
                # HARDCODED OFFSET LMAO WILL IT WORK?
                self.get_logger().info(f'y init: {t.transform.translation.y}')
                if self.goal_pose.position.y > 0:
                    self.goal_pose.position.y -= 1.95*self.tower_top_pose.position.y
                else:
                    self.goal_pose.position.x += 0.012
                    self.goal_pose.position.y -= 2.3*self.tower_top_pose.position.y
                # if self.goal_pose.position.y < 0:
                #     self.goal_pose.position.y -= self.tower_top_pose.position.y
                # else: 
                #     self.goal_pose.position.y -= self.tower_top_pose.position.y
                if self.goal_pose.position.y > 0:
                    self.goal_pose.position.z = t.transform.translation.z + 0.008
                else:
                    self.goal_pose.position.z = t.transform.translation.z - 0.006
                self.goal_pose.orientation.x = t.transform.rotation.x
                self.goal_pose.orientation.y = t.transform.rotation.y
                self.goal_pose.orientation.z = t.transform.rotation.z
                self.goal_pose.orientation.w = t.transform.rotation.w
                self.get_logger().info(f'Goal Pose:\n{self.goal_pose}')
                # publish when found so cv node knows when to stop publishing
                self.piece_found_pub.publish(Bool())
                self.state = State.ORIENT
            except TransformException:
                print("couldn't do panda_link0->brick transform")
        elif self.state == State.FINDTOP:
            try:
                t = self.tf_buffer.lookup_transform('panda_link0', 'starting_top', rclpy.time.Time())
                self.get_logger().info(f'transform bw base and top:\n{t}')
                self.tower_top_pose.position.x = t.transform.translation.x
                self.tower_top_pose.position.y = t.transform.translation.y + 0.005
                self.tower_top_pose.position.z = t.transform.translation.z + 0.024
                self.get_logger().info(f'TOWER top Pose:\n{self.tower_top_pose}')
                # publish when found so cv node knows when to stop publishing
                self.place_pose.position.z = self.tower_top_pose.position.z
                # assume tower at 45 degree offset
                # These 3 pieces are where the center of the tower should end up.
                s = self.piece_width/sqrt(2)
                offset = 0.03
                # Use self.top_ori to determine which one. +1 = Normal (below). -1: opposite
                piece_1 = Pose()
                piece_1.position.x = self.tower_top_pose.position.x + s - offset
                piece_1.position.y = self.tower_top_pose.position.y - s - offset
                piece_1.position.z = self.tower_top_pose.position.z
                self.get_logger().info(f'PIECE1:\n{piece_1}')
                piece_2 = Pose()
                piece_2.position.x = self.tower_top_pose.position.x - offset
                piece_2.position.y = self.tower_top_pose.position.y - offset
                piece_2.position.z = self.tower_top_pose.position.z
                self.get_logger().info(f'PIECE2:\n{piece_2}')
                piece_3 = Pose()
                piece_3.position.x = self.tower_top_pose.position.x - s - offset
                piece_3.position.y = self.tower_top_pose.position.y + s - offset
                piece_3.position.z = self.tower_top_pose.position.z
                self.get_logger().info(f'PIECE3:\n{piece_3}')
                piece_4 = Pose()
                piece_4.position.x = self.tower_top_pose.position.x + s - offset
                piece_4.position.y = self.tower_top_pose.position.y + s + offset
                piece_4.position.z = self.tower_top_pose.position.z + self.piece_height
                self.get_logger().info(f'PIECE4:\n{piece_4}')
                piece_5 = Pose()
                piece_5.position.x = self.tower_top_pose.position.x - offset
                piece_5.position.y = self.tower_top_pose.position.y + offset
                piece_5.position.z = self.tower_top_pose.position.z + self.piece_height
                self.get_logger().info(f'PIECE5:\n{piece_5}')
                piece_6 = Pose()
                piece_6.position.x = self.tower_top_pose.position.x - s - offset
                piece_6.position.y = self.tower_top_pose.position.y + s - offset
                piece_6.position.z = self.tower_top_pose.position.z + self.piece_height
                self.get_logger().info(f'PIECE6:\n{piece_6}')
                self.place_locations = [piece_1, piece_2, piece_3, piece_4, piece_5, piece_6]
                self.place_pose = piece_1
                self.state = State.IDLE
            except TransformException:
                print("couldn't do panda_link0->tower transform")
        elif self.state == State.LETGO:
            self.future = await self.PlanEx.release()
            self.get_logger().info("DONE RELEASING")
            self.state = State.IDLE

def test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()
