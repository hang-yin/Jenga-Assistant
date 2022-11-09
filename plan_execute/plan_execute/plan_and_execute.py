import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, PlanningScene
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# import moveit movegroup


class PlanAndExecute:
    def __init__(self, node):
        self.node = node
        # Create a client
        self.node._action_client = ActionClient(self.node,
                                                MoveGroup,
                                                '/move_action')
        self.node._execute_client = ActionClient(self.node,
                                                ExecuteTrajectory,
                                                '/execute_trajectory')
        # Make it so we can call the IK service
        self.node.cbgroup = MutuallyExclusiveCallbackGroup()
        self.node.IK = self.node.create_client(GetPositionIK,
                                               "compute_ik",
                                               callback_group = self.node.cbgroup)
        if not self.node.IK.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "IK" service to become available')
        # Get MoveGroup() from the node
        self.move_group = self.node.movegroup #idk if this is even close to right <3 -Liz
        self.node.js_sub = self.node.create_subscription(JointState,
                                                    "/joint_states",
                                                    self.js_callback,
                                                    10)
        self.js = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.node.block_pub = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
        # define a generic MoveGroup Goal
        self.master_goal = MoveGroup.Goal()
        self.master_goal.request.workspace_parameters.header.frame_id = 'panda_link0'
        self.master_goal.request.workspace_parameters.min_corner.x = -1.0
        self.master_goal.request.workspace_parameters.min_corner.y = -1.0
        self.master_goal.request.workspace_parameters.min_corner.z = -1.0
        self.master_goal.request.workspace_parameters.max_corner.x = 1.0
        self.master_goal.request.workspace_parameters.max_corner.y = 1.0
        self.master_goal.request.workspace_parameters.max_corner.z = 1.0
        self.master_goal.request.group_name = 'panda_arm'
        self.master_goal.request.num_planning_attempts = 10
        self.master_goal.request.allowed_planning_time = 5.0
        self.master_goal.request.planner_id = ''
        # add constraints???
        # self.master_goal.request.goal_constraints = []
        self.master_goal.request.start_state.is_diff = False
        self.master_goal.request.start_state.joint_state.velocity = []
        self.master_goal.request.start_state.joint_state.effort = []
        self.master_goal.request.start_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.master_goal.request.start_state.attached_collision_objects = []
        self.master_goal.request.start_state.is_diff = False
        self.master_goal.request.pipeline_id = 'move_group'
        self.master_goal.request.max_velocity_scaling_factor = 1.0
        self.master_goal.request.max_acceleration_scaling_factor = 0.1
        # when planning, set goal.request.plan_only to True, 
        # when executing, set goal.request.plan_only to False
    
    def printIKreq(self, req):
        new_str = (str(req)).replace(',',',\n')
        self.node.get_logger().info(new_str)

    def js_callback(self, data):
        """Save js (sensor_msgs/JointStates type)."""
        self.js = data

    def fill_constraints(self, joint_names, joint_positions):
        constraints = []
        for n, i in enumerate(joint_names):
            name = i
            pos = joint_positions[n]
            print(name, pos)
            constraint_i = JointConstraint(joint_name=name,
                                           position=float(pos),
                                           tolerance_above=0.0001,
                                           tolerance_below=0.0001,
                                           weight=1.0)
            constraints.append(constraint_i)
        self.master_goal.request.goal_constraints = [Constraints(name='', joint_constraints=constraints)]
    
    def createIKreq(self, end_pos, end_orientation):
        request = PositionIKRequest()
        request.group_name = self.master_goal.request.group_name
        # "Seed" position but it doesn't really matter since almost always converges
        request.robot_state.joint_state.name = self.js.name
        request.robot_state.joint_state.position = self.js.position
        request.robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        request.constraints.name = ''
        request.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        request.pose_stamped.header.frame_id = 'panda_link0'
        request.pose_stamped.pose.position = end_pos
        request.pose_stamped.pose.orientation = end_orientation
        request.timeout.sec = 20
        return request

    def getStartPose(self):
        startpose = Pose()
        t = self.tf_buffer.lookup_transform(
                                            self.master_goal.request.workspace_parameters.header.frame_id,
                                            'panda_hand_tcp',
                                            rclpy.time.Time())
        startpose.position.x = t.transform.translation.x
        startpose.position.y = t.transform.translation.y
        startpose.position.z = t.transform.translation.z
        startpose.orientation.x = t.transform.rotation.x
        startpose.orientation.y =  t.transform.rotation.y
        startpose.orientation.z = t.transform.rotation.z
        startpose.orientation.w = t.transform.rotation.w
        self.printIKreq(startpose)
        return startpose

    async def plan_to_position(self, start_pose, end_pos, execute):
        """Returns MoveGroup action from a start pose to an end position"""
        print("Plan to position")
        if not start_pose:
            # We start at current location 
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            # compute ik to get joint states of start
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            response_start = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request_start))
            self.master_goal.request.start_state.joint_state = response_start.solution.joint_state
        self.master_goal.planning_options.plan_only = not execute
        request = self.createIKreq(end_pos.position, start_pose.orientation)
        plan_result = await self.plan(request)
        if execute:
            execute_result = await self.execute(plan_result)
            return execute_result
        else:
            return plan_result

    async def plan_to_orientation(self, start_pose, end_orientation, execute):
        """Returns MoveGroup action from a start pose to an end orientation"""
        self.node.get_logger().info("\n\n\n\n\nHERE\n\n\n\n\n\n")
        if not start_pose:
            # We start at current location 
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            # compute ik to get joint states of start
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            response_start = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request_start))
            self.master_goal.request.start_state.joint_state = response_start.solution.joint_state
        self.master_goal.planning_options.plan_only = not execute
        request = self.createIKreq(start_pose.position, end_orientation.orientation)
        print("REQUEST")
        self.printIKreq(request)
        plan_result = await self.plan(request)
        if execute:
            execute_result = await self.execute(plan_result)
            return execute_result
        else:
            return plan_result

    
    async def plan_to_pose(self, start_pose, end_pose, execute):
        """Returns MoveGroup action from a start pose to an end pose (position + orientation)"""
        print("Plan to Pose")
        if not start_pose:
            # We start at current location 
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            # compute ik to get joint states of start
            print("NEW START")
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            response_start = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request_start))
            self.master_goal.request.start_state.joint_state = response_start.solution.joint_state
            print(response_start)
            print(self.master_goal.request.start_state.joint_state)
        
        self.master_goal.planning_options.plan_only = not execute
        request = self.createIKreq(end_pose.position, end_pose.orientation)
        plan_result = await self.plan(request)
        if execute:
            execute_result = await self.execute(plan_result)
            print("EXECUTE")
            return execute_result
        else:
            print("PLAN")
            return plan_result

    async def plan(self, IKrequest):
        response = await self.node.IK.call_async(GetPositionIK.Request(ik_request = IKrequest))
        joint_names = response.solution.joint_state.name
        joint_positions = np.array(response.solution.joint_state.position)
        self.fill_constraints(joint_names, joint_positions)
        self.node._action_client.wait_for_server()
        self.printIKreq(self.master_goal)
        plan = await self.node._action_client.send_goal_async(self.master_goal)
        plan_result = await plan.get_result_async()
        return plan_result

    async def execute(self, plan_result):
        print("Wait for execute client")
        self.node._execute_client.wait_for_server()
        execute_future = await self.node._execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=plan_result.result.planned_trajectory))
        execute_result = await execute_future.get_result_async()
        return execute_result

    def place_block(self, pos):
        print("Place Block")
        scene = PlanningScene()
        # Fill in with position
        scene.robot_state.joint_state = self.js
        scene.robot_model_name = 'panda'
        self.node.block_pub.publish(scene)
