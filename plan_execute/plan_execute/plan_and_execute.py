import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, \
                            PlanningScene, PlanningSceneComponents, CollisionObject
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from shape_msgs.msg import SolidPrimitive


class PlanAndExecute:
    def __init__(self, node):
        self.node = node
        self.node.cbgroup = MutuallyExclusiveCallbackGroup()
        self.node._action_client = ActionClient(self.node,
                                                MoveGroup,
                                                '/move_action')
        self.node._execute_client = ActionClient(self.node,
                                                 ExecuteTrajectory,
                                                 '/execute_trajectory')
        self.node.IK = self.node.create_client(GetPositionIK,
                                               "compute_ik",
                                               callback_group=self.node.cbgroup)
        if not self.node.IK.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Timeout waiting for "IK" service to become available')
        self.node.planscene = self.node.create_client(GetPlanningScene,
                                                      "get_planning_scene",
                                                      callback_group=self.node.cbgroup)
        if not self.node.planscene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Timeout waiting for "planscene" service to become available')
        self.move_group = self.node.movegroup
        self.node.js_sub = self.node.create_subscription(JointState,
                                                         "/joint_states",
                                                         self.js_callback,
                                                         10)
        self.js = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.node.block_pub = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
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
        self.master_goal.request.start_state.is_diff = False
        self.master_goal.request.start_state.joint_state.velocity = []
        self.master_goal.request.start_state.joint_state.effort = []
        self.master_goal.request.start_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.master_goal.request.start_state.attached_collision_objects = []
        self.master_goal.request.start_state.is_diff = False
        self.master_goal.request.pipeline_id = 'move_group'
        self.master_goal.request.max_velocity_scaling_factor = 1.0
        self.master_goal.request.max_acceleration_scaling_factor = 0.1

    def printBlock(self, req):
        new_str = (str(req)).replace(',', ',\n')
        self.node.get_logger().info(new_str)

    def js_callback(self, data):
        """Save js (sensor_msgs/JointStates type)."""
        self.js = data

    def fill_constraints(self, joint_names, joint_positions):
        constraints = []
        for n, i in enumerate(joint_names):
            name = i
            pos = joint_positions[n]
            constraint_i = JointConstraint(joint_name=name,
                                           position=float(pos),
                                           tolerance_above=0.0001,
                                           tolerance_below=0.0001,
                                           weight=1.0)
            constraints.append(constraint_i)
        self.master_goal.request.goal_constraints = [Constraints(name='',
                                                                 joint_constraints=constraints)]

    def createIKreq(self, end_pos, end_orientation):
        request = PositionIKRequest()
        request.group_name = self.master_goal.request.group_name
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
        temp_frame_id = self.master_goal.request.workspace_parameters.header.frame_id
        t = self.tf_buffer.lookup_transform(
                                            temp_frame_id,
                                            'panda_hand_tcp',
                                            rclpy.time.Time())
        startpose.position.x = t.transform.translation.x
        startpose.position.y = t.transform.translation.y
        startpose.position.z = t.transform.translation.z
        startpose.orientation.x = t.transform.rotation.x
        startpose.orientation.y = t.transform.rotation.y
        startpose.orientation.z = t.transform.rotation.z
        startpose.orientation.w = t.transform.rotation.w
        self.printBlock(startpose)
        return startpose

    async def plan_to_position(self, start_pose, end_pos, execute):
        """Return MoveGroup action from a start pose to an end position."""
        self.node.get_logger().info("Plan to position")
        if not start_pose:
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            request_temp = GetPositionIK.Request(ik_request=request_start)
            response_start = await self.node.IK.call_async(request_temp)
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
        """Return MoveGroup action from a start pose to an end orientation."""
        self.node.get_logger().info("\n\n\n\n\nHERE\n\n\n\n\n\n")
        if not start_pose:
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            request_temp = GetPositionIK.Request(ik_request=request_start)
            response_start = await self.node.IK.call_async(request_temp)
            self.master_goal.request.start_state.joint_state = response_start.solution.joint_state
        self.master_goal.planning_options.plan_only = not execute
        request = self.createIKreq(start_pose.position, end_orientation.orientation)
        self.node.get_logger().info("REQUEST")
        self.printBlock(request)
        plan_result = await self.plan(request)
        if execute:
            execute_result = await self.execute(plan_result)
            return execute_result
        else:
            return plan_result

    async def plan_to_pose(self, start_pose, end_pose, execute):
        """Return MoveGroup action from a start to end pose (position + orientation)."""
        self.node.get_logger().info("Plan to Pose")
        if not start_pose:
            start_pose = self.getStartPose()
            self.master_goal.request.start_state.joint_state = self.js
        else:
            self.node.get_logger().info("NEW START")
            request_start = self.createIKreq(start_pose.position, start_pose.orientation)
            request_temp = GetPositionIK.Request(ik_request=request_start)
            response_start = await self.node.IK.call_async(request_temp)
            self.master_goal.request.start_state.joint_state = response_start.solution.joint_state
            self.node.get_logger().info(response_start)
            self.node.get_logger().info(self.master_goal.request.start_state.joint_state)

        self.master_goal.planning_options.plan_only = not execute
        request = self.createIKreq(end_pose.position, end_pose.orientation)
        plan_result = await self.plan(request)
        if execute:
            execute_result = await self.execute(plan_result)
            self.node.get_logger().info("EXECUTE")
            return execute_result
        else:
            self.node.get_logger().info("PLAN")
            return plan_result

    async def plan(self, IKrequest):
        response = await self.node.IK.call_async(GetPositionIK.Request(ik_request=IKrequest))
        joint_names = response.solution.joint_state.name
        joint_positions = np.array(response.solution.joint_state.position)
        self.fill_constraints(joint_names, joint_positions)
        self.node._action_client.wait_for_server()
        self.printBlock(self.master_goal)
        plan = await self.node._action_client.send_goal_async(self.master_goal)
        plan_result = await plan.get_result_async()
        return plan_result

    async def execute(self, plan_result):
        self.node.get_logger().info("Wait for execute client")
        self.node._execute_client.wait_for_server()
        traj_goal = ExecuteTrajectory.Goal(trajectory=plan_result.result.planned_trajectory)
        execute_future = await self.node._execute_client.send_goal_async(traj_goal)
        execute_result = await execute_future.get_result_async()
        return execute_result

    async def place_block(self, pos):
        self.node.get_logger("Place Block")
        scene_request = PlanningSceneComponents()
        scene_request.components = 0
        temp_scene_request = GetPlanningScene.Request(components=scene_request)
        scene = await self.node.planscene.call_async(temp_scene_request)
        self.printBlock(scene)
        scene = scene.scene
        scene.robot_state.joint_state = self.js
        primepose = Pose()
        prime = SolidPrimitive()
        prime.type = 1
        prime.dimensions = [0.2, 0.2, 0.2]
        collision = CollisionObject()
        collision.header = self.js.header
        collision.header.frame_id = 'panda_link0'
        collision.pose = pos
        collision.id = 'box1'
        collision.primitives = [prime]
        collision.primitive_poses = [primepose]
        scene.world.collision_objects = [collision]
        self.printBlock(scene)
        self.node.block_pub.publish(scene)
