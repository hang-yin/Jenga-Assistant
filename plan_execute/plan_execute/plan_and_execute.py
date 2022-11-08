import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint, RobotTrajectory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# import moveit movegroup

def printIKreq(req):
    print()
    print((str(req)).replace(',',',\n'))
    print()

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
        self.master_goal.request.group_name = 'panda_arm'
        self.master_goal.request.max_velocity_scaling_factor = 1.0
        self.master_goal.request.max_acceleration_scaling_factor = 0.1
        # when planning, set goal.request.plan_only to True, 
        # when executing, set goal.request.plan_only to False
        # printIKreq(self.master_goal)

    def js_callback(self, data):
        """Save js (sensor_msgs/JointStates type)."""
        self.js = data
        #Update start
        self.master_goal.request.start_state.joint_state = data

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
        request.group_name = 'panda_arm'
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

    async def plan_to_position(self, start_pose, end_pos, execute):
        """Returns MoveGroup action from a start pose to an end position"""
        startpose = Pose()
        t = self.tf_buffer.lookup_transform(
                                            self.master_goal.request.workspace_parameters.header.frame_id,
                                            'panda_link8',
                                            rclpy.time.Time())
        startpose.position.x = t.transform.translation.x
        startpose.position.y = t.transform.translation.y
        startpose.position.z = t.transform.translation.z
        startpose.orientation.x = t.transform.rotation.x
        startpose.orientation.y =  t.transform.rotation.y
        startpose.orientation.z = t.transform.rotation.z
        startpose.orientation.w = t.transform.rotation.w
        printIKreq(f"START POSITION{startpose}")
        request = self.createIKreq(end_pos, startpose.orientation)

        response = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request))
        # printIKreq(response.solution)
        # printIKreq(response.error_code)
        joint_names = response.solution.joint_state.name
        joint_positions = np.array(response.solution.joint_state.position)
        # print(joint_names)
        # print(np.array(joint_positions))
        # print("FILLING WITH RESULT OF IK \n\n\n")
        self.fill_constraints(joint_names, joint_positions)
        self.master_goal.planning_options.plan_only = True
        # print("wait for server")
        self.node._action_client.wait_for_server()
        # print("return")
        plan = await self.node._action_client.send_goal_async(self.master_goal)
        # printIKreq(plan)
        # print(type(plan))
        result = await plan.get_result_async()
        # print("RESULT")
        # printIKreq(result)
        if execute:
            # print("Wait for execute client")
            self.node._execute_client.wait_for_server()
            # print("Send thing")
            # print(type(result))
            # print("\n\nhere\n\n")
            # printIKreq(result.result.error_code)
            # printIKreq(result.result.planned_trajectory)
            # print("go")
            plan2 = await self.node._execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=result.result.planned_trajectory))
            # print("DONE??")
            # printIKreq(plan2)
            return plan2
        else:
            return result
        

        # return response.solution, response.error_code
        # put into the move group 
        # 1. Call GetPositionIK.srv to get the joint states of final position 
        # 2. Wait for service response (await?)
        # 3. Receive RobotState soln. soln.joint_state gives joint_state type msg
        # 4. Plug this into mvg.request.goal_constraints.joint_constraints (joint_state type)
        # 5. Return mvg action
        # return mvg
    async def plan_to_orientation(self, start_pose, end_orientation, execute):
        """Returns MoveGroup action from a start pose to an end orientation"""
        request = self.createIKreq(Point(), end_orientation)
        response = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request))
        printIKreq(response.solution)
        printIKreq(response.error_code)
        joint_names = response.solution.joint_state.name
        joint_positions = np.array(response.solution.joint_state.position)
        print(joint_names)
        print(np.array(joint_positions))
        print("FILLING WITH RESULT OF IK \n\n\n")
        self.fill_constraints(joint_names, joint_positions)
        self.master_goal.planning_options.plan_only = True
        print("wait for server")
        self.node._action_client.wait_for_server()
        print("return")
        plan = await self.node._action_client.send_goal_async(self.master_goal)
        printIKreq(plan)
        print(type(plan))
        result = await plan.get_result_async()
        print("RESULT")
        if execute:
            print("Wait for execute client")
            self.node._execute_client.wait_for_server()
            print("Send thing")
            print(type(result))
            print("\n\nhere\n\n")
            # printIKreq(result.result.error_code)
            printIKreq(result.result.planned_trajectory)
            print("go")
            plan2 = await self.node._execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=result.result.planned_trajectory))
            print("DONE??")
            printIKreq(plan2)
            return plan2
        else:
            return result

    
    async def plan_to_pose(self, start_pose, end_pose, execute):
        """Returns MoveGroup action from a start pose to an end pose (position + orientation)"""
        request = self.createIKreq(end_pose.position, end_pose.orientation)
        response = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request))
        printIKreq(response.solution)
        printIKreq(response.error_code)
        joint_names = response.solution.joint_state.name
        joint_positions = np.array(response.solution.joint_state.position)
        print(joint_names)
        print(np.array(joint_positions))
        print("FILLING WITH RESULT OF IK \n\n\n")
        self.fill_constraints(joint_names, joint_positions)
        self.master_goal.planning_options.plan_only = True
        print("wait for server")
        self.node._action_client.wait_for_server()
        print("return")
        plan = await self.node._action_client.send_goal_async(self.master_goal)
        printIKreq(plan)
        print(type(plan))
        result = await plan.get_result_async()
        print("RESULT")
        if execute:
            print("Wait for execute client")
            self.node._execute_client.wait_for_server()
            print("Send thing")
            print(type(result))
            print("\n\nhere\n\n")
            # printIKreq(result.result.error_code)
            printIKreq(result.result.planned_trajectory)
            print("go")
            plan2 = await self.node._execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=result.result.planned_trajectory))
            print("DONE??")
            printIKreq(plan2)
            return plan2
        else:
            return result