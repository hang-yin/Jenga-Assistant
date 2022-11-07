import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint
from rclpy.callback_groups import ReentrantCallbackGroup
# import moveit movegroup

def printIKreq(req):
    print()
    print((str(req)).replace(',','\n'))
    print()

class PlanAndExecute:
    def __init__(self, node):
        self.node = node
        # Create a client
        self.node._action_client = ActionClient(self.node,
                                                MoveGroup,
                                                '/move_action')
        # Make it so we can call the IK service
        self.node.cbgroup = ReentrantCallbackGroup()
        self.node.IK = self.node.create_client(GetPositionIK,
                                               "compute_ik",
                                               callback_group = self.node.cbgroup)
        # Get MoveGroup() from the node
        self.move_group = self.node.movegroup #idk if this is even close to right <3 -Liz

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
        self.master_goal.request.start_state.joint_state.name = ['panda_joint1', 'panda_joint2',
                                                                 'panda_joint3','panda_joint4',
                                                                 'panda_joint5','panda_joint6','panda_joint7',
                                                                 'panda_finger_joint1','panda_finger_joint2']
        self.master_goal.request.start_state.joint_state.position = [0.0,-0.7853981633974483,0.0,
                                                                     -2.356194490192345,0.0,1.5707963267948966
                                                                     ,0.7853981633974483,0.035,0.035]
        self.master_goal.request.start_state.joint_state.velocity = []
        self.master_goal.request.start_state.joint_state.effort = []
        self.master_goal.request.start_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.master_goal.request.start_state.attached_collision_objects = []
        self.master_goal.request.start_state.is_diff = False
        self.fill_constraints(self.master_goal.request.start_state.joint_state.name,
                              [0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.master_goal.request.pipeline_id = 'move_group'
        self.master_goal.request.group_name = 'panda_arm'
        self.master_goal.request.max_velocity_scaling_factor = 1.0
        self.master_goal.request.max_acceleration_scaling_factor = 0.1
        # when planning, set goal.request.plan_only to True, 
        # when executing, set goal.request.plan_only to False
        # printIKreq(self.master_goal)

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
        print("BEFORE")
        printIKreq(self.master_goal.request.goal_constraints)
        self.master_goal.request.goal_constraints = [Constraints(name='', joint_constraints=constraints)]
        print("AFTER")
        printIKreq(self.master_goal.request.goal_constraints)
    
    async def plan_to_position(self, start_pose, end_pos):
        """Returns MoveGroup action from a start pose to an end position"""
        request = PositionIKRequest()
        # printIKreq(request)
        request.group_name = 'panda_arm' # NOt sure how to find this
        # Robot state is "Seed" guess. IDK how to get this. Since it's in angles? 
        # MUST CONTAIN STATE OF ALL JOINTS TO BE USED BY IK SOLVER

        # I think we should use the joint_state message, I also think this can probably be initialized
        # print(type(request))
        request.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2',
                       'panda_joint3','panda_joint4',
                       'panda_joint5','panda_joint6','panda_joint7',
                       'panda_finger_joint1','panda_finger_joint2']
        request.robot_state.joint_state.position = [-0.2231403838057399, 0.13284448454250933, 
                                                    -0.19602126983568066, -1.4435717608445389, 
                                                    0.0700470262663259, 1.302200406478571,
                                                    0.1637209011241946, 0.035, 0.035]
        # Do we get this by looking at JSP? What if the start state is not current pos? 
        request.robot_state.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        # Constraints: Default empty. Do we need to add?
        request.constraints.name = ''
        # Pose Stamped: How we specify a location of a joint.
        # I think we will assume that the joint we want the position of is the end effector.
        request.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        request.pose_stamped.header.frame_id = 'panda_link0'
        request.pose_stamped.pose.position = end_pos
        request.timeout.sec = 20
        # printIKreq(request)
        # convert end_pos to end_pose
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
        plan = self.node._action_client.send_goal_async(self.master_goal)
        printIKreq(plan)
        return plan
        

        # return response.solution, response.error_code
        # put into the move group 
        # 1. Call GetPositionIK.srv to get the joint states of final position 
        # 2. Wait for service response (await?)
        # 3. Receive RobotState soln. soln.joint_state gives joint_state type msg
        # 4. Plug this into mvg.request.goal_constraints.joint_constraints (joint_state type)
        # 5. Return mvg action
        # return mvg
    async def plan_to_orientation(self, start_pose, end_orientation):
        """Returns MoveGroup action from a start pose to an end orientation"""
        # Make copy of MoveGroup
        mvg = self.move_group
        # Call GetPositionIK.srv
        angles = await self.node.IK.call_async(end_orientation)
        return mvg
    def plan_to_pose(self,start_pose, end_pose):
        """Returns MoveGroup action from a start pose to an end pose (position + orientation)"""
        mvg = MoveGroup()
        return mvg
    def execute(self, mvg):
        """Takes a MoveGroup object, sends it through the client"""
        self.master_goal.planning_options.plan_only = False
        pass