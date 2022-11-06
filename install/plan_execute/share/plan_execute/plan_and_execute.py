import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from rclpy.callback_groups import ReentrantCallbackGroup
# import robot state type
from moveit_msgs.msg import RobotState
# import joint state type
from sensor_msgs.msg import JointState
# import header type
from std_msgs.msg import Header

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
    async def plan_to_position(self, start_pose, end_pos):
        """Returns MoveGroup action from a start pose to an end position"""
        request = PositionIKRequest()
        printIKreq(request)
        request.group_name = '' # NOt sure how to find this
        # Robot state is "Seed" guess. IDK how to get this. Since it's in angles? 
        # MUST CONTAIN STATE OF ALL JOINTS TO BE USED BY IK SOLVER

        # I think we should use the joint_state message, I also think this can probably be initialized
        print(type(request))
        request.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2',
                       'panda_joint3','panda_joint4',
                       'panda_joint5','panda_joint6,panda_joint7',
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
        request.pose_stamped.header.frame_id = 'name_of_end_effector'
        request.pose_stamped.pose.position = end_pos
        request.timeout.sec = 20
        printIKreq(request)
        # convert end_pos to end_pose
        angles = await self.node.IK.call_async(GetPositionIK.Request(ik_request = request))
        print(angles)
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
        pass