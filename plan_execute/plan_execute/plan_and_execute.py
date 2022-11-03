import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient



class PlanAndExecute:
    def __init__(self, node):
        # super().__init__("our_node")
        # Create a client
        self.node = node
        self.node._action_client = ActionClient(self,
                                                MoveGroup,
                                                '/move_action')
    def plan_to_position(self, start_pos, end_pos):
        mvg = MoveGroup()
        # 1. Call GetPositionIK.srv to get the joint states of final position 
        # 2. Wait for service response (await?)
        # 3. Receive RobotState soln. soln.joint_state gives joint_state type msg
        # 4. Plug this into mvg.request.goal_constraints.joint_constraints (joint_state type)
        # 5. Return mvg action
        return mvg
    def plan_to_orientation(self, start_pos, end_orientation):
        pass 
    def plan_to_pose(self,start_pos, end_pose):
        pass
    def execute(self, mvg):
        # CAll the client
        pass