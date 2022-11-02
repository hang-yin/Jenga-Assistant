import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from rclpy.node import Node
from rclpy.action import ActionClient



class PlanAndExecute:
    def __init__(self, node):
        # super().__init__("our_node")
        # Create a client
        node._action_client = ActionClient(self,
                                           MoveGroup,
                                           '/move_action')
    def plan_to_position(self, position):
        pass
    def plan_to_orientation(self, orientation):
        pass 
    def plan_to_pose(self, pose):
        pass
    def execute(self):
        pass