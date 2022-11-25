import rclpy
import math
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from ament_index_python.packages import get_package_share_path
import numpy as np
from tf2_ros import TransformBroadcaster
# np.set_printoptions(threshold=sys.maxsize)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


def quaternion_from_euler(ai, aj, ak): # I STOLE THIS FROM THE ROS DOCS
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def quaternion_multiply(Q0,Q1): # I STOLE THIS FROM AUTOMATIC ADDISON
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion

def deg_to_rad(deg):
    rad = math.pi/180*deg
    return rad

class Calibrate(Node):
    def __init__(self):
        super().__init__('cali')
        self.freq = 60.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        # Define frames
        self.frame_camera = "camera_color_optical_frame"
        self.frame_tag = "tag36h11:0"
        self.frame_rotate = "rotate"
        self.frame_ee = "panda_hand_tcp"
        self.frame_base = "panda_link0"

        self.rot = TransformStamped()
        self.t = TransformStamped()
        self.rot_base = TransformStamped()

        camera_path = get_package_share_path('camera')
        self.tf_path  = camera_path / 'tf.rviz'

    def timer_callback(self):
        #listener for the camera to tag
        try:
            r = self.tf_buffer.lookup_transform(
                self.frame_camera,
                self.frame_tag,
                rclpy.time.Time())
            og_q = np.array([r.transform.rotation.x, r.transform.rotation.y,
                             r.transform.rotation.z, r.transform.rotation.w])
            # self.get_logger().info(f"t: {r}")
        except:
            self.get_logger().info(
                f'Could not transform {self.frame_camera} to {self.frame_tag}')
            return


        # listener for the end effector to the base
        try:
            s = self.tf_buffer.lookup_transform(
                self.frame_ee,
                self.frame_base,
                rclpy.time.Time())
            # self.get_logger().info(f"s: {s}")
        except:
            self.get_logger().info(
                f'Could not transform {self.frame_ee} to {self.frame_base}')
            return

        rad = deg_to_rad(90)
        #create tf between the tag and the rotated frame
        self.rot.header.stamp = self.get_clock().now().to_msg()
        self.rot.header.frame_id = self.frame_tag
        self.rot.child_frame_id = self.frame_rotate
        self.rot.transform.translation.x = 0.0
        self.rot.transform.translation.y = 0.0
        self.rot.transform.translation.z = 0.0
        # calculate the rotations with quaternians
        z_rot = quaternion_from_euler(0.0, 0.0, rad)
        qz_rot = quaternion_multiply(og_q, z_rot)
        y_rot = quaternion_from_euler(-rad, 0.0, 0.0)
        q_new = quaternion_multiply(qz_rot, y_rot)
        self.rot.transform.rotation.x = q_new[0]
        self.rot.transform.rotation.y = q_new[1]
        self.rot.transform.rotation.z = q_new[2]
        self.rot.transform.rotation.w = q_new[3]
        self.tf_broadcaster.sendTransform(self.rot)

        # create a tf between the rotated frame and panda_link0
        self.rot_base = s
        self.rot_base.header.stamp = self.get_clock().now().to_msg()
        self.rot_base.header.frame_id = self.frame_rotate
        self.rot_base.child_frame_id = self.frame_base
        self.tf_broadcaster.sendTransform(self.rot_base)

        # listener for the camera to base
        try:
            cam_base = self.tf_buffer.lookup_transform(
                self.frame_camera,
                self.frame_base,
                rclpy.time.Time())
            self.get_logger().info(f"cam_base: {cam_base}")
            dump = {'ros_parameters': {'x_trans': cam_base.transform.translation.x,
                                       'y_trans': cam_base.transform.translation.y,
                                       'z_trans': cam_base.transform.translation.z,
                                       'x_rot': cam_base.transform.rotation.x,
                                       'y_rot': cam_base.transform.rotation.y,
                                       'z_rot': cam_base.transform.rotation.z,
                                       'w_rot': cam_base.transform.rotation.w}}
        except:
            self.get_logger().info(
                f'Could not transform {self.frame_camera} to {self.frame_base}')
            return
        # self.get_logger().info(self.tf_path)
        with open('tf.yaml', 'w') as file: 
                yaml.dump(dump, file)
        

def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    c = Calibrate()
    rclpy.spin(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()