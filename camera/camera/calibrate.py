import rclpy
import math
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformBroadcaster
# np.set_printoptions(threshold=sys.maxsize)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped


def quaternion_from_euler(ai, aj, ak):
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
        self.frame_world = "world"
        self.frame_camera = "camera_color_optical_frame"
        self.frame_tag = "tag36h11:0"
        self.frame_ee = "panda_hand_tcp"
        self.frame_base = "panda_link0"

        s = TransformStamped()

    def timer_callback(self):
        # try:
        #     r = self.tf_buffer.lookup_transform(
        #         self.frame_camera,
        #         self.frame_tag,
        #         rclpy.time.Time())
        # except:
        #     self.get_logger().info(
        #         f'Could not transform {self.frame_camera} to {self.frame_tag}')
        #     return
        # self.get_logger().info(f"t: {r}")

        try:
            s = self.tf_buffer.lookup_transform(
                self.frame_ee,
                self.frame_base,
                rclpy.time.Time())
        except:
            self.get_logger().info(
                f'Could not transform {self.frame_ee} to {self.frame_base}')
            return
        self.get_logger().info(f"s: {s}")
        
        
        t = s
        # self.tf_buffer = Buffer()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_tag
        t.child_frame_id = self.frame_base

        self.tf_broadcaster.sendTransform(t)
        # self.get_logger().info(f"hey its here {t}")

        # if s and r:
        #     t = TransformStamped()
        #     self.tf_buffer = Buffer()
        #     t.header.stamp = self.get_clock().now().to_msg()
        #     t.header.frame_id = self.frame_tag
        #     t.child_frame_id = self.frame_base

        #     t.transform.translation.x = s.transform.translation.x
        #     t.transform.translation.y = s.transform.translation.y
        #     t.transform.translation.z = s.transform.translation.z
        #     q = quaternion_from_euler(s.transform.rotation.x, s.transform.rotation.y, s.transform.rotation.z)
        #     t.transform.rotation.x = q[0]
        #     t.transform.rotation.y = q[1]
        #     t.transform.rotation.z = q[2]
        #     t.transform.rotation.w = q[3]

        #     self.tf_broadcaster.sendTransform(t)
        #     self.get_logger().info(f"hey its here {t}")





def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    c = Calibrate()
    rclpy.spin(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()