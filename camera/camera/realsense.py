import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2
import sys
np.set_printoptions(threshold=sys.maxsize)

class Cam(Node):
    def __init__(self):
        super().__init__('cam')
        self.freq = 100.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        # self.color_sub = self.create_subscription(CompressedImage,
        #                                           "/camera/color/image_raw/compressed",
        #                                           self.color_callback,
        #                                           10)
        self.color_sub = self.create_subscription(Image,
                                                  "/camera/color/image_raw",
                                                  self.color_callback,
                                                  10)
        self.depth_sub = self.create_subscription(Image,
                                                  "/camera/depth/image_rect_raw",
                                                  self.depth_callback,
                                                  10)
        clipping_dist_meters = 0.25
        self.clipping_distance = 0.2
        self.br = CvBridge()

        self.color_frame = None
        self.depth_frame = None
        

    def color_callback(self, data):
        # print("COLOR CALLBACK")
        # height is 720
        # width = 1280
        # encoding is rgb8
        # is bigendian is 0
        # Step is 3840
        # Len of data.data array is 2764800
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.color_frame = current_frame
        cv2.imshow("why am I blue", current_frame)
        cv2.waitKey(1)

    def depth_callback(self, data):
        # print("DEPTH CALLBACK")
        # height is 720
        # width = 1280
        # encoding is 16UC1
        # is bigendian is 0
        # Step is 2560
        # Len of data.data array is 1843200
        # len/step = 720
        # print(np.reshape(data.data,(720,1280,2)))
        current_frame = self.br.imgmsg_to_cv2(data)
        self.depth_frame = current_frame
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(current_frame, alpha=0.3), cv2.COLORMAP_JET)
        cv2.imshow("im 13 and this is deep", depth_colormap)
        if self.color_frame is not None:
            depth_image_3d = np.dstack((current_frame,current_frame,current_frame))
            # print(depth_image_3d)
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), 153, self.color_frame)
            cv2.imshow("AAAAAAAH", bg_removed)
        cv2.waitKey(1)

    def timer_callback(self):
        pass


def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    c = Cam()
    rclpy.spin(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()