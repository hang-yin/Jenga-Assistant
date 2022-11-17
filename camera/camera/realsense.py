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

        self.max_depth = 200
        self.min_depth = 0

        cv2.namedWindow('mask')
        cv2.createTrackbar('min depth', 'mask' , self.min_depth, 5000, self.min_depth_trackbar)
        cv2.createTrackbar('max depth', 'mask' , self.max_depth, 5000, self.max_depth_trackbar)

    def min_depth_trackbar(self, val):
        self.min_depth = val

    def max_depth_trackbar(self, val):
        self.max_depth = val

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
        # Index of largest element
        min_depth = 100
        max_depth = 500
        mask = cv2.inRange(current_frame, min_depth, max_depth)
        cv2.imshow("mask", mask)

        # blur = cv2.blur(current_frame,(5,5))
        # blur_colormap = cv2.applyColorMap(cv2.convertScaleAbs(blur, alpha=0.3), cv2.COLORMAP_JET)
        # cv2.imshow("blurry", blur_colormap)
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