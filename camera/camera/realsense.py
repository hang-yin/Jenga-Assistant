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
                                                  "/camera/aligned_depth_to_color/image_raw",
                                                  self.depth_callback,
                                                  10)
        self.br = CvBridge()

        self.color_frame = None
        self.depth_frame = None
<<<<<<< HEAD

        self.band_start = 100
        self.band_width = 50

        kernel_size = 50

        self.kernel = np.ones((kernel_size,kernel_size),np.uint8)

        cv2.namedWindow('mask')
        cv2.createTrackbar('band width', 'mask' , self.band_width, 100, self.band_width_tb)
        cv2.createTrackbar('band start', 'mask' , self.band_start, 1000, self.band_start_tb)

        cv2.namedWindow('Closed for business')
        cv2.createTrackbar('kernel size', 'Closed for business', kernel_size, 100, self.kernel_trackbar)

    def band_width_tb(self, val):
        self.band_width = val

    def band_start_tb(self, val):
        self.band_start = val

    def kernel_trackbar(self, val):
        self.kernel = np.ones((val,val),np.uint8)
=======
        
        self.max_depth = 200
        self.min_depth = 0

        cv2.namedWindow('mask')
        cv2.createTrackbar('min depth', 'mask' , self.min_depth, 5000, self.min_depth_trackbar)
        cv2.createTrackbar('max depth', 'mask' , self.max_depth, 5000, self.max_depth_trackbar)

    def min_depth_trackbar(self, val):
        self.min_depth = val

    def max_depth_trackbar(self, val):
        self.max_depth = val
>>>>>>> a5f2d50 (i did something)

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
        # cv2.imshow("why am I blue", current_frame)
        # cv2.waitKey(1)

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
        # cv2.imshow("im 13 and this is deep", depth_colormap)
        # Index of largest element
        mask = cv2.inRange(current_frame, self.band_start, self.band_start+self.band_width)
        cv2.imshow("mask", mask)

        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        cv2.imshow("Closed for business", closing)

        contours, _ = cv2.findContours(closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        areas = []
        large_contours = []
        max_centroid = None
        print(f"Number of countours: {len(contours)}")
        for c in contours: 
            M = cv2.moments(c)
            area = cv2.contourArea(c)
            try: 
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroid = (cx,cy)
                if area > 100:
                    centroids.append(centroid)
                    areas.append(area)
                    large_contours.append(c)
            except: 
                pass
        print(areas)
        if len(areas) != 0: 
            largest_index = np.argmax(areas)
            max_centroid = centroids[largest_index]
            print(f"estimated center: {max_centroid}")
            centroid_depth = current_frame[max_centroid[1]][max_centroid[0]]
            print(f"depth: {centroid_depth}\n")


        if self.color_frame is not None:
<<<<<<< HEAD
            drawn_contours = cv2.drawContours(self.color_frame, large_contours, -1, (0,255,0), 3)
            if max_centroid is not None:
                drawn_contours = cv2.circle(drawn_contours, max_centroid, 5, [0,0,255], 5)
            cv2.imshow("COUNTOURS", drawn_contours)

=======
            depth_image_3d = np.dstack((current_frame,current_frame,current_frame))
            print("yikes")
            # print(depth_image_3d)
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), 153, self.color_frame)
            mask = cv2.inRange(current_frame, self.min_depth,self.max_depth)
            print(self.min_depth)
            print(f'max depth: {self.max_depth}')
            # res = cv2.bitwise_and(bg_removed, bg_removed, mask = mask)

            cv2.imshow('mask', mask)
            # cv2.imshow('res', res)

            # """contour using mask"""
            #imgray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            #ret, thresh = cv2.threashold(imgray, 127,255,0)
            # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # drawcontours = cv2.drawContours(bg_removed, contours, -1, (0,225,0), 3)
            # cv2.imshow("AAAAAAAH", bg_removed)
>>>>>>> a5f2d50 (i did something)
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