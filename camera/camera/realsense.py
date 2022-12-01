import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
# np.set_printoptions(threshold=sys.maxsize)
import pyrealsense2 as rs2
from enum import Enum, auto
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from math import sqrt, sin, cos
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from std_msgs.msg import Bool
from math import dist


def angle_axis_to_quaternion(theta, axis):
    """
    Convert from angle-axis of rotation to a quaternion.
    This is taken from the tf activity we did in class.
    Args:
        theta:  rotation angle, in radians
        axis: the rotational axis. This will be normalized
    Returns
    -------
    A Quaternion corresponding to the rotation
    """
    magnitude = sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
    normalized = [v/magnitude for v in axis]
    sinTheta2 = sin(theta/2.0)
    return Quaternion(x=normalized[0]*sinTheta2,
                      y=normalized[1]*sinTheta2,
                      z=normalized[2]*sinTheta2,
                      w=cos(theta/2.0))

class State(Enum):
    """The current state of the brick."""

    # Waiting to get frames
    WAITING = auto(),
    # Initial scan to find the top of the tower
    FINDTOP = auto(),
    # Initial scan to find the bottom of the tower
    FINDTABLE = auto(),
    # Scanning to find a piece that's pushed out
    SCANNING = auto(),
    # Publish frame of found jenga piece
    PUBLISHPIECE = auto(),
    # Don't scan but still show the screen
    PAUSED = auto()

class Cam(Node):
    def __init__(self):
        super().__init__('cam')
        self.freq = 60.
        self.timer = self.create_timer(1./self.freq, self.timer_callback)

        self.color_sub = self.create_subscription(Image,
                                                  "/camera/color/image_raw",
                                                  self.color_callback,
                                                  10)
        self.depth_sub = self.create_subscription(Image,
                                                  "/camera/aligned_depth_to_color/image_raw",
                                                  self.depth_callback,
                                                  10)
        self.info_sub = self.create_subscription(CameraInfo,
                                                 "/camera/aligned_depth_to_color/camera_info",
                                                 self.info_callback,
                                                 10)
        self.piece_found_sub = self.create_subscription(Bool,
                                                        'piece_found',
                                                        self.piece_found_cb,
                                                        10)

        self.piece_pub = self.create_publisher(Pose, 'jenga_piece', 10)
        self.corners_pub = self.create_publisher(PoseArray, 'jenga_corners', 10)
        self.scan = self.create_service(Empty, "scan", self.scan_service_callback)
        self.stop = self.create_service(Empty, "stop", self.stop_service_callback)
        self.calib = self.create_service(Empty, "findtower", self.calib_service_callback)
        self.br = CvBridge()

        # Initialize the transform broadcaster
        # Static bc block only gets published once!
        self.tf_broadcaster = TransformBroadcaster(self)
        self.brick = TransformStamped()
        self.frame_brick = "brick"
        self.frame_camera ='camera_link'

        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        self.band_start = 570
        self.band_width = 20

        kernel_size = 25
        self.kernel = np.ones((kernel_size,kernel_size),np.uint8)

        self.sq_orig = [450,0]
        self.sq_sz = 450

        self.rect = None
        self.update_rect()

        self.state = State.WAITING

        self.tower_top = None
        self.table = None
        self.scan_start = 450
        self.scan_index = self.scan_start
        self.max_scan = 1000
        self.scan_step = 0.5

        self.corner_threshold = 0.01
        self.corner_scale = 100

        # How large a contour is to consider it as relevant
        self.object_area_threshold = 10000
        self.piece_area_threshold = 2000
        # Size of about 1 piece when it's on the top
        self.top_area_threshold = 16000

        self.piece_depth = 0.029 # 3 cm

        self.avg_sec = 3.0
        self.avg_frames = int(self.avg_sec*self.freq)
        self.ct = 0
        self.avg_piece = Pose()

        self.piece_x = []
        self.piece_y = []
        self.piece_z = []

        # cv2.namedWindow('Mask')
        # cv2.createTrackbar('kernel size', 'Mask', kernel_size, 100, self.kernel_trackbar)
        
        cv2.namedWindow('Color')
        cv2.createTrackbar('origin x', 'Color', self.sq_orig[0], 1000, self.sqx_trackbar)
        cv2.createTrackbar('origin y', 'Color' , self.sq_orig[1], 1000, self.sqy_trackbar)
        cv2.createTrackbar('size', 'Color' , self.sq_sz, 700, self.sqw_trackbar)
        cv2.createTrackbar('band width', 'Color' , self.band_width, 100, self.band_width_tb)
        cv2.createTrackbar('band start', 'Color' , self.band_start, 1000, self.band_start_tb)

        # cv2.namedWindow('Corners')
        # cv2.createTrackbar('corner threshold*100', 'Corners',
        #                    int(self.corner_threshold*self.corner_scale),
        #                    self.corner_scale, self.corner_trackbar)

    def sqx_trackbar(self, val):
        self.sq_orig[0] = val
        self.update_rect()

    def sqy_trackbar(self, val):
        self.sq_orig[1] = val
        self.update_rect()

    def sqw_trackbar(self, val):
        self.sq_sz = val
        self.update_rect()

    def update_rect(self):
        self.rect = np.array([[self.sq_orig,
                               [self.sq_orig[0]+self.sq_sz, self.sq_orig[1]],
                               [self.sq_orig[0]+self.sq_sz, self.sq_orig[1]+self.sq_sz],
                               [self.sq_orig[0], self.sq_orig[1]+self.sq_sz]]],
                               dtype=np.int32)

    def band_width_tb(self, val):
        self.band_width = val

    def band_start_tb(self, val):
        self.band_start = val

    def kernel_trackbar(self, val):
        self.kernel = np.ones((val,val),np.uint8)

    def corner_trackbar(self, val):
        self.corner_threshold = val/self.corner_scale

    def scan_service_callback(self, _, response):
        """ Make the camera scan for pieces sticking out """
        # Only valid if the tower and table have a height
        if self.tower_top and self.table:
            self.scan_index = self.tower_top +1.2*self.band_width
            self.state = State.SCANNING
            self.get_logger().info("Begin Scanning for pieces")
        else:
            self.get_logger().info("You have to call /findtower first before scanning!!!")
        return response

    def stop_service_callback(self, _, response):
        """ Stop conitnously scanning """
        self.state = State.PAUSED
        self.get_logger().info("Pause Scanning")
        return response
    
    def piece_found_cb(self, data):
        # Stop publishing tf data!!
        self.get_logger().info('Brick found')
        self.state = State.PAUSED

    def calib_service_callback(self, _, response):
        """ Re caluclate the height of the tower """
        self.state = State.FINDTOP
        self.scan_index = self.scan_start
        self.get_logger().info("Searching for tower top")
        return response

    def info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            self.get_logger().info("Getting intrinsics failed?")
            return

    def color_callback(self, data):
        color_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.color_frame = color_frame

    def depth_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        self.depth_frame = current_frame

        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(current_frame, alpha=0.3), cv2.COLORMAP_JET)
        # cv2.imshow("im 13 and this is deep", depth_colormap)

    def get_mask(self, care_about_square = True):
        # Do this in case the subscriber somehow updates in the middle of the function?
        depth_cpy = np.array(self.depth_frame)
        # self.get_logger().info(f"SHAPE: {depth_cpy.shape}")

        # Only keep stuff that's within the appropriate depth band.
        depth_mask = cv2.inRange(depth_cpy, self.band_start, self.band_start+self.band_width)
        # This operation helps to remove "dots" on the depth image.
        # Kernel higher dimensional = smoother. It's also less important if camera is farther away.
        depth_mask = cv2.morphologyEx(depth_mask, cv2.MORPH_CLOSE, self.kernel)

        # All 0s, useful for following bitwise operations.
        bounding_mask = np.zeros((self.intrinsics.height,self.intrinsics.width), np.int8)
        # Creating a square over the area defined in self.rect
        square = cv2.fillPoly(bounding_mask, [self.rect], 255)
        # Blacking out everything that is not within square
        square = cv2.inRange(square, 1, 255)
        # Cropping the depth_mask so that only what is within the square remains.
        depth_mask = cv2.bitwise_and(depth_mask, depth_mask, mask=square)
        # Draw a rectangle on a copy of the depth_mask to visualize where it is.
        # rectangle = cv2.rectangle(np.array(depth_mask),
        #                           self.rect[0][0], self.rect[0][2],
        #                           (255,0,0), 2)
        # cv2.imshow("Mask", rectangle)

        # Find the contours of this cropped mask to help locate tower.
        contours, _ = cv2.findContours(depth_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        areas = []
        large_contours = []
        # self.get_logger().info(f"Number of countours: {len(contours)}")
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
        max_centroid = None
        largest_area = None
        box_area = None
        centroid_pose = None
        box = None
        corner_array = None
        if len(areas) != 0: 
            # There is something large in the image.
            largest_index = np.argmax(areas)
            largest_area = areas[largest_index]
            max_centroid = centroids[largest_index]
            max_contour = large_contours[largest_index]
            centroid_depth = depth_cpy[max_centroid[1]][max_centroid[0]]
            centroid_deprojected = rs2.rs2_deproject_pixel_to_point(self.intrinsics,
                                                                    [max_centroid[0],
                                                                     max_centroid[1]],
                                                                    centroid_depth)
            centroid_pose = Pose()
            centroid_pose.position.x = centroid_deprojected[0]/1000.
            centroid_pose.position.y = centroid_deprojected[1]/1000.
            centroid_pose.position.z = centroid_deprojected[2]/1000.

            # Try finding the corners of the largest thing
            box, box_area, corner_array = self.get_box(max_contour, depth_cpy)

        # Display the images
        color_rect = cv2.rectangle(np.array(self.color_frame),
                                   self.rect[0][0], self.rect[0][2],
                                   (255,0,0), 2)
        drawn_contours = cv2.drawContours(color_rect, large_contours, -1, (0,255,0), 3)
        if max_centroid is not None:
            drawn_contours = cv2.circle(drawn_contours, max_centroid, 5, (0,0,255), 5)
            drawn_contours = cv2.drawContours(drawn_contours, [box], 0, (255,0,0), 3)
            if care_about_square and (largest_area/box_area < 0.9):
                    # The contour is not really a rectangle and therefore doesn't work well
                    largest_area = None
                    centroid_pose = None
                    corner_array = None
        cv2.imshow('Color', drawn_contours)

        cv2.waitKey(1)
        return largest_area, centroid_pose, corner_array

    def get_box(self, max_contour, depth_frame):
        min_rect = cv2.minAreaRect(max_contour)
        box = cv2.boxPoints(min_rect)
        box = np.intp(box)
        box_area = dist(box[0],box[1])*dist(box[1],box[2])
        # self.get_logger().info(f"Box coords: {box}")
        # dy = box[1][1]-box[0][1]
        # dx = box[1][0]-box[0][0]
        # self.get_logger().info(f"dy: {dy}, dx: {dx}")
        # angle = np.arctan2(dy,dx)
        # self.get_logger().info(f"Angle: {angle}")
        # centroid_pose.orientation = angle_axis_to_quaternion(angle, [0, 1, 0])
        corner_array = PoseArray()
        corner_array.header.stamp = self.get_clock().now().to_msg()
        for corner in box:
            corner_depth = depth_frame[corner[1]][corner[0]]
            # self.get_logger().info(f"Corner: {corner_depth}")
            corner_deprojected = rs2.rs2_deproject_pixel_to_point(self.intrinsics,
                                                                    corner,
                                                                    corner_depth)
            # self.get_logger().info(f"DEPROJECTED Corner:{corner_deprojected}")
            cornerPose = Pose()
            cornerPose.position.x = corner_deprojected[0]/1000.
            cornerPose.position.y = corner_deprojected[1]/1000.
            cornerPose.position.z = corner_deprojected[2]/1000.
            corner_array.poses.append(cornerPose)
        # self.get_logger().info(f"CORNER ARRAY: {corner_array}")
        return box, box_area, corner_array

    def timer_callback(self):
        if self.state == State.WAITING:
            self.get_logger().info("Waiting for frames...")
            wait_for = [self.intrinsics, self.depth_frame, self.color_frame]
            if all(w is not None for w in wait_for):
                self.state = State.PAUSED

        elif self.state == State.PAUSED:
            largest_area, _, _ = self.get_mask()

        elif self.state == State.FINDTOP:
            # Begin scanning downwards.
            self.band_start = self.scan_index
            self.scan_index += self.scan_step
            # Reset scan if too big
            if self.scan_index > self.max_scan:
                # This should not happen. But if it doesn't find anything large in the band:
                self.scan_index = self.scan_start
                self.get_logger().info("Didn't find the tower?")
                self.state = State.PAUSED

            largest_area, _, corner_array = self.get_mask()
            if largest_area:
                if largest_area > self.top_area_threshold:
                    # We believe there is an object at this depth
                    self.get_logger().info("FOUND TOWER TOP!!!!!!")
                    self.get_logger().info(f"depth: {self.band_start+self.band_width},"+
                                           f"area: {largest_area}\n")
                    self.tower_top = self.band_start+self.band_width
                    for c in corner_array.poses:
                        self.get_logger().info(f"Corner:{c.position}")
                    # Go down past the top pieces, or else this will also be detected as table.
                    # UNCOMMENT THESE LATER
                    # self.scan_index = self.tower_top + self.band_width
                    # self.band_start = self.tower_top + self.band_width
                    if largest_area > 3*self.top_area_threshold:
                        self.get_logger().info("Think 3 pieces on top")
                    elif largest_area > 2*self.top_area_threshold:
                        self.get_logger().info("Think 2 pieces on top")
                    else:
                        self.get_logger().info("Think 1 piece on top")
                    # Go and find the table
                    self.state = State.PAUSED
                    self.get_logger().info("Searching for table PAUSE")

        elif self.state == State.FINDTABLE:
            # Basically same logic as findtop.
            # Keep scanning downwards
            self.band_start = self.scan_index
            self.scan_index += self.scan_step
            # Reset scan if too big.
            if self.scan_index > self.max_scan:
                # This should not happen. But if it doesn't find anything large in the band:
                self.scan_index = self.scan_start
                self.get_logger().info("Didn't find the table?")
                self.state = State.PAUSED

            # The contour of the table will not be a square.
            largest_area, _, _ = self.get_mask(care_about_square=False)
            if largest_area:
                if largest_area > self.object_area_threshold:
                    # We believe there is an object at this depth
                    self.get_logger().info("FOUND TABLE!!!!!!")
                    self.get_logger().info(f"depth: {self.band_start+self.band_width},"+
                                           f"area: {largest_area}\n")
                    self.table = self.band_start
                    self.scan_index = self.tower_top + self.band_width
                    self.band_start = self.tower_top + self.band_width
                    self.state = State.PAUSED
        elif self.state == State.SCANNING:
            # Keep scanning downwards
            self.band_start = self.scan_index
            self.scan_index += self.scan_step
            # Reset scan if too big.
            if self.scan_index+1.2*self.band_width > self.table:
                self.scan_index = self.tower_top +1.2*self.band_width
                self.get_logger().info("Reset scan")
            # Look for piece sticking out in range from top to table
            largest_area, centroid_pose, _ = self.get_mask()
            if largest_area:
                if largest_area > self.piece_area_threshold:
                    if self.ct < self.avg_frames:
                        self.get_logger().info(f'Current pose: {centroid_pose.position}')
                        # self.get_logger().info("Piece (?) detected")
                        # self.get_logger().info(f"Depth: {self.band_start}, area: {largest_area}")
                        # self.get_logger().info(f"Pose in camera frame: {centroid_pose}")
                        # Avg the pose
                        self.piece_x.append(centroid_pose.position.x)
                        self.piece_y.append(centroid_pose.position.y)
                        self.piece_z.append(centroid_pose.position.z)
                        # self.avg_piece.position.x += centroid_pose.position.x
                        # self.avg_piece.position.y += centroid_pose.position.y
                        # self.avg_piece.position.z += centroid_pose.position.z
                        self.ct += 1
                        # Stay at same scan level
                        self.scan_index -= self.scan_step
                    else:
                        # self.avg_piece.position.x /= float(self.avg_frames)
                        # self.avg_piece.position.y /= float(self.avg_frames)
                        # self.avg_piece.position.z /= float(self.avg_frames)
                        self.avg_piece.position.x = np.median(self.piece_x)
                        self.avg_piece.position.y = np.median(self.piece_y)
                        self.avg_piece.position.z = np.median(self.piece_z)
                        self.piece_x = []
                        self.piece_y = []
                        self.piece_z = []
                        self.avg_piece.position.z += self.piece_depth/2.
                        self.get_logger().info("Done averaging")
                        self.get_logger().info(f"Final pose: {self.avg_piece}")
                        self.ct = 0
                        self.piece_pub.publish(self.avg_piece)
                        # This used to be centroid_pose now it is the averaged centroid pose
                        self.brick.transform.translation.x = self.avg_piece.position.x
                        self.brick.transform.translation.y = self.avg_piece.position.y
                        self.brick.transform.translation.z = self.avg_piece.position.z
                        # calculate the rotations with quaternians
                        self.brick.transform.rotation.x = self.avg_piece.orientation.x
                        self.brick.transform.rotation.y = self.avg_piece.orientation.y
                        self.brick.transform.rotation.z = self.avg_piece.orientation.z
                        self.brick.transform.rotation.w = self.avg_piece.orientation.w
                        # self.avg_piece = Pose()
                        self.state = State.PUBLISHPIECE
        elif self.state == State.PUBLISHPIECE:
            # self.get_logger().info('PUBLISH TFS!')
            # Continue to publish camera image
            _,_,_ = self.get_mask()
            #create tf between the tag and the rotated frame using avg_piece
            self.brick.header.stamp = self.get_clock().now().to_msg()
            self.brick.header.frame_id = self.frame_camera
            self.brick.child_frame_id = self.frame_brick
            self.tf_broadcaster.sendTransform(self.brick)






def main(args=None):
    """Start and spin the node."""
    rclpy.init(args=args)
    c = Cam()
    rclpy.spin(c)
    rclpy.shutdown()


if __name__ == '__main__':
    main()