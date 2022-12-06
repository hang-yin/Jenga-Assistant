import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
from enum import Enum, auto
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from math import sqrt, dist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, Int16
from keras.models import load_model
from ament_index_python.packages import get_package_share_path


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
        self.top_pub = self.create_publisher(Int16, 'top_size', 10)
        self.scan = self.create_service(Empty, "scan", self.scan_service_callback)
        self.stop = self.create_service(Empty, "stop", self.stop_service_callback)
        self.calib = self.create_service(Empty, "findtower", self.calib_service_callback)
        self.br = CvBridge()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.brick = TransformStamped()
        self.frame_brick = "brick"
        self.frame_camera ='camera_link'

        # These will be updated from my subscribers to camera data
        self.color_frame = None
        self.depth_frame = None
        self.intrinsics = None

        # For smoothing the image
        kernel_size = 25
        self.kernel = np.ones((kernel_size,kernel_size),np.uint8)

        # Bounding rectangle
        self.sq_orig = [415,0]
        self.sq_sz = 500
        self.rect = None
        self.update_rect()

        # Current state
        self.state = State.WAITING

        # depth "bands"
        self.band_start = 570
        self.band_width = 20
        self.tower_top = None
        self.table = None
        self.scan_start = 450
        self.scan_index = self.scan_start
        self.max_scan = 1000
        self.scan_step = 0.5

        # Contour area threshold for detecting the table
        self.table_area_threshold = 10000
        # Contour area threshold for detecting a piece sticking out
        self.piece_area_threshold = 1000
        # Little bit less than the area of 1 piece when it's on the top
        self.top_area_threshold = 16000

        self.piece_depth = 0.029 # 3 cm

        self.avg_sec = 3.0
        self.avg_frames = int(self.avg_sec*self.freq)
        self.ct = 0
        self.avg_piece = Pose()

        # For averaging (median-ing?) the coordinates of piece
        self.piece_x = []
        self.piece_y = []
        self.piece_z = []

        self.avg_area = 0

        # For storing the coordinates of the top of the tower
        self.starting_top = None
        self.current_top = None

        # cv2.namedWindow('Mask')
        # cv2.createTrackbar('kernel size', 'Mask', kernel_size, 100, self.kernel_trackbar)
        
        cv2.namedWindow('Color')
        
        cv2.createTrackbar('origin x', 'Color', self.sq_orig[0], 1000, self.sqx_trackbar)
        cv2.createTrackbar('origin y', 'Color' , self.sq_orig[1], 1000, self.sqy_trackbar)
        cv2.createTrackbar('size', 'Color' , self.sq_sz, 700, self.sqw_trackbar)
        cv2.createTrackbar('band width', 'Color' , self.band_width, 100, self.band_width_tb)
        cv2.createTrackbar('band start', 'Color' , self.band_start, 1000, self.band_start_tb)
        

        # load machine learning model
        model_path = get_package_share_path('camera') / 'keras_model.h5'
        label_path = get_package_share_path('camera') / 'labels.txt'
        self.model = load_model(model_path)
        self.labels = open(label_path, 'r').readlines()

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
    
    def piece_found_cb(self, d_):
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

    def get_mask(self, care_about_square = True, get_lines = False):
        
        # Do this in case the subscriber somehow updates in the middle of the function?
        depth_cpy = np.array(self.depth_frame)
        # Only keep stuff that's within the appropriate depth band.
        depth_mask = cv2.inRange(np.array(depth_cpy), self.band_start, self.band_start+self.band_width)
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
        # Find the contours of this cropped mask to help locate tower.
        contours, _ = cv2.findContours(depth_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids, areas, large_contours = [], [], []
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
        largest_area, centroid_pose, = None, None
        max_centroid, box, box_area = None, None, None
        if len(areas) != 0: 
            # There is something large in the image.
            largest_index = np.argmax(areas)
            largest_area = areas[largest_index]
            # self.get_logger().info(f"LARGEST AREA: {largest_area}")
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

            min_rect = cv2.minAreaRect(max_contour)
            box = cv2.boxPoints(min_rect)
            box = np.intp(box)
            # Save original box area to test if the contour is a good fit
            box_area = dist(box[0],box[1])*dist(box[1],box[2])
        
        # Display the images
        color_rect = cv2.rectangle(np.array(self.color_frame),
                                   self.rect[0][0], self.rect[0][2],
                                   (255,0,0), 2)
        

        drawn_contours = cv2.drawContours(color_rect, large_contours, -1, (0,255,0), 3)
        if max_centroid is not None:
            drawn_contours = cv2.circle(drawn_contours, max_centroid, 5, (0,0,255), 5)
            drawn_contours = cv2.drawContours(drawn_contours, [box], 0, (255,0,0), 3)
            contour_ratio = largest_area/box_area
            # self.get_logger().info(f"CONTOUR RATIO: {contour_ratio}")
            if care_about_square and (contour_ratio<0.7):
                # The contour is not really a rectangle and therefore doesn't work well
                largest_area, centroid_pose = None, None

        cv2.imshow('Color', drawn_contours)

        cv2.waitKey(1)
        return largest_area, centroid_pose

    def publish_top(self):
        if self.starting_top is not None:
            self.starting_top.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.starting_top)
        if self.current_top is not None:
            self.current_top.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(self.current_top)

    def timer_callback(self):

        # Every time you need to publish the top of the tower
        self.publish_top()

        # process color_frame with ML model
        if self.color_frame is not None:
            image = cv2.resize(self.color_frame, (224, 224), interpolation=cv2.INTER_AREA)
            image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
            image = (image / 127.5) - 1
            probabilities = self.model.predict(image)
            # TODO: if argmax(probabilities) is consistently equal to "block ready" for a while
            # then we can call /scan to grab the block
            self.get_logger().info(self.labels[np.argmax(probabilities)])

        if self.state == State.WAITING:
            # Pause while we wait for frames
            self.get_logger().info("Waiting for frames...")
            wait_for = [self.intrinsics, self.depth_frame, self.color_frame]
            if all(w is not None for w in wait_for):
                self.get_logger().info("Searching for tower top!!")
                self.state = State.FINDTOP

        elif self.state == State.PAUSED:
            # Just print out the camera data
            largest_area, _ = self.get_mask()

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

            largest_area, centroid_pose = self.get_mask()
            if largest_area:
                # self.get_logger().info(f"Largest area {largest_area}")
                if largest_area > self.top_area_threshold:
                    if self.ct < self.avg_frames:
                        # self.get_logger().info(f'Current pose: {centroid_pose.position}')
                        # Record the position
                        self.piece_x.append(centroid_pose.position.x)
                        self.piece_y.append(centroid_pose.position.y)
                        self.piece_z.append(centroid_pose.position.z)
                        self.ct += 1
                        # Stay at same scan level
                        self.scan_index -= self.scan_step
                    else:
                        self.ct = 0
                        avg_x = np.median(self.piece_x)
                        avg_y = np.median(self.piece_y)
                        avg_z = np.median(self.piece_z)
                        self.piece_x = []
                        self.piece_y = []
                        self.piece_z = []
                        self.get_logger().info("FOUND TOWER TOP!!!!!!")
                        self.get_logger().info(f"depth: {self.band_start+self.band_width},"+
                                            f"area: {largest_area}\n")
                        self.get_logger().info(f"Centroid pose in cam frame:\n{avg_x},{avg_y},{avg_z}")
                        self.tower_top = self.band_start+self.band_width
                        # This will begin publishing to the tf tree
                        if self.starting_top is None:
                            self.starting_top = TransformStamped()
                            self.starting_top.header.frame_id = self.frame_camera
                            self.starting_top.child_frame_id = 'starting_top'
                            self.starting_top.transform.translation.x = avg_x
                            self.starting_top.transform.translation.y = avg_y
                            self.starting_top.transform.translation.z = avg_z
                        else:
                            self.current_top = TransformStamped()
                            self.current_top.header.frame_id = self.frame_camera
                            self.current_top.child_frame_id = 'current_top'
                            self.current_top.transform.translation.x = avg_x
                            self.current_top.transform.translation.y = avg_y
                            self.current_top.transform.translation.z = avg_z
                        # Go down past the top pieces, or else this will also be detected as table.
                        # UNCOMMENT THESE LATER
                        self.scan_index = self.tower_top + self.band_width
                        self.band_start = self.tower_top + self.band_width
                        num_pieces = Int16()
                        if largest_area > 3*self.top_area_threshold:
                            self.get_logger().info("Think 3 pieces on top")
                            num_pieces.data = 3
                        elif largest_area > 2*self.top_area_threshold:
                            self.get_logger().info("Think 2 pieces on top")
                            num_pieces.data = 2
                        else:
                            self.get_logger().info("Think 1 piece on top")
                            num_pieces.data = 1
                        # Go and find the table
                        self.top_pub.publish(num_pieces)
                        self.state = State.FINDTABLE
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
            largest_area, _ = self.get_mask(care_about_square=False)
            if largest_area:
                if largest_area > self.table_area_threshold:
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
            largest_area, centroid_pose = self.get_mask()
            if largest_area:
                if largest_area > self.piece_area_threshold:
                    if self.ct < self.avg_frames:
                        # self.get_logger().info(f'Current pose: {centroid_pose.position}')
                        self.piece_x.append(centroid_pose.position.x)
                        self.piece_y.append(centroid_pose.position.y)
                        self.piece_z.append(centroid_pose.position.z)
                        self.ct += 1
                        # Stay at same scan level
                        self.scan_index -= self.scan_step
                    else:
                        # take median to avoid weird jumping behavior
                        self.avg_piece.position.x = np.median(self.piece_x)
                        self.avg_piece.position.y = np.median(self.piece_y)
                        self.avg_piece.position.z = np.median(self.piece_z)
                        self.piece_x = []
                        self.piece_y = []
                        self.piece_z = []
                        self.avg_piece.position.z += self.piece_depth/2.
                        self.get_logger().info("Done averaging!")
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
            # Continue to publish camera image
            _, _ = self.get_mask()
            # Create tf between the tag and the rotated frame using avg_piece
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