#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from city_driving.msg import LaneLines
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file
from visualization_tools import *

class HomographyTransformer():
    """
    """
    def __init__(self):
        self.PTS_IMAGE_PLANE = [[260, 226],
                   [455, 242],
                   [174, 289],
                   [494, 290]] # dummy points
        ######################################################

        # PTS_GROUND_PLANE units are in inches
        # car looks along positive x axis with positive y axis to left

        ######################################################
        ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
        self.PTS_GROUND_PLANE = [[54.5, 13.5],
                            [40.5, -12],
                            [24, 13.5],
                            [24.5, -9]] # dummy points
        ######################################################                   
        self.METERS_PER_INCH = 0.0254

         #Initialize data into a homography matrix

        np_pts_ground = np.array(self.PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * self.METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(self.PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.
        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y


class LaneDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_lane_lines (LaneLines) : the coordinates of the line in the image frame (units are pixels).
    """
    def __init__(self):
        self.vertical_crop = 200
        self.horizontal_crop = 0
        # toggle line follower vs cone parker
        self.LineFollower = False
        self.rate = rospy.Rate(10) #hz
        # Subscribe to ZED camera RGB frames
        self.lane_pub = rospy.Publisher("/relative_lane_lines", LaneLines, queue_size=1)
        # self.lane_px_pub = rospy.Publisher("/center_pixel", CenterPixel, queue_size=10)
        # self.line_pub_left = rospy.Publisher("/lane_lines_ground_left", Marker, queue_size=1)
        # self.line_pub_right = rospy.Publisher("/lane_lines_ground_right", Marker, queue_size=1)
        # self.look_ahead_point = rospy.Publisher("/look_ahead_ground", Marker, queue_size=1)
        self.debug_pub = rospy.Publisher("/zed/zed_node/rgb/lane_lines_img", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback, queue_size=1, buff_size=99999)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        # self.lane_lines_pub = rospy.Publisher("/zed/zed_node/rgb/image_bound_boxes", Image, queue_size=10)
        # self.lane_detect_pub_mask = rospy.Publisher("/zed/zed_node/rgb/image_mask",Image,queue_size=10)


    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False

    def calculate_slope(self, coordinates):

        if coordinates[0] - coordinates[2] == 0:
            slope = 1000
        else: 
            delta_y = coordinates[1] - coordinates[3]
            delta_x = coordinates[0] - coordinates[2]
            slope = float(delta_y)/float(delta_x)
        
        return slope

    def midpoint(self, x1, y1, x2, y2):
        return ((x1 + x2)/2, (y1 + y2)/2)


    def get_center_point(self, left_lane, right_lane):
        left_lane_equation = self.line((left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]))
        right_lane_equation = self.line((right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]))
        center_x, _ = self.intersection(left_lane_equation, right_lane_equation)

        # center_left = self.midpoint(left_lane[0], left_lane[1], left_lane[2], left_lane[3])
        # center_right = self.midpoint(right_lane[0], right_lane[1], right_lane[2], right_lane[3])
        # _, center_y = self.midpoint(center_left[0], center_left[1], center_right[0], center_right[1])
        # print(center_y)

        return (int(center_x)+20, 210) #DEBUGING


    def get_hough_lines(self, image):
        """
        Applies a mask, edge detection and hough transform to the roi of the image 

        Parameters:
            image (numpy array) - image ready in cv2 format

        Returns:
            linesP (numpy array) - array of lines found in form of (x1, y1, x2, y2)
        """
        # Create mask
        lower = np.array([180, 180, 180], dtype="uint8")
        upper = np.array([255, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)
        masked = cv2. bitwise_and(image, image, mask = mask)

        src = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Check if image is loaded fine
        if src is None:
            print ('Error opening image!')
            # print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
            return -1

        # Create ROI -- EDIT THIS for ZED camera!
        # width: 672 pixels
        # height: 376 pixels
        roi = src[self.vertical_crop: , self.horizontal_crop:] # 200
        dst = cv2.Canny(roi, 50, 200, None, 3)
        
        # Copy edges to the images that will display the results in BGR
        cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
        # Finds Hough Lines
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 30)

        return linesP, image
    

    def get_lane_center(self, linesP, src):
        """
        Gets the right and left lane from the set of hough lines

        Parameters: 
            linesP (list) - list of hough lines
        Returns:
            (center_x, center_y) (tuple) - center between two lanes
        """
        right_lanes = []
        left_lanes = []

        for i in range(0, len(linesP)):
            l = linesP[i][0]
            if 0.8 > self.calculate_slope(l) > 0.3:
                left_lanes.append(l)
                cv2.line(src, (l[0]+self.horizontal_crop, l[1]+self.vertical_crop), 
                (l[2]+self.horizontal_crop, l[3]+self.vertical_crop), (0,0,255), 3, cv2.LINE_AA)
            
            if -0.8 < self.calculate_slope(l) < -0.3: 
                right_lanes.append(l)
                cv2.line(src, (l[0]+self.horizontal_crop, l[1]+self.vertical_crop), 
                (l[2]+self.horizontal_crop, l[3]+self.vertical_crop), (0,0,255), 3, cv2.LINE_AA)

        left_lane = np.average(np.array(left_lanes), axis=0).astype(int)
        cv2.line(src, (left_lane[0]+self.horizontal_crop, left_lane[1]+self.vertical_crop), 
        (left_lane[2]+self.horizontal_crop, left_lane[3]+self.vertical_crop), (0,255,0), 3, cv2.LINE_AA)

        right_lane = np.average(np.array(right_lanes), axis=0).astype(int)
        cv2.line(src, (right_lane[0]+self.horizontal_crop, right_lane[1]+self.vertical_crop), 
        (right_lane[2]+self.horizontal_crop, right_lane[3]+self.vertical_crop), (0,255,0), 3, cv2.LINE_AA)


        center_x, center_y = self.get_center_point(left_lane, right_lane)
        # rospy.loginfo((center_x, center_y))
        cv2.circle(src, (center_x+self.horizontal_crop, center_y), 20, (255,0,0), -1) #adjust by crop

        transform = HomographyTransformer()

        # left_lane_x1_ground, left_lane_y1_ground = transform.transformUvToXy(left_lane[0], left_lane[1])
        # left_lane_x2_ground, left_lane_y2_ground = transform.transformUvToXy(left_lane[2], left_lane[3])
        # right_lane_x1_ground, right_lane_y1_ground = transform.transformUvToXy(right_lane[0], right_lane[1])
        # right_lane_x2_ground, right_lane_y2_ground = transform.transformUvToXy(left_lane[2], right_lane[3])

        # left_lane_xs = (left_lane_x1_ground, left_lane_x2_ground)
        # left_lane_ys = (left_lane_y1_ground, left_lane_y2_ground)
        # right_lane_xs = (right_lane_x1_ground, right_lane_x2_ground)
        # right_lane_ys = (right_lane_y1_ground, right_lane_y2_ground)
        # print(left_lane_xs)

        # VisualizationTools.plot_line(left_lane_xs, left_lane_ys, self.line_pub_left, frame="/base_frame")
        # VisualizationTools.plot_line(right_lane_xs, right_lane_ys, self.line_pub_right, frame="/base_frame")


        debug_msg = self.bridge.cv2_to_imgmsg(src, "passthrough")
        self.debug_pub.publish(debug_msg)

        # convert to ground plane
        x1_ground, y1_ground = transform.transformUvToXy(center_x+self.horizontal_crop, center_y) #plus vertical crop
        # VisualizationTools.plot_point(x1_ground, y1_ground, self.look_ahead_point, frame="/base_frame")

        #Return point
        if linesP is None:
            rospy.loginfo("No lines detected!")
        else:
            return x1_ground, y1_ground


    def image_callback(self, image_msg):
        """
        Takes in image message from camera and then publishes the inside lane in a LaneLines message
        """
        im = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        linesP, dst = self.get_hough_lines(im)
        center_x, center_y = self.get_lane_center(linesP, dst)
        print((center_x, center_y))
        
        lane_msg = LaneLines()
        lane_msg.x1 = center_x
        lane_msg.y1 = center_y
        lane_msg.x2 = 0
        lane_msg.y2 = 0

        self.lane_pub.publish(lane_msg)
        # self.rate.sleep() run rostopic hz topic_name
        

if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
