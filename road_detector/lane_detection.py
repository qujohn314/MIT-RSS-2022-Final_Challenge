#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from city_driving.msg import LaneLines
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file


class LaneDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_lane_lines (LaneLines) : the coordinates of the line in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.lane_pub = rospy.Publisher("/relative_lane_lines", LaneLines, queue_size=10)
        self.debug_pub = rospy.Publisher("/lane_lines_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        # self.lane_lines_pub = rospy.Publisher("/zed/zed_node/rgb/image_bound_boxes", Image, queue_size=10)
        # self.lane_detect_pub_mask = rospy.Publisher("/zed/zed_node/rgb/image_mask",Image,queue_size=10)

    def calculate_slope(self, coordinates):
        if coordinates[0] - coordinates[2] == 0:
            slope = 1000
        else: 
            slope = (coordinates[1] - coordinates[3])/(coordinates[0] - coordinates[2])
        
        return slope

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
        horizontal_crop = 1000
        vertical_crop = 1500
        roi = src[vertical_crop:, horizontal_crop:horizontal_crop+1800]
        dst = cv2.Canny(roi, 50, 200, None, 3)
        
        # Copy edges to the images that will display the results in BGR
        cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
        # Finds Hough Lines
        # linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10) #og parameters
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 30)

        #Graph lines (optional) for debugging
        if linesP is not None:
            for i in range(0, len(linesP)):
                l = linesP[i][0]
                # NEED TO PUBLISH THIS TO ZED SOME WAY
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        

        #Return lines
        if linesP is None:
            rospy.loginfo("No lines detected!")
        else:
            return linesP, dst


    def get_lane(self, linesP, dst):
        """
        Gets the inside lane from the set of hough lines

        Parameters: 
            linesP (list) - list of hough lines
        Returns:
            inside_line (numpy array) - line that represents the lane [x1, y1, x2, y2]
        """
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            if abs(self.calculate_slope(l)) > 1: #gets rid of horizontal lines
                if l[1] > len(dst) - 100 or l[3] > len(dst) - 100:
                    if l[0] < len(dst[0])/2 or l[2] < len(dst[0])/2:
                        inside_lane = l
                        return inside_lane

        rospy.loginfo("Inside lane not detected")


    def image_callback(self, image_msg):
        """
        Takes in image message from camera and then publishes the inside lane in a LaneLines message
        """
        im = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        linesP, dst = self.get_hough_lines(im)
        inside_lane = self.get_lane(linesP, dst)

        lane_msg = LaneLines()
        lane_msg.x1 = inside_lane[0]
        lane_msg.y1 = inside_lane[1]
        lane_msg.x2 = inside_lane[2]
        lane_msg.y2 = inside_lane[3]

        self.lane_pub.publish(lane_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
