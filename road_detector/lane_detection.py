#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from computer_vision.color_segmentation import cd_color_segmentation
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

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
    self.lane_pub = rospy.Publisher("/relative_lane_lines", LaneLine, queue_size=10)
    self.debug_pub = rospy.Publisher("/lane_lines_img", Image, queue_size=10)
    self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
    self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
    self.lane_lines_pub = rospy.Publisher("/zed/zed_node/rgb/image_bound_boxes", Image, queue_size=10)
    self.lane_detect_pub_mask = rospy.Publisher("/zed/zed_node/rgb/image_mask",Image,queue_size=10)

def image_callback(self, image_msg):
    """
    """

    image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
	
	# img_width= image.shape[1]
	# img_height = image.shape[0]
	# large_cutoff = round(img_height/2)
	# small_section_cutoff = round(img_height*0.8)
	
	# for row in range(large_cutoff):
	# 	for column in range(img_width):
	# 		image[row][column] = np.array([0,0,0]) # convert to black pixel
	
	# for row in range(small_section_cutoff:img_height):
	# 	for column in range(img_width):
	# 		image[row][column] = np.array([0,0,0]) # convert to black pixels
	

    # Loads an image
    src = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    #Edge detection
    dst = cv2.Canny(src[0:], 50, 200, None, 3)
    debug_msg = self.bridge.cv2_to_imgmsg(dst, "bgr8")
    
    # Copy edges to the images that will display the results in BGR
    cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    #Hough Transform
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

	pixel_msg = ConeLocationPixel()
