#!/usr/bin/env python
import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from computer_vision.color_segmentation import cd_color_segmentation
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from city_driving.msg import CarWashPixel

#from os import path
#import sys
#sys.path.append(path.abspath('../../'))

from Project1.file1 import something
class CarWashDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # Subscribe to ZED camera RGB frames
        self.car_wash_pub = rospy.Publisher("/relative_car_wash_px", CarWashPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/car_wash_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################


        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")


        #Change HSV ranges to match blue of car wash
        car_wash_bounding_box = cd_color_segmentation(image,image_format=1,low_range=[0,0,0],high_range=[0,0,0])
        pixel_msg = CarWashPixel()

        if not car_wash_bounding_box:
            # rospy.loginfo("No car wash detected")
            pixel_msg.u = -100000
            pixel_msg.v = -100000
        else:
            x1 = cone_bounding_box[0][0]
            y1 = cone_bounding_box[0][1]
            x2 = cone_bounding_box[1][0]
            y2 = cone_bounding_box[1][1]
            pixel_msg.u = x1 + (x2-x1)/2
            pixel_msg.v = y2

        self.debug_pub.publish(debug_msg)
        self.car_wash_pub(pixel_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('car_wash_detector', anonymous=True)
        CarWashDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
