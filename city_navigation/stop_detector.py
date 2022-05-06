#!/usr/bin/env python
import cv2
import rospy
import sys

import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


from ackermann_msgs.msg import AckermannDriveStamped

from homography_transformer import HomographyTransformer

class SignDetector:
    def __init__(self):
        self.stopping_distance = 2.0 #Should stop between 0.75-1meter away
        self.stopping_buffer = 0 # if we need more of a buffer in order to stop 
        self.pole_sign_ratio = 1.1
        self.homography_transformer = HomographyTransformer()

        self.publisher = rospy.Publisher("/should_stop", Bool, queue_size=10)
        self.subscriber = rospy.Subscriber("stop_sign_bbox", Float32MultiArray, self.callback)
        

    def callback(self, msg):
        rospy.loginfo("SignDetector Running")
        ret = Bool()
        ret.data = False

        # Process image without CV Bridge

        bounding_box = msg.data
        if bounding_box is not None and len(bounding_box) == 4: 
            rospy.loginfo("ML sees a stop sign! Is it within range?")
            # calculated distance to the stop sign, if within stopping distance publish bool command
            xmin, ymin, xmax, ymax = bounding_box
            sign_height = ymax - ymin

            base_v = ymax + sign_height * self.pole_sign_ratio
            base_u = (xmax - xmin)//2

            x, y = self.homography_transformer.transformUvToXy(base_u, base_v)
           
            # FOR NOW: distance from the robot's camera is the x value 
            if x <= self.stopping_distance + self.stopping_buffer:
                rospy.loginfo("Stop sign is in range! " + str(x))
                ret.data = True 
        self.publisher.publish(ret)

if __name__=="__main__":
    rospy.init_node("stop_detector_listener")
    detect = SignDetector()
    rospy.spin()
