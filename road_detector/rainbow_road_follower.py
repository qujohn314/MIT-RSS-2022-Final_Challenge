#!/usr/bin/env python

import time
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs import Image

class RainbowRoadFollower:
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")

        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.road_detector_sub = rospy.Subscriber("/road_mask", Image, self.road_detector_callback)
    
    def road_detector_callback(self,img_msg):
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('rainbow_road_follower', anonymous=True)
        RainbowRoadFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
