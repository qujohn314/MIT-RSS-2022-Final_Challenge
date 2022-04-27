#!/usr/bin/env python2

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from lane_detection.msg import LaneLines

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class LaneFollower:
    LANE_TOPIC = rospy.get_param("/relative_lane_lines")
    DRIVE_TOPIC = rospy.get_param("drive_topic")
    SIDE = 1 # +1 represents left, -1 represents right
    VELOCITY = 2.0
    DESIRED_DISTANCE = 0.3 # manually adjusted (track lanes are 1.22m wide)

    TURNING_RADIUS = 0.325/np.tan(0.34)
    FRONT_BUFFER = 0.7 # car needs 0.62m to turn when wall is in front, added .7 to be safe
    #FRONT_BUFFER = 1
    previous_error = 0
    img_height = 100 #placeholder
    img_width = 100 #placeholder

    # more conservative
    # k_p = 5
    # k_d = 3
    k_p = 0.8
    k_d = 0.2

    def __init__(self):
        rospy.init_node('lane_follower')
        self.lane_subscriber = rospy.Subscriber(self.LANE_TOPIC, LaserScan, self.laser_callback)
        self.steering_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def lane_callback(self, msg):
        side_range = self.get_side_range(msg)
        steering_angle = self.find_new_steering_angle(side_range)
        
        steering_msg = AckermannDriveStamped()
        steering_msg.header.stamp = rospy.Time.now()
        steering_msg.header.frame_id = 'base_link' 
        steering_msg.drive.steering_angle = steering_angle
        #steering_msg.drive.steering_angle = 0
        steering_msg.drive.speed = self.VELOCITY

        self.steering_publisher.publish(steering_msg)
        #self.error_publisher.publish(error)

    def find_new_steering_angle(self, side_range):
        if len(side_range) < 2:
            return 0.0

        dust_idx = np.where(np.sqrt(side_range[:, 0]**2 + side_range[:, 1]**2) <= 0.1)
        weights = 1/(side_range[:, 0]**2 + side_range[:,1]**2)**3
        weights[dust_idx] = 0
        m,b = np.polyfit(side_range[:, 0], side_range[:,1], 1)#, w=weights)#, w=weights) #equation of wall
        y_line = side_range[:, 0]*m + b
        self.visualize.plot_line(side_range[:, 0], y_line, self.line_pub, frame="/laser")

        # closest distance from robot (0, 0) to wall 
        closest_distance = np.abs(b) / np.sqrt((m**2) + 1.0)

        # use a pd controller to find steering angle
        error = closest_distance - self.DESIRED_DISTANCE
        derivative = (error - self.previous_error)
        steering_angle = np.arctan(self.k_p * error +
                                self.k_d * derivative) * self.SIDE + np.arctan(m)
        self.previous_error = error
        # self.error_publisher.publish(error)
        return steering_angle
        

    def get_side_range(self, data):

        data_range = []
        # convert this operation to numpy
        for x,y in data:
            new_x = x - img_width/2.0
            new_y = img_height - y
        
            scaled_x = new_x * pixel_to_meters
            scaled_y = new_y * pixel_to_meters

            data_range.append((scaled_x, scaled_y))
        
        
        # ranges = np.array(data.ranges)
        # r_theta_ranges = np.array([np.array([ranges[i], data.angle_min + i * data.angle_increment])
        #                     for i in range(len(ranges))])

        # # right, left wall lines
        # splits = np.array_split(r_theta_ranges, 2)
        # splits = [list(filter(lambda x : x[0] < self.DESIRED_DISTANCE + self.FRONT_BUFFER, split)) for split in splits]

        # # x, y coordinates
        # xy_ranges = np.array([np.array([np.array([p[0] * np.cos(p[1]), p[0] * np.sin(p[1])]) for p in split])
        #                         for split in splits])

        # return xy_ranges[1]
    
    # def check_front(self, msg):
    #     # make sure that the three points directly in front of the robot are further than desired dist
    #     N = len(msg.ranges)
    #     front_dists = np.asarray(msg.ranges[N//2-1:N//2+1])
    #     avg_dist_to_front = front_dists.mean()

    #     if avg_dist_to_front < self.FRONT_BUFFER:
    #     #if avg_dist_to_front < self.DESIRED_DISTANCE + self.FRONT_BUFFER:
    #         new_str_angle = -0.34 if self.SIDE == 1 else 0.34
	#     #rospy.loginfo("Something in front %f", avg_dist_to_front)
    #         return new_str_angle
    #     return None

if __name__ == "__main__":
    rospy.init_node('lane_follower')
    lane_follower = LaneFollower()
    rospy.spin()