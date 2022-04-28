#!/usr/bin/env python2

import numpy as np
import rospy

from lane_detection.msg import LaneLines, LaneLocation
from ackermann_msgs.msg import AckermannDriveStamped

class LaneFollower:
    LANE_TOPIC = rospy.get_param("/ground_lane_lines")
    DRIVE_TOPIC = rospy.get_param("drive_topic")
    SIDE = 1 # +1 represents left, -1 represents right
    VELOCITY = 2.0
    DESIRED_DISTANCE = 0.5 # manually adjusted (track lanes are 1.22m wide)

    TURNING_RADIUS = 0.325/np.tan(0.34)
    FRONT_BUFFER = 0.7 # car needs 0.62m to turn when wall is in front, added .7 to be safe
    #FRONT_BUFFER = 1
    previous_error = 0

    # more conservative
    # k_p = 5
    # k_d = 3
    k_p = 0.8
    k_d = 0.2

    def __init__(self):
        rospy.init_node('lane_follower')
        self.lane_subscriber = rospy.Subscriber(self.LANE_TOPIC, LaneLocation, self.lane_callback)
        self.steering_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def lane_callback(self, msg):
        x1 = msg.x1_ground
        y1 = msg.y1_ground
        x2 = msg.x2_ground
        y2 = msg.y2_ground
        steering_angle = self.find_new_steering_angle(x1, y1, x2, y2)
        
        steering_msg = AckermannDriveStamped()
        steering_msg.header.stamp = rospy.Time.now()
        steering_msg.header.frame_id = 'base_link' 
        steering_msg.drive.steering_angle = steering_angle
        #steering_msg.drive.steering_angle = 0
        steering_msg.drive.speed = self.VELOCITY

        self.steering_publisher.publish(steering_msg)
        #self.error_publisher.publish(error)

    def find_new_steering_angle(self, x1, y1, x2, y2):
        m, b = self.get_slope_intercept(x1, y1, x2, y2)
        # self.visualize.plot_line(side_range[:, 0], y_line, self.line_pub, frame="/laser")

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
        

    def get_slope_intercept(self, x1, y1, x2, y2):
        if x1 - x2 == 0:
            m = 1000
        else: 
            m = float(y1 - y2)/float(x1 - x2)
        
        b = float(y1 - float(m*x1))

        return (m, b)

if __name__ == "__main__":
    rospy.init_node('lane_follower')
    lane_follower = LaneFollower()
    rospy.spin()