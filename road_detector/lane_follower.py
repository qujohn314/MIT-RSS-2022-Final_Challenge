#!/usr/bin/env python

import rospy
import numpy as np

from city_driving.msg import LaneLines
from ackermann_msgs.msg import AckermannDriveStamped

class LaneFollower():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_lane_lines", LaneLines, self.lane_callback)

        DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation"
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        # self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)

        self.follow_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.car_point = (0, 0) # add offset for center of rear axle
        self.car_length = 0.55
        self.follow_buffer = 0.8  # within buffer we want to brake
        self.starting_buffer = self.follow_buffer
 
        self.previous_angle = 0
        self.steering_angle = 0
        self.alpha = 0
        self.look_ahead_distance = 0
        self.error_tolerance = 3  # degrees
        self.rate = rospy.Rate(10) #hz

        self.successful_run = False

    def lane_callback(self, msg):
        self.relative_x = msg.x1 # + 0.26
        self.relative_y = msg.y1

        drive_cmd = AckermannDriveStamped()
        relative_angle = np.arctan2(self.relative_x, self.relative_y)

        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
        
        rospy.loginfo("PURE PURSUIT")
        # pure pursuit controller 
        drive_cmd.drive.speed = 3  # limit below 1 m/s for velocity
        alpha = np.arctan2(self.relative_y, self.relative_x)
        rospy.loginfo(drive_cmd.drive.speed)
        rospy.loginfo("ALPHA -------------------")
        rospy.loginfo(alpha)
        drive_cmd.drive.steering_angle = 0.75*np.arctan2(2 * self.car_length * np.sin(alpha), distance_to_cone)
       
        self.drive_pub.publish(drive_cmd)

    # def error_publisher(self):
    #     """
    #     Publish the error between the car and the cone. We will view this
    #     with rqt_plot to plot the success of the controller
    #     """
    #     error_msg = ParkingError()

    #     #################################

    #     # YOUR CODE HERE
    #     # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
    #     error_msg.x_error = self.relative_x
    #     error_msg.y_error = self.relative_y 
    #     error_msg.distance_error = np.sqrt(self.relative_x**2+self.relative_y**2)

    #     #################################
        
    #     self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


