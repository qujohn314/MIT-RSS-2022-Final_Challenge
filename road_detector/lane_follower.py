#!/usr/bin/env python2

# import numpy as np
# import rospy

# from lane_detection.msg import LaneLines, LaneLocation
# from ackermann_msgs.msg import AckermannDriveStamped

# class LaneFollower:
#     LANE_TOPIC = rospy.get_param("/ground_lane_lines")
#     DRIVE_TOPIC = rospy.get_param("drive_topic")
#     SIDE = 1 # +1 represents left, -1 represents right
#     VELOCITY = 2.0
#     DESIRED_DISTANCE = 0.5 # manually adjusted (track lanes are 1.22m wide)

#     TURNING_RADIUS = 0.325/np.tan(0.34)
#     FRONT_BUFFER = 0.7 # car needs 0.62m to turn when wall is in front, added .7 to be safe
#     #FRONT_BUFFER = 1
#     previous_error = 0

#     # more conservative
#     # k_p = 5
#     # k_d = 3
#     k_p = 0.8
#     k_d = 0.2

#     def __init__(self):
#         rospy.init_node('lane_follower')
#         self.lane_subscriber = rospy.Subscriber(self.LANE_TOPIC, LaneLocation, self.lane_callback)
#         self.steering_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

#     def lane_callback(self, msg):
#         x1 = msg.x1_ground
#         y1 = msg.y1_ground
#         x2 = msg.x2_ground
#         y2 = msg.y2_ground
#         steering_angle = self.find_new_steering_angle(x1, y1, x2, y2)
        
#         steering_msg = AckermannDriveStamped()
#         steering_msg.header.stamp = rospy.Time.now()
#         steering_msg.header.frame_id = 'base_link' 
#         steering_msg.drive.steering_angle = steering_angle
#         #steering_msg.drive.steering_angle = 0
#         steering_msg.drive.speed = self.VELOCITY

#         self.steering_publisher.publish(steering_msg)
#         #self.error_publisher.publish(error)

#     def find_new_steering_angle(self, x1, y1, x2, y2):
#         m, b = self.get_slope_intercept(x1, y1, x2, y2)
#         # self.visualize.plot_line(side_range[:, 0], y_line, self.line_pub, frame="/laser")

#         # closest distance from robot (0, 0) to wall 
#         closest_distance = np.abs(b) / np.sqrt((m**2) + 1.0)

#         # use a pd controller to find steering angle
#         error = closest_distance - self.DESIRED_DISTANCE
#         derivative = (error - self.previous_error)
#         steering_angle = np.arctan(self.k_p * error +
#                                 self.k_d * derivative) * self.SIDE + np.arctan(m)
#         self.previous_error = error
#         # self.error_publisher.publish(error)
#         return steering_angle
        

#     def get_slope_intercept(self, x1, y1, x2, y2):
#         if x1 - x2 == 0:
#             m = 1000
#         else: 
#             m = float(y1 - y2)/float(x1 - x2)
        
#         b = float(y1 - float(m*x1))

#         return (m, b)

# if __name__ == "__main__":
#     rospy.init_node('lane_follower')
#     lane_follower = LaneFollower()
#     rospy.spin()

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

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
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
 
        self.steering_angle = 0
        self.alpha = 0
        self.look_ahead_distance = 0
        self.error_tolerance = 3  # degrees

        self.successful_run = False

    def lane_callback(self, msg):
        self.relative_x = msg.x1 + 0.26
        self.relative_y = msg.y1

        drive_cmd = AckermannDriveStamped()
        relative_angle = np.arctan2(self.relative_y, self.relative_x)
        # rospy.loginfo(relative_angle)
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2)
	    # rospy.loginfo("Distance to cone: %f", distance_to_cone)
        if(0.5 <= distance_to_cone <= 0.8 and -self.error_tolerance*2*np.pi/360 <= relative_angle <= self.error_tolerance*2*np.pi/360):
            rospy.loginfo("Complete!")
            # drive_cmd.drive.speed = 0
        # if(-np.pi/4 <= relative_angle <= np.pi/4):
        #     self.relative_x=None
        #     self.relative_y=None

        # elif self.relative_x == -100000 or self.relative_y == -100000: # if we cant see cone, assuming relative pos are None
        #     # move in a circle 
        #     rospy.loginfo("move in a circle")
        #     drive_cmd.drive.speed = 1
        #     drive_cmd.drive.steering_angle = 0.34 # probably the max steering angle

        # elif distance_to_cone < self.starting_buffer: # check if within desired distance, back up if not 
        #     rospy.loginfo("Within desired distance of cone, backing up")
        #     drive_cmd.drive.speed = -1
        #     drive_cmd.drive.steering_angle = 0
        
        else:
            rospy.loginfo("PURE PURSUIT")
            # pure pursuit controller 
            drive_cmd.drive.speed = 1  # limit below 1 m/s for velocity
            alpha = np.arctan2(self.relative_y, self.relative_x)
            drive_cmd.drive.steering_angle = np.arctan2(2 * self.car_length * np.sin(alpha), distance_to_cone)
       
        # drive_cmd.drive.steering_angle_velocity = 0
        # drive_cmd.drive.acceleration = 0
        # drive_cmd.drive.jerk = 0
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