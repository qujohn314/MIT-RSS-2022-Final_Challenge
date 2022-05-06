#!/usr/bin/env python

import time
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from enum import Enum
from std_msgs.msg import Bool
from city_driving.msg import CarWashPixel
from visual_servoing.msg import ConeLocation
from homography_transformer import HomographyTransformer

class State(Enum):
    STOPPED = 1
    DRIVING = 2
    CAR_WASH = 3

class Car_Wash_State(Enum):
    INACTIVE = 1
    ENTERING = 2
    WALL_FOLLOW = 3


class StateMachine:
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.orange_line_sub = rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)
        self.wall_follower_sub = rospy.Subscriber("/wall_following_drive", AckermannDriveStamped, self.wall_following_callback)

        self.parking_controller_subscriber = rospy.Subscriber("/parking_controller_drive_cmd", AckermannDriveStamped, self.parking_callback)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.stop_sub = rospy.Subscriber("/should_stop", Bool, self.stop_callback)
        self.car_wash_sub = rospy.Subscriber("/relative_car_wash_px", CarWashPixel, self.car_wash_callback)
        self.parking_cmd = None
        self.state = State.DRIVING
        self.car_wash_state = Car_Wash_State.INACTIVE
        self.road_detetced = False
        self.last_stop_time_start = None
        self.last_stop_time_stop = None 
        self.car_length = 0.55

        self.homography_transformer = HomographyTransformer()

        self.pre_stopped_state = State.DRIVING

        self.car_wash_x = None
        self.car_wash_y = None
        self.wall_follow_cmd = None
    
    def stop_callback(self, msg):
        now = rospy.Time.now()
        if self.last_stop_time_stop is not None:
            rospy.loginfo("Not Stopped for: " + str((now - self.last_stop_time_stop)/1000000000))
        # if stop sign is in range and the last time we stopped was over a second ago 
        if msg.data and (self.last_stop_time_start is None or (self.state != State.STOPPED and now - self.last_stop_time_stop > rospy.Duration(2.5))):
            self.pre_stopped_state = self.state
            self.state = State.STOPPED 
            self.last_stop_time_start = now
            rospy.loginfo("NEW STOP SIGN DETECTED!")
        self.run()

    def parking_callback(self, msg):
        self.parking_cmd = msg
        self.run()
    
    def car_wash_callback(self, msg):
        if msg.u >= -10000 and msg.v >= -10000 and self.state != State.CAR_WASH:
            if self.car_wash_state == Car_Wash_State.INACTIVE:
                self.car_wash_state = Car_Wash_State.ENTERING
                self.state = State.CAR_WASH

                x, y = self.homography_transformer.transformUvToXy(msg.u, msg.v)

                self.car_wash_x = x
                self.car_wash_y = y

        elif msg.u >= -10000 and msg.v >= -10000 and self.state == State.CAR_WASH and self.car_wash_state == Car_Wash_State.ENTERING:
            x, y = self.homography_transformer.transformUvToXy(msg.u, msg.v)

            self.car_wash_x = x
            self.car_wash_y = y

        elif self.state == State.CAR_WASH and self.car_wash_state == Car_Wash_State.ENTERING and msg.u <= -10000 and msg.v <= -10000:
            self.car_wash_state = Car_Wash_State.WALL_FOLLOW
        
        self.run()

        
        

    def relative_cone_callback(self, msg):
        relative_x = msg.x_pos + 0.26 #What is this 0.26?
        relative_y = msg.y_pos
        if relative_x > -10000 or relative_y > -10000:
            self.road_detetced = True
        else:
            self.road_detetced = False

    def wall_following_callback(self, msg):
        if self.state == State.CAR_WASH and self.car_wash_state == Car_Wash_State.WALL_FOLLOW:
            self.wall_follow_cmd = msg
        
        if self.state == State.CAR_WASH and self.car_wash_state == Car_Wash_State.WALL_FOLLOW and self.road_detetced == True:
            self.car_wash_state = Car_Wash_State.INACTIVE
            self.state = self.pre_stopped_state

        self.run()

    def run(self):
        cmd = AckermannDriveStamped()
        rospy.loginfo(self.state)
        if self.state == State.DRIVING:
            if self.parking_cmd == None:
                rospy.loginfo("Waiting for command from parking controller ... ")
                time.sleep(2)
                return 
    
            cmd = self.parking_cmd

        elif self.state == State.STOPPED:
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.drive.speed = 0
            # cmd.drive.steering_angle_velocity = 0
            cmd.drive.acceleration = 0
            # cmd.drive.jerk = 0
            rospy.loginfo("Stopped for: " + str((cmd.header.stamp - self.last_stop_time_start)/1000000000))
            if cmd.header.stamp - self.last_stop_time_start > rospy.Duration(2.5):
                self.state = State.DRIVING
                self.last_stop_time_stop = rospy.Time.now()
                rospy.loginfo("Time expired transitioning states.")

        elif self.state == State.CAR_WASH:  
            if self.car_wash_state == Car_Wash_State.ENTERING:

                relative_angle = np.arctan2(self.car_wash_x, self.car_wash_y)
                distance_to_cone = np.sqrt(self.car_wash_x**2 + self.car_wash_y**2)
                
                rospy.loginfo("PURE PURSUIT")
                # pure pursuit controller 
                cmd.drive.speed = 0.44  # limit below 1 m/s for velocity
                alpha = np.arctan2(self.car_wash_y, self.car_wash_x)

                cmd.drive.steering_angle = np.arctan2(2 * self.car_length * np.sin(alpha), distance_to_cone)

             elif self.car_wash_state == Car_Wash_State.WALL_FOLLOW:
                     cmd = self.wall_follow_cmd
             

        
        self.drive_pub.publish(cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('StateMachine', anonymous=True)
        StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
