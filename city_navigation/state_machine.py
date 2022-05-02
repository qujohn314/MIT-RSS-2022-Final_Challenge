#!/usr/bin/env python

import time
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from enum import Enum
from std_msgs.msg import Bool
from city_driving.msg import CarWashPixel

class State(Enum):
    STOPPED = 1
    DRIVING = 2
    CAR_WASH = 3


class StateMachine:
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")

        self.parking_controller_subscriber = rospy.Subscriber("/parking_controller_drive_cmd", AckermannDriveStamped, self.parking_callback)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.stop_sub = rospy.Subscriber("/should_stop", Bool, self.stop_callback)
        self.car_wash_sub = rospy.Subscriber("/relative_car_wash_px", CarWashPixel, self.car_wash_callback)
        self.parking_cmd = None
        self.state = State.DRIVING
        self.last_stop_time_start = None
        self.last_stop_time_stop = None

        self.stopping_distance = 0.75 
    
    def stop_callback(self, msg):
        now = rospy.Time.now()
        if self.last_stop_time_stop is not None:
            rospy.loginfo("Not Stopped for: " + str((now - self.last_stop_time_stop)/1000000000))
        # if stop sign is in range and the last time we stopped was over a second ago 
        if msg.data and (self.last_stop_time_start is None or (self.state != State.STOPPED and now - self.last_stop_time_stop > rospy.Duration(3))):
            self.state = State.STOPPED 
            self.last_stop_time_start = now
            rospy.loginfo("NEW STOP SIGN DETECTED!")
        self.run()

    def parking_callback(self, msg):
        self.parking_cmd = msg
        self.run()
    
    def car_wash_callback(self, msg):
        if msg.u >= -10000 and msg.v >= -10000:
            #self.state == CAR_WASH:
            rospy.loginfo("Found carwash")
        self.run()
        
    def run(self):
        rospy.loginfo(self.state)
        if self.state == State.DRIVING:
            if self.parking_cmd == None:
                rospy.loginfo("Waiting for command from parking controller ... ")
                time.sleep(2)
                return 
            cmd = self.parking_cmd
            rospy.loginfo("drive command set!")

        elif self.state == State.STOPPED:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.drive.speed = 0
            # cmd.drive.steering_angle_velocity = 0
            cmd.drive.acceleration = 0
            # cmd.drive.jerk = 0
            rospy.loginfo("Stopped for: " + str((cmd.header.stamp - self.last_stop_time_start)/1000000000))
            if cmd.header.stamp - self.last_stop_time_start > rospy.Duration(3):
                self.state = State.DRIVING
                self.last_stop_time_stop = rospy.Time.now()
                rospy.loginfo("Time expired transitioning states.")
            

        elif self.state == State.CAR_WASH:
            return 

        
        self.drive_pub.publish(cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('StateMachine', anonymous=True)
        StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
