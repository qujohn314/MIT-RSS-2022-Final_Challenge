import time
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped

class StateMachine:
    def __init__(self):
        DRIVE_TOPIC = rospy.get_param("~drive_topic")

        self.parking_controller_subscriber("/parking_controller_drive_cmd", AckermannDriveStamped, self.callback)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.stop_sub("/should_stop", Bool, self.stop_callback)
        self.parking_cmd = None
        self.stop = False
        self.last_stop_time = None
        self.car_wash = False # set to always false for now

        self.stopping_distance = 0.75 
        

    def stop_callback(self, msg):
        now = rospy.Time.now()

        # if stop sign is close and the last time we stopped was over a second ago 
        if msg.data and (last_stop_time == None or now - self.last_stop_time > rospy.Duration(1)):
            self.stop = True

    def parking_callback(self, msg):
        self.parking_cmd = msg
    
    def car_wash_callback(self, msg):
        pass 
        
    def callback(self, img_msg):
        if not self.stop:
            if not self.car_wash:
                if self.parking_cmd == None:
                    rospy.loginfo("Waiting for command from parking controller ... ")
                    time.sleep(2)
                    return 
                cmd = self.parking_cmd
            else:
                return # do whatever we have to do for car wash
        else:
            cmd = AckermannDriveStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "base_link"
            cmd.drive.speed = 0
            # cmd.drive.steering_angle_velocity = 0
            cmd.drive.acceleration = 0
            # cmd.drive.jerk = 0

            self.stop = False
            self.last_stop_time = cmd.header.stamp
        
        self.drive_pub.publish(cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('StateMachine', anonymous=True)
        StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
