import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


from detector import StopSignDetector

from ackermann_msgs.msg import AckermannDriveStamped

from homography_transformer import HomographyTransformer

class SignDetector:
    def __init__(self):
        self.stopping_distance = 1
        self.stopping_buffer = 0 # if we need more of a buffer in order to stop 
        self.pole_sign_ratio = 0.3
        self.homography_transformer = HomographyTransformer()

        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/should_stop", Bool, queue_size=10)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)
        

    def callback(self, img_msg):
        ret = Bool()
        ret.data = False

        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_stop, bounding_box = self.detector.predict(rgb_img)
        if is_stop: 
            # calculated distance to the stop sign, if within stopping distance publish bool command
            xmin, ymin, xmax, ymax = bounding_box
            sign_height = ymax - ymin
            base_v = ymin - sign_height * self.pole_sign_ratio
            base_u = (xmax - xmin)//2
            x, y = self.homography_transformer.transformUvToXy(base_u, base_v)
            # FOR NOW: distance from the robot's camera is the x value 
            if x > self.stopping_distance + self.stopping_buffer:
                ret.data = True 
        

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
