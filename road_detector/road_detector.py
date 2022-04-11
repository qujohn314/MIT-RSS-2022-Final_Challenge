#!/home/.yolovenv/bin/python

import cv2
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image as ImageMsg
import torch
from __utils__ import *
import torch.nn as nn
from model import RoadDetectorNet


class RoadDetector:
    def __init__(self):
        rospy.logerr("start init")
        # TODO: Modify path to weights
        self.model = torch.load(path_to_model_weights)
        self.subscriber = rospy.Subscriber(
            "/zed/zed_node/rgb/image_rect_color", ImageMsg, self.callback)
        self.publisher = rospy.Publisher("/road_mask", ImageMsg, queue_size=10)

        self.H, self.W = patch_H, patch_W
        rospy.logerr("done init")

    def callback(self, img_msg):
        np_img = np.frombuffer(img_msg.data,
                               dtype=np.uint8).reshape(img_msg.height,
                                                       img_msg.width, -1)
        bgr_img = np_img[:, :, :-1]
        # Reshape for velodyne
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)[::-1, ::-1]
        temp_shape = rgb_img.shape
        rgb_img = rgb_img.reshape(temp_shape[-1], temp_shape[0], temp_shape[1])

        rgb_img_tensor = torch.tensor(rgb_img.copy())
        split = split_img(rgb_img_tensor,
                          (self.H, self.W)).type(torch.FloatTensor)

        output = self.model(split.reshape([-1, 3, self.H, self.W])).reshape(
            [split.shape[0], split.shape[1], 2])
        mask = torch.argmax(output, dim=-1)

        new_msg = ImageMsg()
        new_msg.height, new_msg.width = split.shape[0], split.shape[1]
        new_msg.data = list(mask.reshape(-1))

        self.publisher.publish(new_msg)
        return


if __name__ == "__main__":
    rospy.init_node("road_detector")
    road_detector = RoadDetector()
    rospy.spin()
