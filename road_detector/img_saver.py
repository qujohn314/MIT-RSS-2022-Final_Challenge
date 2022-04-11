from numpy import save
import rospy
from sensor_msgs.msg import Image
import numpy as np
import PIL.Image as pimage
import cv2

image_list = []

if __name__ == "__main__":
    rospy.init_node("img_saver")

    def process_imgs():
        saved_imgs = image_list[::10]
        for i, img in enumerate(saved_imgs):
            im = pimage.fromarray(img)
            # TODO: Change below path
            im.save(
                "/path/to/ml-final-challenge/saved_images/"
                + str(i) + ".png")
        print("finished saving!")
        return

    def cb(msg):
        # TODO: If camera is upside-down, set flip = True, else leave as False
        flip = False
        # TODO: Set total_images = #images in the rosbag you recorded. Get this info
        # with rosbag info [rosbag name]
        total_images = None

        np_img = np.frombuffer(msg.data,
                               dtype=np.uint8).reshape(msg.height, msg.width,
                                                       -1)[:, :, :-1]
        np_img = cv2.cvtColor(np_img, cv2.COLOR_BGR2RGB)
        if flip:
            np_img = np.copy(np_img[::-1, ::-1])
        else:
            np_img = np.copy(np_img)
        image_list.append(np_img)
        print(len(image_list))
        if len(image_list) == total_images:
            process_imgs()

    # TODO: Modify the topic to whatever camera feed you subscribed to
    # (the one below is the default, so you might not need to change it).
    sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color",
                           Image,
                           cb,
                           queue_size=10)

    rospy.spin()