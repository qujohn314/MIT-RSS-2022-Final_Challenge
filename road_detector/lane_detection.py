#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError
from city_driving.msg import LaneLines#, CenterPixel
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file

class HomographyTransformer():
    """
    """
    def __init__(self):
        self.PTS_IMAGE_PLANE = [[260, 226],
                   [455, 242],
                   [174, 289],
                   [494, 290]] # dummy points
        ######################################################

        # PTS_GROUND_PLANE units are in inches
        # car looks along positive x axis with positive y axis to left

        ######################################################
        ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
        self.PTS_GROUND_PLANE = [[54.5, 13.5],
                            [40.5, -12],
                            [24, 13.5],
                            [24.5, -9]] # dummy points
        ######################################################                   
        self.METERS_PER_INCH = 0.0254

         #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.
        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.
        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y


class LaneDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_lane_lines (LaneLines) : the coordinates of the line in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.lane_pub = rospy.Publisher("/relative_lane_lines", LaneLines, queue_size=10)
        # self.lane_px_pub = rospy.Publisher("/center_pixel", CenterPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/zed/zed_node/rgb/lane_lines_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        # self.lane_lines_pub = rospy.Publisher("/zed/zed_node/rgb/image_bound_boxes", Image, queue_size=10)
        # self.lane_detect_pub_mask = rospy.Publisher("/zed/zed_node/rgb/image_mask",Image,queue_size=10)


    def calculate_slope(self, coordinates):
        if coordinates[0] - coordinates[2] == 0:
            slope = 1000
        else: 
            delta_y = coordinates[1] - coordinates[3]
            delta_x = coordinates[0] - coordinates[2]
            slope = float(delta_y)/float(delta_x)
        
        return slope

    def midpoint(self, x1, y1, x2, y2):
        return ((x1 + x2)/2, (y1 + y2)/2)


    def get_center_point(self, left_lane, right_lane):
        center_left = self.midpoint(left_lane[0], left_lane[1], left_lane[2], left_lane[3])
        center_right = self.midpoint(right_lane[0], right_lane[1], right_lane[2], right_lane[3])
        center_x, center_y = self.midpoint(center_left[0], center_left[1], center_right[0], center_right[1])

        return (int(center_x), int(center_y))


    def get_hough_lines(self, image):
        """
        Applies a mask, edge detection and hough transform to the roi of the image 

        Parameters:
            image (numpy array) - image ready in cv2 format

        Returns:
            linesP (numpy array) - array of lines found in form of (x1, y1, x2, y2)
        """
        # Create mask
        lower = np.array([180, 180, 180], dtype="uint8")
        upper = np.array([255, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)
        masked = cv2. bitwise_and(image, image, mask = mask)

        src = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Check if image is loaded fine
        if src is None:
            print ('Error opening image!')
            # print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
            return -1

        # Create ROI -- EDIT THIS for ZED camera!
        # width: 672 pixels
        # height: 376 pixels
        horizontal_crop = 0
        vertical_crop = 200 # 150
        roi = src[vertical_crop:450, horizontal_crop:] # 200
        dst = cv2.Canny(roi, 50, 200, None, 3)
        
        # Copy edges to the images that will display the results in BGR
        cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
        # Finds Hough Lines
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 30)

        return linesP, cdstP
    

    def get_lane_center(self, linesP, cdstP):
        """
        Gets the right and left lane from the set of hough lines

        Parameters: 
            linesP (list) - list of hough lines
        Returns:
            (center_x, center_y) (tuple) - center between two lanes
        """
        right_lanes = []
        left_lanes = []

        for i in range(0, len(linesP)):
            l = linesP[i][0]
            rospy.loginfo(self.calculate_slope(l))
            if 0.8 > self.calculate_slope(l) > 0.3:
                left_lanes.append(l)
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            
            if -0.8 < self.calculate_slope(l) < -0.3: 
                right_lanes.append(l)
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

        left_lane = np.average(np.array(left_lanes), axis=0).astype(int)
        cv2.line(cdstP, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0,255,0), 3, cv2.LINE_AA)
        right_lane = np.average(np.array(right_lanes), axis=0).astype(int)
        cv2.line(cdstP, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0,255,0), 3, cv2.LINE_AA)


        center_x, center_y = self.get_center_point(left_lane, right_lane)
        # rospy.loginfo((center_x, center_y))
        cv2.circle(cdstP, (center_x, center_y), 20, (255,0,0), -1)

        debug_msg = self.bridge.cv2_to_imgmsg(cdstP, "passthrough")
        self.debug_pub.publish(debug_msg)

        # convert to ground plane
        transform = HomographyTransformer()
        x1_ground, y1_ground = transform.transformUvToXy(center_x, center_y)

        #Return point
        if linesP is None:
            rospy.loginfo("No lines detected!")
        else:
            return center_x, center_y


    def image_callback(self, image_msg):
        """
        Takes in image message from camera and then publishes the inside lane in a LaneLines message
        """
        im = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        linesP, dst = self.get_hough_lines(im)
        center_x, center_y = self.get_lane_center(linesP, dst)

        lane_msg = LaneLines()
        lane_msg.x1 = center_x
        lane_msg.y1 = center_y
        lane_msg.x2 = 0
        lane_msg.y2 = 0

        self.lane_pub.publish(lane_msg)
        

if __name__ == '__main__':
    try:
        rospy.init_node('LaneDetector', anonymous=True)
        LaneDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



# #!/usr/bin/env python

# import numpy as np
# import rospy

# import cv2
# from cv_bridge import CvBridge, CvBridgeError
# from city_driving.msg import LaneLines#, CenterPixel
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point  # geometry_msgs not in CMake file

# # class HomographyTransformer():
# #     """
# #     """
# #     def __init__(self):
# #         self.PTS_IMAGE_PLANE = [[260, 226],
# #                    [455, 242],
# #                    [174, 289],
# #                    [494, 290]] # dummy points
# #         ######################################################

# #         # PTS_GROUND_PLANE units are in inches
# #         # car looks along positive x axis with positive y axis to left

# #         ######################################################
# #         ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# #         self.PTS_GROUND_PLANE = [[54.5, 13.5],
# #                             [40.5, -12],
# #                             [24, 13.5],
# #                             [24.5, -9]] # dummy points
# #         ######################################################                   
# #         self.METERS_PER_INCH = 0.0254

# #          #Initialize data into a homography matrix
# #          def __init__(self):

# #         np_pts_ground = np.array(PTS_GROUND_PLANE)
# #         np_pts_ground = np_pts_ground * METERS_PER_INCH
# #         np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

# #         np_pts_image = np.array(PTS_IMAGE_PLANE)
# #         np_pts_image = np_pts_image * 1.0
# #         np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

# #         self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

# #     def transformUvToXy(self, u, v):
# #         """
# #         u and v are pixel coordinates.
# #         The top left pixel is the origin, u axis increases to right, and v axis
# #         increases down.
# #         Returns a normal non-np 1x2 matrix of xy displacement vector from the
# #         camera to the point on the ground plane.
# #         Camera points along positive x axis and y axis increases to the left of
# #         the camera.
# #         Units are in meters.
# #         """
# #         homogeneous_point = np.array([[u], [v], [1]])
# #         xy = np.dot(self.h, homogeneous_point)
# #         scaling_factor = 1.0 / xy[2, 0]
# #         homogeneous_xy = xy * scaling_factor
# #         x = homogeneous_xy[0, 0]
# #         y = homogeneous_xy[1, 0]
# #         return x, y


# class LaneDetector():
#     """
#     A class for applying your cone detection algorithms to the real robot.
#     Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
#     Publishes to: /relative_lane_lines (LaneLines) : the coordinates of the line in the image frame (units are pixels).
#     """
#     def __init__(self):
#         # toggle line follower vs cone parker
#         self.LineFollower = False

#         # Subscribe to ZED camera RGB frames
#         self.lane_pub = rospy.Publisher("/relative_lane_lines", LaneLines, queue_size=10)
#         # self.lane_px_pub = rospy.Publisher("/center_pixel", CenterPixel, queue_size=10)
#         self.debug_pub = rospy.Publisher("/zed/zed_node/rgb/lane_lines_img", Image, queue_size=10)
#         self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
#         self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
#         # self.lane_lines_pub = rospy.Publisher("/zed/zed_node/rgb/image_bound_boxes", Image, queue_size=10)
#         # self.lane_detect_pub_mask = rospy.Publisher("/zed/zed_node/rgb/image_mask",Image,queue_size=10)

#     def calculate_slope(self, coordinates):
#         if coordinates[0] - coordinates[2] == 0:
#             slope = 1000
#         else: 
#             delta_y = coordinates[1] - coordinates[3]
#             delta_x = coordinates[0] - coordinates[2]
#             slope = float(delta_y)/float(delta_x)
        
#         return slope

#     def midpoint(self, x1, y1, x2, y2):
#         return ((x1 + x2)/2, (y1 + y2)/2)

#     def get_hough_lines(self, image):
#         """
#         Applies a mask, edge detection and hough transform to the roi of the image 

#         Parameters:
#             image (numpy array) - image ready in cv2 format

#         Returns:
#             linesP (numpy array) - array of lines found in form of (x1, y1, x2, y2)
#         """
#         # Create mask
#         lower = np.array([180, 180, 180], dtype="uint8")
#         upper = np.array([255, 255, 255], dtype="uint8")
#         mask = cv2.inRange(image, lower, upper)
#         masked = cv2. bitwise_and(image, image, mask = mask)

#         src = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
#         # Check if image is loaded fine
#         if src is None:
#             print ('Error opening image!')
#             # print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
#             return -1

#         # Create ROI -- EDIT THIS for ZED camera!
#         # width: 672 pixels
#         # height: 376 pixels
#         horizontal_crop = 0
#         vertical_crop = 200 # 150
#         roi = src[vertical_crop:450, horizontal_crop:] # 200
#         dst = cv2.Canny(roi, 50, 200, None, 3)
        
#         # Copy edges to the images that will display the results in BGR
#         cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
#         # Finds Hough Lines
#         # linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10) #og parameters
#         linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 30)

#         # #Graph lines (optional) for debugging
#         # if linesP is not None:
#         #     for i in range(0, len(linesP)):
#         #         l = linesP[i][0]
#         #         cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
        
#         # debug_msg = self.bridge.cv2_to_imgmsg(cdstP, "passthrough")
#         # self.debug_pub.publish(debug_msg)
#         right_lanes = []
#         left_lanes = []

#         for i in range(0, len(linesP)):
#             l = linesP[i][0]
#             rospy.loginfo(self.calculate_slope(l))
#             if 0.8 > self.calculate_slope(l) > 0.3:
#                 left_lanes.append(l)
#                 cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            
#             if -0.8 < self.calculate_slope(l) < -0.3: 
#                 right_lanes.append(l)
#                 cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

#         left_lane = np.average(np.array(left_lanes), axis=0).astype(int)
#         cv2.line(cdstP, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0,255,0), 3, cv2.LINE_AA)
#         right_lane = np.average(np.array(right_lanes), axis=0).astype(int)
#         cv2.line(cdstP, (right_lane[0], right_lane[1]), (right_lane[2], right_lane[3]), (0,255,0), 3, cv2.LINE_AA)


#         center_x, center_y = self.get_center_point(left_lane, right_lane)
#         # rospy.loginfo((center_x, center_y))
#         cv2.circle(cdstP, (center_x, center_y), 20, (255,0,0), -1)
#         # transform = HomographyTransformer()
#         # x1_ground, y1_ground = transform.transformUvToXy(center_x, center_y)

#         # #Publish relative xy position of object in real world
#         # relative_xy_msg.x1_pos = x1_ground
#         # relative_xy_msg.y1_pos = y1_ground
#         # relative_xy_msg.x2_pos = x2_ground
#         # relative_xy_msg.y2_pos = y2_ground

#         debug_msg = self.bridge.cv2_to_imgmsg(cdstP, "passthrough")
#         self.debug_pub.publish(debug_msg)

#         #Return lines
#         if linesP is None:
#             rospy.loginfo("No lines detected!")
#         else:
#             return linesP, dst

#     def get_center_point(self, left_lane, right_lane):
#         center_left = self.midpoint(left_lane[0], left_lane[1], left_lane[2], left_lane[3])
#         center_right = self.midpoint(right_lane[0], right_lane[1], right_lane[2], right_lane[3])
#         center_x, center_y = self.midpoint(center_left[0], center_left[1], center_right[0], center_right[1])

#         return (int(center_x), int(center_y))

#     def get_lane(self, linesP, dst):
#         """
#         Gets the inside lane from the set of hough lines

#         Parameters: 
#             linesP (list) - list of hough lines
#         Returns:
#             inside_line (numpy array) - line that represents the lane [x1, y1, x2, y2]
#         """
#         for i in range(0, len(linesP)):
#             l = linesP[i][0]
#             if abs(self.calculate_slope(l)) > .2: #gets rid of horizontal lines
#                 if l[1] > len(dst) - 100 or l[3] > len(dst) - 100:
#                     if l[0] < len(dst[0])/2 or l[2] < len(dst[0])/2:
#                         inside_lane = l
#                         # rospy.loginfo(inside_lane)
#                         return inside_lane

#         rospy.loginfo("Inside lane not detected")
#         return [0, 0, 0, 0]


#     def image_callback(self, image_msg):
#         """
#         Takes in image message from camera and then publishes the inside lane in a LaneLines message
#         """
#         im = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
#         linesP, dst = self.get_hough_lines(im)
#         inside_lane = self.get_lane(linesP, dst)

#         lane_msg = LaneLines()
#         lane_msg.x1 = inside_lane[0]
#         lane_msg.y1 = inside_lane[1]
#         lane_msg.x2 = inside_lane[2]
#         lane_msg.y2 = inside_lane[3]

#         self.lane_pub.publish(lane_msg)

# if __name__ == '__main__':
#     try:
#         rospy.init_node('LaneDetector', anonymous=True)
#         LaneDetector()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
