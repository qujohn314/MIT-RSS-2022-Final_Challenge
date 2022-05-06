import cv2
import numpy as np
import pdb

import rospy
#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################
BGR = 0
RGB = 1
HSV = 2


def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def bounding_box_to_image(img, x, y, w, h):
        img_copy = img.copy()
        cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,255,0),2)
        return img_copy

def get_image_mask(img, image_format=0):
        img_copy = img.copy()
	bounding_box = ((0,0),(0,0))
        HSV_img = img_copy

        if image_format == RGB:
                HSV_img = cv2.cvtColor(img_copy,cv2.COLOR_RGB2HSV) 
        if image_format == BGR:
                HSV_img = cv2.cvtColor(img_copy,cv2.COLOR_BGR2HSV) 


        #Values for yellow-orange cones.
        #H(0-180) S(0-255) V(0-255)

        #low_range = np.array([8,227,128])
        #high_range = np.array([36,255,255])
	low_range = np.array([2, 140, 120])
	high_range = np.array([30, 255, 255])

        #Creating erosion kernel
        erosion_kernel = np.ones((4,4),np.uint8)
        dialation_kernel = np.ones((5,5),np.uint8)

        #Applying mask for specific HSV range then remove noise
        mask = cv2.inRange(HSV_img, low_range, high_range)
        #mask = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR) 
        #return cv2.cvtColor(HSV_img,cv2.COLOR_HSV2BGR)
        return mask


def cd_color_segmentation(img, template=None,image_format=0,low_range=None,high_range=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
                image_format: 0=BGR,1=RGB,2=HSV. 0 Default.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
        img_copy = img.copy()
	bounding_box = ((0,0),(0,0))
        HSV_img = img_copy

        if image_format == RGB:
                HSV_img = cv2.cvtColor(img_copy,cv2.COLOR_RGB2HSV) 
        if image_format == BGR:
                HSV_img = cv2.cvtColor(img_copy,cv2.COLOR_BGR2HSV) 


        #Values for yellow-orange cones.
        #H(0-180) S(0-255) V(0-255)

        if low_range is None:
                low_range = np.array([0,0,0])
        if high_range is None:
                high_range = np.array([180,255,255])

        #Creating erosion kernel
        erosion_kernel = np.ones((4,4),np.uint8)
        dialation_kernel = np.ones((5,5),np.uint8)

        #Applying mask for specific HSV range then remove noise
        mask = cv2.inRange(HSV_img, low_range, high_range)
	#rospy.loginfo(np.where(mask != 0))
        eroded_image = cv2.erode(mask,erosion_kernel)
        output_image = cv2.dilate(eroded_image,dialation_kernel,iterations=2)
	
        #Find contours
        _,contours,_ = cv2.findContours(output_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Visualize contour lines
        #cv2.drawContours(img_copy,contours,-1,(0,255,0),3)
	if len(contours) < 1:
		#print("No cone detected!")
		return None

        x, y, w, h = cv2.boundingRect(contours[0])
        #cv2.rectangle(img_copy,(x,y),(x+w,y+h),(0,255,0),2)

        #image_print(img_copy)

        bounding_box = ((x,y),(x+w,y+h))
	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
