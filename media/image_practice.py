#!/usr/bin/env python

import sys
import math
import cv2
import numpy as np
import argparse
import os
import scipy.ndimage as ndi
from PIL import Image

def calculate_slope(coordinates):
    if coordinates[0] - coordinates[2] == 0:
        slope = 1000
    else: 
        slope = (coordinates[1] - coordinates[3])/(coordinates[0] - coordinates[2])
    
    return slope

def main(argv):
    
    parser = argparse.ArgumentParser(description="Get some clicks")
    parser.add_argument("input_path", help="Path to input image file")

    args = parser.parse_args()
    input_file = args.input_path

    if not os.path.isfile(input_file):
        print("Error, need an input file.")
        sys.exit(1)
    
    # Loads an image and applies a mask to isolate white pixels
    image = cv2.imread(input_file, cv2.IMREAD_COLOR)
    lower = np.array([180, 180, 180], dtype="uint8")
    upper = np.array([255, 255, 255], dtype="uint8")
    mask = cv2.inRange(image, lower, upper)
    masked = cv2. bitwise_and(image, image, mask = mask)
    # im = Image.fromarray(masked).convert('RGB')
    # im.save('masked.png')

    src = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
    # src = cv2.imread(masked, cv2.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1

    horizontal_crop = 1000
    vertical_crop = 1500
    roi = src[vertical_crop:, horizontal_crop:horizontal_crop+1800]
    im = Image.fromarray(roi).convert('RGB')
    im.save('roi.png')
    dst = cv2.Canny(roi, 50, 200, None, 3)
    
    # Copy edges to the images that will display the results in BGR
    cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    
    # Finds Hough Lines
    # linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 100, 30)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            if abs(calculate_slope(l)) > 1: #gets rid of horizontal lines
                cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)

    # cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            if abs(calculate_slope(l)) > 1: #gets rid of horizontal lines
                if l[1] > len(dst) - 100 or l[3] > len(dst) - 100:
                    if l[0] < len(dst[0])/2 or l[2] < len(dst[0])/2:
                        left_lane = l
                        break

    cv2.line(cdstP, (left_lane[0], left_lane[1]), (left_lane[2], left_lane[3]), (0,255,0), 3, cv2.LINE_AA)
    im = Image.fromarray(cdstP).convert('RGB')
    im.save('masked_hough_lines.png')
    
    sys.exit(0)
    return 0
    
if __name__ == "__main__":
    main(sys.argv[1:])
